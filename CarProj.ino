#include <Arduino.h>
#include <Wire.h>
#include <math.h>
//#include <U8g2lib.h>

#define max_size 20

template<typename T>
class Stack {
  private:
    T stack[max_size];
    unsigned int top;
  public:
    Stack() {top = 0;}
    bool push(const T &a);
    bool pop();
    T peek();
    int size() {return top;}
};

template<typename T>
bool Stack<T>::push(const T &a) {
    if (top >= max_size) {
        return false;
    } else {
        stack[top] = a;
        ++top;
        return true;
    }
}

template<typename T>
bool Stack<T>::pop() {
    if (top != 0) {
        --top;
        return true;
    } else {
        return false;
    }
}

template<typename T>
T Stack<T>::peek() {
    T res;
    if (top != 0) {
        res = stack[top-1];
    }
    return res;
}

enum DirType{
	FORWARD=0,
	BACKWARD,
	LEFT,
	RIGHT
};

enum Dir2Type{
	STOP=0,
	UP,
	DOWN
};

typedef unsigned long TimeType;

typedef struct {
    int x;
	int y;
    int theta;
	int speed;
	int delay;
	int up[2];
}TaskType;

typedef struct {
	int state=0;
	int recieve_state=0;
	int loop_time;

	int dir=-1;
	bool go=false;

	int upDir[2]={0,0};

	int speed=0;
	double x=0, y=0;
	double theta=0;
	double up[2]={0,0};

	int rx, ry,
		gx, gy,
		bx, by; //160x240
	int delTheta;

	int trace1=0,
		trace2=0,
		trace3=0;

	int task_delay=0;
	double task_dist=0;
	double task_delTheta=0;
}StateType;

// the unit is cm/ms
#define SPEED 255.0/250
#define SPEED_TURN 180.0/PI*255/250 /6.3
#define ACCELERATE 0.00001

#define tracing1 digitalRead(51)
#define tracing2 digitalRead(52)
#define tracing3 digitalRead(53)

void setup() {
	pinMode(2, OUTPUT); //left
	pinMode(3, OUTPUT);	//right

	pinMode(5, OUTPUT); //right ,low is back
	pinMode(6, OUTPUT);	//left

	pinMode(4, OUTPUT); //
	pinMode(12, OUTPUT); //

	pinMode(7, OUTPUT); //low is back
	pinMode(13, OUTPUT); //

	pinMode(51, INPUT);
	pinMode(52, INPUT);
	pinMode(53, INPUT);

	Serial1.begin(115200);
	Serial.begin(115200);
}

StateType state;

void setSpeed(int speed) {
	if(speed != state.speed) {
		OCR3A = speed;
		OCR3B = OCR3A / 2;
		OCR3C = OCR3A / 2;
		state.speed = speed;
	}
}

void up(int pin, int dir) { //pin0 is horizontal, pin1 is vertical
	if (state.upDir[pin] != dir) {
		state.upDir[pin] = dir;
		if (pin) {
			switch (dir) {
				case 0: digitalWrite(7, LOW);
						digitalWrite(4, LOW);
					break;
				case 1: digitalWrite(7, HIGH);
						analogWrite(4, 128);
					break;
				case 2: digitalWrite(7, LOW);
						analogWrite(4, 128);
					break;
				default: break;
			}
			//TCCR0B = (TCCR0B & 0xf8) | 2;
		} else {
			switch (dir) {
				case 0: digitalWrite(13, LOW);
						digitalWrite(12, LOW);
					break;
				case 1: digitalWrite(13, LOW);
						analogWrite(12, 128);
					break;
				case 2: digitalWrite(13, HIGH);
						analogWrite(12, 128);
					break;
				default: break;
			}
			TCCR1B = (TCCR0B & 0xf8) | 2;
		}
	}
}

void go(bool g) {
	if (g && !state.go) {
		TCCR3A = 0x2B;	//0010 1011
		TCCR3B = 0x1B;	//0001 1011

		//ICR3 = 255;
		setSpeed(255);
		state.go = true;
	}
	if (!g && state.go) {
		digitalWrite(2, LOW);
		digitalWrite(3, LOW);
		state.go = false;
	}
}

void setDir(int dir) {
	if (state.dir != dir) {
		state.dir = dir;
		switch (dir) {
			case 0: digitalWrite(5, LOW);
					digitalWrite(6, LOW);
					break;
			case 1: digitalWrite(5, HIGH);
					digitalWrite(6, HIGH);
					break;
			case 2: digitalWrite(5, HIGH);
					digitalWrite(6, LOW);
					break;
			case 3: digitalWrite(5, LOW);
					digitalWrite(6, HIGH);
					break;
			default: break;
		}
	}
}

double goodTheta(double theta) {
	while (theta > 180) {
		theta -= 360;
	}
	while (theta <= -180) {
		theta += 360;
	}
	return theta;
}

void justTurn(double delTheta) {
	delTheta = goodTheta(delTheta);
	if (delTheta > 0) {
		setDir(LEFT);
	} else if (delTheta < 0) {
		setDir(RIGHT);
	}
	go(true);
}

bool notEqual(double a, double b) {
	return abs(a - b) >= 0.1;
}

Stack<TaskType> tasks;

void autoSetSpeed(double dist) {
	static int last_time=0;
	int del_time = millis() - last_time;
	if (del_time > 100) {
		last_time = millis();
		double v_tar = sqrt(2 * ACCELERATE * dist + pow(SPEED / tasks.peek().speed, 2));
		//double arate = (pow(SPEED / tasks.peek().speed, 2) - pow(SPEED / state.speed, 2)) / 2 / dist;
		//if (abs(arate) > ACCELERATE) {
			//setSpeed((int)(SPEED / (SPEED / state.speed + del_time * arate)));
			//Serial.print(arate);
		//} else {
			if (v_tar >= SPEED / state.speed) {
				if (SPEED / state.speed + del_time * ACCELERATE > SPEED / 60) {
					setSpeed(60);
				} else {
					setSpeed((int)(SPEED / (SPEED / state.speed + del_time * ACCELERATE)));
				}
			} else {
				if (SPEED / state.speed - del_time * ACCELERATE < SPEED / 255) {
					setSpeed(255);
				} else {
					setSpeed((int)(SPEED / (SPEED / state.speed - del_time * ACCELERATE)));
				}
			}
		//}
	}
}

void autoSpeed() {
	if (state.dir <= 1) {
		autoSetSpeed(state.task_dist);
	} else {
		autoSetSpeed(state.task_delTheta * SPEED / SPEED_TURN);
	}
}

int SONG_TONE[]={212,200,190,212,159,169,212,200,190,212,142,159,
212,200,106,126,159,169,190,119,110,126,159,142,159};

int SONG_LONG[]={9,3,12,12,12,24,9,3,12,12,12,24,
9,3,12,12,12,12,12,9,3,12,12,12,24};

void loop()
{
	static int counter=0;
	counter++;
	int del_time = millis() - state.loop_time;
	state.loop_time = millis();

	if (Serial1.available() > 0) {
		int r = Serial1.read();
		// Serial1.write(r);
		switch (state.recieve_state) {
			case 0:
				if (r == 0x99) {
					state.recieve_state = 1;
				}
				break;
			case 1:
				state.delTheta = ((r - 128) + state.delTheta*9) / 10;
				state.recieve_state = 2;
				break;
			case 2:
				state.rx = r;
				state.recieve_state = 3;
				break;
			case 3:
				state.ry = r;
				state.recieve_state = 4;
				break;
			case 4:
				state.gx = r;
				state.recieve_state = 5;
				break;
			case 5:
				state.gy = r;
				state.recieve_state = 6;
				break;
			case 6:
				state.bx = r;
				state.recieve_state = 7;
				break;
			case 7:
				state.by = r;
				state.recieve_state = 0;
				break;
			default: break;
		}
	}

	if (state.state == 0 && millis() > 300) {
		TaskType goToPoint = {
			0, //x
			0, //y
			0, //theta
			255, //speed
			0, //delay
			{0, 0}	//24 is full
		};
		tasks.push(goToPoint);
		goToPoint = {-10, 0, 0, 255, 0, {0, 0}};
		tasks.push(goToPoint);
		goToPoint = {10, 10, 0, 255, 0, {0, 0}};
		tasks.push(goToPoint);
		goToPoint = {20, 0, 0, 255, 0, {0, 0}};
		tasks.push(goToPoint);
		state.state = 1;
	}

	if (state.state == 2) {
		setDir(FORWARD);
		go(true);
		for(int i=0; i<25; i++) {
			setSpeed(SONG_TONE[i]);
			for(int j=0; j<SONG_LONG[i]; j++) {
				_delay_ms(100);
			}
		}
	}

	if (!(counter%1000)) {
		Serial.print(state.x);
		Serial.print(',');
		Serial.print(state.y);
		Serial.print(',');
		Serial.print(state.theta);
		Serial.print(',');
		Serial.print(state.speed);
		Serial.print(',');
		Serial.print(state.up[0]);
		Serial.print(',');
		Serial.print(state.up[1]);
		Serial.print('\n');
	}

	if (state.state == 1) {
		if (state.go && state.speed != 0) {
			switch (state.dir) {
				case FORWARD:
					state.x += cos(state.theta) * del_time * SPEED / state.speed;
					state.y += sin(state.theta) * del_time * SPEED / state.speed;
					break;
				case BACKWARD:
					state.x -= cos(state.theta) * del_time * SPEED / state.speed;
					state.y -= sin(state.theta) * del_time * SPEED / state.speed;
					break;
				case LEFT:
					state.theta += del_time * SPEED_TURN / state.speed;
					state.theta = goodTheta(state.theta);
					break;
				case RIGHT:
					state.theta -= del_time * SPEED_TURN / state.speed;
					state.theta = goodTheta(state.theta);
					break;
				default: break;
			}
		}
		for (int i=0; i<2; i++) {
			switch (state.upDir[i]) {
				case UP:
					state.up[i] += del_time * SPEED / 255;
					break;
				case DOWN:
					state.up[i] -= del_time * SPEED / 255;
					break;
				default: break;
			}
		}
		if (tasks.size()) {
			int taskFinish = 0;
			if (state.task_delay == 0 && tasks.peek().delay != 0) {
				state.task_delay = millis();
			}
			if (tasks.peek().delay != 0 && (int)millis() < state.task_delay + tasks.peek().delay) {
			} else {
				if (notEqual(state.x, tasks.peek().x) || notEqual(state.y, tasks.peek().y)) {
					double taskTheta = atan2(tasks.peek().y - state.y, tasks.peek().x - state.x)*180/PI;
					if (notEqual(state.theta, taskTheta)) {
						state.task_delTheta = taskTheta - state.theta;
						justTurn(state.task_delTheta);
					} else {
						setDir(FORWARD);
						state.task_dist = sqrt(pow(state.x - tasks.peek().x, 2) + pow(state.y - tasks.peek().y, 2));
						go(true);
					}
					autoSpeed();
				} else if (notEqual(state.theta, tasks.peek().theta)) {
					state.task_delTheta = tasks.peek().theta - state.theta;
					justTurn(state.task_delTheta);
					autoSpeed();
				} else {
					taskFinish += 1;
					go(false);
				}
				for (int i=0; i<2; i++) {
					if (notEqual(tasks.peek().up[i], state.up[i])) {
						if (tasks.peek().up[i] > state.up[i]) {
							up(i, UP);
						} else {
							up(i, DOWN);
						}
					} else {
						up(i, STOP);
						taskFinish += 1;
					}
				}
				if (taskFinish == 3) {
					tasks.pop();
					state.task_delay = 0;
				}
			}
		}
	}
}
