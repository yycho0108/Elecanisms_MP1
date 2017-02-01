#ifndef __MOTOR_H__
#define __MOTOR_H__

#define FULL_SPEED (0xFFFF)
uint16_t f2d(float f);
/* MOTOR START */
typedef enum _Dir{FW,BK} Dir;
typedef enum {false,true} bool;

typedef struct _Motor{
 // parameters
	_PIN* pins[2];
	_OC* oc;
	_TIMER* timer;
	int freq;
// states
	float speed;
	bool braked;
} Motor;

void motor_init(Motor* m);
void drive(Motor* m, float speed);
void brake(Motor* m);
void sweep(float* speed, float* ds, float lim);

extern Motor m1, m2;

#endif