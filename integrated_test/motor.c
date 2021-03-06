#include <p24FJ128GB206.h>
//k#include "config.h"
#include "common.h"
#include "ui.h"
#include "oc.h"
#include "pin.h"
#include "timer.h"
#include "motor.h"

Motor m1 = {{&D[7],&D[8]}, &oc7, &timer5, (int)32e3, 0, true};
Motor m2; // not used

Dir dir(float f){
	return f>0?FW:BK;
}

// convert float speed to uint16_t
uint16_t f2d(float f){
	float absf = f>0?f:-f;
	return absf * FULL_SPEED;
}

void motor_init(Motor* m){
	oc_pwm(m->oc, m->pins[FW], m->timer, m->freq, 0); 
	//oc_pwm(m->oc, m->pins[BK], m->timer, m->freq, 0); 
}

void drive(Motor* m, float speed){
	// speed = -1.0 ~ 1.0
	Dir m_d = dir(m->speed);
	Dir d = dir(speed);

	if(m_d != d){
		// flip direction ...
		pin_clear(m->pins[m_d]);
		oc_free(m->oc);
		oc_pwm(m->oc, m->pins[d], m->timer, m->freq, 0); 
	}

	m->speed = speed;
	uint16_t duty = f2d(m->speed);
	pin_write(m->pins[d],duty);
}

void brake(Motor* m){
	pin_write(m->pins[FW], 0);
	pin_write(m->pins[BK], 0);
}

/* MOTOR END */

/* Behaviors BEGIN */
void sweep(float* speed, float* ds, float lim){
	lim = lim>0?lim:-lim;

	*speed += *ds;
	if(*speed > lim){
		*ds = -*ds;
		*speed = lim;
	}else if (*speed < -lim){
		*ds = -*ds;
		*speed = -lim;
	}
}
/* Behaviors End */

void init_motors(){
	motor_init(&m1);
}

// int16_t main(void) {
//     init_clock();
//     init_ui();
//     init_timer();
//     init_pin();
//     init_oc();
//     init_motors();

//     led_on(&led2);
//     led_on(&led3);

//     timer_setPeriod(&timer1, .5);
//     timer_start(&timer1);

// 	float speed = 0.0;
// 	float ds = 0.02;
// 	float lim = 0.1;

// 	int cnt = 0;
//     while (1) {
//         if (timer_flag(&timer1)) {
//             timer_lower(&timer1);
// 			drive(&m1, speed);
// 			sweep(&speed,&ds,1.0);
//         }
//     }
// }
