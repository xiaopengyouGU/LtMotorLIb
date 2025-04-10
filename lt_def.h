#ifndef __LT_DEF_H__
#define __LT_DEF_H__

#define LT_USING_MOTOR_TEST
#define PI							3.141593
#define PID_INTEGRAL_LIMIT			6000
#define PID_OUTPUT_LIMIT			2000

#define PULSE_ENCODER_DEV_NAME_1	"pulse4"
#define PULSE_ENCODER_DEV_NAME_2    "pulse4"	
#define PULSE_ENCODER_DEV_NAME_3	"pulse4"
#define PULSE_ENCODER_DEV_NAME_4	"pulse4"
#define PULSE_ENCODER_DEV_NAME_5	"pulse4"

#define PWM_DEV_NAME_1				"pwm2"
#define PWM_DEV_NAME_2				"pwm2"
#define PWM_DEV_NAME_3				"pwm2"
#define PWM_DEV_NAME_4				"pwm2"
#define PWM_DEV_NAME_5				"pwm2"

#define TIMER_DEV_NAME_1			"timer2"
#define TIMER_DEV_NAME_2			"timer2"
#define TIMER_DEV_NAME_3			"timer2"
#define TIMER_DEV_NAME_4			"timer2"
#define TIMER_DEV_NAME_5			"timer2"
/* DC motor definition parameter */
#define MOTOR_PERIOD				100000		/* 0.1ms, unit: ns */
#define MOTOR_MAX_SPEED				100			/* DC motor maximum speed */
#define MOTOR_MIN_SPEED				0.01		/* DC motor minimum speed, dead region */
#define MOTOR_MAX_ANGLE				180			/* for steering engine */
#define	MOTOR_ANGLE_PERIOD 			20000000	/* 20ms */
#define MOTOR_ANGLE_BASE			500000		/* 0.5ms */

/* stepper motor definition parameter */
#define STEPPER_MAX_SPEED			300			/* stepper motor maximum speed */
#define STEPPER_MIN_SPEED			0.1			/* stepper motor minimum speed, dead region */
#define STEPPER_MAX_PERIOD			1000000000	/* 1000ms, unit: ns */
#define STEPPER_DUTY_CYCLE			0.5			/* default duty cycle */


/* the ration between stepper's per circle pulses and encoder resolution */
#define PULSE_RATIO					(360.0f*STEPPER_SUBDIVIDE/STEPPER_ANGLE)/ENCODER_TOTAL_RESOLUTION_1
#define ENCODER_TOTAL_RESOLUTION_1  4*600			




#endif
