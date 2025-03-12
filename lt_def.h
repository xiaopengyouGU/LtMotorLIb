#ifndef __LT_DEF_H__
#define __LT_DEF_H__

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
#define PWM_PERIOD					100000	/* 0.1ms */
#define PWM_MAX_VAL					100
#define PWM_MIN_VAL					0.1		/* dead region */

#define ENCODER_TOTAL_RESOLUTION	4*13
#define MOTOR_REDUCTION_RATIO		34
#define MOTOR_MAX_ANGLE				180		/* for steering engine */



#endif
