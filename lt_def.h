#ifndef __LT_DEF_H__
#define __LT_DEF_H__

#define SQRT_3						1.73205
#define GET_BIT(x,pos)		(1 & (x >> pos) )

#define LT_NAME_MAX					RT_NAME_MAX + 2
#define LT_USING_MOTOR_MSH_TEST
#define PI							3.141593
#define PID_INTEGRAL_LIMIT			6000
#define PID_OUTPUT_LIMIT			2000
#define PID_POS_CONST				0.01f
#define PID_VEL_CONST				0.1f

#define ENCODER_NAME_1			"pulse4"
#define ENCODER_NAME_2  		"pulse4"	
#define ENCODER_NAME_3			"pulse4"
#define ENCODER_NAME_4			"pulse4"
#define ENCODER_NAME_5			"pulse4"

#define I2C_NAME_1				"i2c1"
#define I2C_NAME_2				"i2c2"
#define I2C_NAME_3				"i2c2"
#define I2C_NAME_4				"i2c2"
#define I2C_NAME_5				"i2c2"


#define PWM_NAME_1				"pwm2"
#define PWM_NAME_2				"pwm2"
#define PWM_NAME_3				"pwm2"
#define PWM_NAME_4				"pwm2"
#define PWM_NAME_5				"pwm2"

#define TIMER_NAME_1			"timer11"
#define TIMER_NAME_2			"timer13"
#define TIMER_NAME_3			"timer14"
#define TIMER_NAME_4			"timer11"
#define TIMER_NAME_5			"timer11"

/* DC motor definition parameter */
#define MOTOR_PERIOD				100		    /* 0.1ms, unit: us */
#define MOTOR_MAX_SPEED				100			/* DC motor maximum speed */
#define MOTOR_MIN_SPEED				0.01		/* DC motor minimum speed, dead region */
#define MOTOR_MAX_ANGLE				180			/* for steering engine */
#define	MOTOR_ANGLE_PERIOD 			20000   	/* 20ms, unit: us */
#define MOTOR_ANGLE_BASE			500		    /* 0.5ms, unit: us */

/* stepper motor definition parameter */
#define STEPPER_MAX_SPEED			300			/* stepper motor maximum speed */
#define STEPPER_MIN_SPEED			0.1			/* stepper motor minimum speed, dead region */
#define STEPPER_MAX_PERIOD			1000000	    /* 1000ms, unit: us */
#define STEPPER_DUTY_CYCLE			0.5			/* default duty cycle */


/* the ration between stepper's per circle pulses and encoder resolution */
#define PULSE_RATIO					(360.0f*STEPPER_SUBDIVIDE/STEPPER_ANGLE)/ENCODER_TOTAL_RESOLUTION_1
#define ENCODER_TOTAL_RESOLUTION_1  4*600			




#endif
