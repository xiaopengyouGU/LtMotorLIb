#ifndef __LT_MOTOR_CONTROL_H__
#define __LT_MOTOR_CONTROL_H__
	
#include <rtthread.h>
#include <rtdevice.h>
#include "lt_def.h"

void lt_motor_test(void);
/* declare special motor operators */
extern struct lt_motor_ops _motor_dc_ops;
extern struct lt_motor_ops _motor_bldc_ops;
extern struct lt_motor_ops _motor_stepper_ops;
/*******************************************************************************/
/* define pid object */
struct lt_pid_object
{
	float target_val;		
	float control_u;
	float err;
	float err_prev;
	float err_last;
	float Kp,Ki,Kd;
	float integral;
	float dt;			/* sample time */
};
typedef struct lt_pid_object* lt_pid_t;

lt_pid_t lt_pid_create(float Kp, float Ki, float Kd, float dt);
rt_err_t lt_pid_delete(lt_pid_t pid);

void lt_pid_reset(lt_pid_t pid);
void lt_pid_set(lt_pid_t pid, float Kp, float Ki, float Kd);
void lt_pid_set_target(lt_pid_t pid, float target);
void lt_pid_set_dt(lt_pid_t pid, float dt);

float lt_pid_get_control(lt_pid_t pid);
float lt_pid_control(lt_pid_t pid,float curr_val);
float lt_pid_incre_control(lt_pid_t pid, float curr_val);
/*******************************************************************************/

/*******************************************************************************/
/* motor part!!! */
#define MOTOR_TYPE_DC		0x01
#define MOTOR_TYPE_BLDC		0x02
#define MOTOR_TYPE_STEPPER	0x03
#define MOTOR_TYPE_UNKNOWN	0x00

#define PULSE_ENCODER_NUM_1 0x01
#define PULSE_ENCODER_NUM_2 0x02
#define PULSE_ENCODER_NUM_3 0x03
#define PULSE_ENCODER_NUM_4 0x04
#define PULSE_ENCODER_NUM_5 0x05

#define PWM_NUM_1 0x01
#define PWM_NUM_2 0x02
#define PWM_NUM_3 0x03
#define PWM_NUM_4 0x04
#define PWM_NUM_5 0x05
#define PWM_CHANNEL_1 0x01
#define PWM_CHANNEL_2 0x02
#define PWM_CHANNEL_3 0x03
#define PWM_CHANNEL_4 0x04

/* motor status */
#define MOTOR_STATUS_RUN				0x00
#define MOTOR_STATUS_STOP				0x01
#define MOTOR_STATUS_ACCELERATE			0x02
#define MOTOR_STATUS_DECELERATE			0x03

/* motor control command */
#define MOTOR_CTRL_OUTPUT				0x01
#define MOTOR_CTRL_OUTPUT_ANGLE			0x02
#define MOTOR_CTRL_MEASURE_SPEED		0x03
#define MOTOR_CTRL_MEASURE_POSITION		0x04

struct lt_motor_ops;

/* define motor struct */
struct lt_motor_object{
	rt_device_t encoder;
	struct rt_device_pwm* pwm;
	rt_uint16_t resolution;
	rt_uint8_t reduction_ratio;
	rt_uint8_t pwm_channel;
	
	rt_base_t forward_pin;
	rt_base_t reversal_pin;
	
	char  name[RT_NAME_MAX]; 
//	float speed;
//	float target_speed;
//	float position;
//	float target_position;
	rt_int32_t encoder_count;			/* prev encoder number */	
	rt_uint8_t type;
	rt_uint8_t status;					/* motor status */
	const struct lt_motor_ops *ops;		/* motor control operators */
	void* user_data;					
};
typedef struct lt_motor_object* lt_motor_t;

/* motor control operator */
struct lt_motor_ops
{	
	lt_motor_t(*create)(char* name,rt_uint8_t reduction_ration,rt_uint8_t type);
	rt_err_t (*control)(lt_motor_t motor, int cmd,void* arg);
	float (*measure_speed)(lt_motor_t motor,rt_uint32_t measure_time);
	float (*measure_position)(lt_motor_t motor);
};

lt_motor_t lt_motor_create(char* name,rt_uint8_t reduction_ration,rt_uint8_t type);
rt_err_t lt_motor_set_pwm(lt_motor_t motor, rt_uint16_t pwm_num,rt_uint8_t pwm_channel);
rt_err_t lt_motor_set_encoder(lt_motor_t motor, rt_uint16_t encoder_num,rt_uint16_t resolution);
rt_err_t lt_motor_set_dir_pins(lt_motor_t motor, rt_base_t forward_pin, rt_base_t reversal_pin);
rt_err_t lt_motor_set_ops(lt_motor_t motor, struct lt_motor_ops * ops);

float lt_motor_measure_speed(lt_motor_t, rt_uint32_t measure_time_ms);
float lt_motor_measure_position(lt_motor_t);
rt_err_t lt_motor_control(lt_motor_t, int cmd, void* arg);

/*******************************************************************************/
/* stepper motor part */

#define STEPPER_CTRL_OUTPUT					MOTOR_CTRL_OUTPUT
#define STEPPER_CTRL_OUTPUT_ANGLE			MOTOR_CTRL_OUTPUT_ANGLE
#define STEPPER_CTRL_MEASURE_SPEED			MOTOR_CTRL_MEASURE_SPEED
#define STEPPER_CTRL_MEASURE_POSITION		MOTOR_CTRL_MEASURE_POSITION
/* basic control commanda are the same */
#define STEPPER_CTRL_CONFIG					(MOTOR_CTRL_MEASURE_POSITION + 0x10)
#define STEPPER_CTRL_ENABLE					(MOTOR_CTRL_MEASURE_POSITION + 0x11)
#define STEPPER_CTRL_DISABLE				(MOTOR_CTRL_MEASURE_POSITION + 0x12)
#define STEPPER_CTRL_TRAPZOID_ACCELERATE	(MOTOR_CTRL_MEASURE_POSITION + 0x13)
#define STEPPER_CTRL_S_CURVE_ACCELERATE		(MOTOR_CTRL_MEASURE_POSITION + 0x14)
#define STEPPER_CTRL_LINE_INTERPOLATION		(MOTOR_CTRL_MEASURE_POSITION + 0x15)
#define STEPPER_CTRL_CIRCULAR_INTERPOLATION	(MOTOR_CTRL_MEASURE_POSITION + 0x16)

#define TIMER_NUM_1 0x01
#define TIMER_NUM_2 0x02
#define TIMER_NUM_3 0x03
#define TIMER_NUM_4 0x04
#define TIMER_NUM_5 0x05

#define STEPPER_INTERP_DIR_CW	1
#define STEPPER_INTERP_DIR_CCW	-1

struct lt_motor_stepper_object
{
	struct lt_motor_object parent;
	
	rt_ubase_t enable_pin;		/* enable pin */
	rt_uint8_t config_flag;		/* config flag */
	rt_uint16_t period;			/* pulse period ms */
	rt_uint16_t subdivide;		/* stepper angle subdivide */
	float stepper_angle;
	rt_timer_t soft_timer;		/* inner software timer of stepper motor */
	rt_uint32_t *accel_series;	/* accelerate series */
	rt_uint16_t index,max_index;/* series index */
	
	rt_device_t hw_timer;		/* hardware timer */
};
typedef struct lt_motor_stepper_object * lt_stepper_t;

struct lt_stepper_config
{
	rt_uint32_t enable_pin;		/* enable pin */
	rt_uint16_t period;			/* pulse period  ms */
	float stepper_angle;		/* unit: degree */
	rt_uint16_t subdivide;		/* subdivide number */
	rt_uint8_t timer_num;		/* hardware timer num */
	rt_uint32_t timer_freq;		/* hardware timer frequency */
};

struct lt_stepper_config_accel
{
	int step;				/* +: forward, -: reversal */
	rt_uint16_t accel;		/* accelerarte, unit : rpm/m */
	rt_uint16_t decel;		/* decelerate, unit : rpm/m */
	rt_uint16_t speed;		/* speed, unit : rpm */
	float freq_max;			/* max frequency */
	float freq_min;			/* min frequency */
	float flexible;			/* curve shape factor: flexible = 0 --> constant accel */
};

struct lt_stepper_config_interp
{
	rt_int32_t x_start;
	rt_int32_t y_start;
	rt_int32_t x_end;
	rt_int32_t y_end;
	rt_int8_t dir;				/* interpolation direction, 1: clockwise, -1: counter-clockwise */
	rt_uint32_t freq;
	rt_uint32_t num_pulse;
	lt_motor_t x_stepper;
	lt_motor_t y_stepper;
	rt_int32_t deviation;		/* position deviation */
	rt_uint8_t curr_axis;		/* current axis */
	rt_int8_t x_dir;			/* x move direction, 1: clockwise, -1: counter-clockwise */
	rt_int8_t y_dir;			/* y move direction */
	rt_uint8_t quadrant;			/* circular quadrant */
};


/*******************************************************************************/
struct lt_filter_object
{
	void(*process)(void* parameter);
	void(*create)(void *config);
};
typedef struct lt_filter_object * lt_filter_t;

lt_filter_t lt_filter_create(rt_uint8_t filter_type, void* config);
rt_err_t lt_filter_process(lt_filter_t);


/* define filter param */
struct lt_iir_filter_param
{
	float b0,b1,b2;			/* forward coefficient */
	float a1,a2;			/* feedback coefficient */
	float d1,d2;			/* delay element */
};


//lt_iir_filter_t lt_iir_filter_create(float b0, float b1, float b2, float a1,float a2);
//rt_err_t lt_iir_filter_process(lt_iir_filter_t* iir_filter);

/*******************************************************************************/

#endif
