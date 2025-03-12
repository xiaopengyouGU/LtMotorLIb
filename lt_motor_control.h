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
