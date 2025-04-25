#ifndef __LT_MOTOR_LIB_H__
#define __LT_MOTOR_LIB_H__
	
#include <rtthread.h>
#include <rtdevice.h>
#include "lt_def.h"
#define GET_BIT(x,pos)		(1 & (x >> pos) )
void lt_motor_test(void);
/* declare special motor operators */
extern char* _status[4];
extern char* _type[4];
extern struct lt_motor_ops _motor_dc_ops;
//extern struct lt_motor_ops _motor_bldc_ops;
extern struct lt_motor_ops _motor_stepper_ops;

/************************* common functions ************************************/
float _constrains(float val, float up_limit, float down_limit);
float _constrains_dead_region(float val,float up_limit, float down_limit);
rt_uint8_t _get_rotation_dir(float *input);			
rt_uint8_t _get_quard(rt_int32_t x_pos, rt_int32_t y_pos);
/* check whether start pos and end pos in the same quarent and circle */
rt_uint8_t _check_pos(rt_int32_t x_start, rt_int32_t y_start, rt_int32_t x_end, rt_int32_t y_end);
/************************* common functions ************************************/

/*******************************************************************************/
struct lt_filter_object			/* low pass filter object */
{
	float Tf;
	float dt;
	float val_prev;				/* last filter value */
};
typedef struct lt_filter_object * lt_filter_t;

lt_filter_t lt_filter_create(float Tf, float dt);
void lt_filter_set_tf(lt_filter_t, float Tf);
void lt_filter_set_dt(lt_filter_t, float dt);
void lt_filter_set(lt_filter_t, float Tf, float dt);
void lt_filter_reset(lt_filter_t);
float lt_filter_process(lt_filter_t,float value);
rt_err_t lt_filter_delete(lt_filter_t);

/*******************************************************************************/

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
	float int_limit;	/* integral limit */
	float output_limit;	/* output limit */	
};
typedef struct lt_pid_object* lt_pid_t;

lt_pid_t lt_pid_create(float Kp, float Ki, float Kd, float dt);
rt_err_t lt_pid_delete(lt_pid_t pid);

void lt_pid_reset(lt_pid_t pid);
void lt_pid_set(lt_pid_t pid, float Kp, float Ki, float Kd);
void lt_pid_set_target(lt_pid_t pid, float target);
void lt_pid_set_dt(lt_pid_t pid, float dt);
void lt_pid_set_output_limit(lt_pid_t pid,float limit);
void lt_pid_set_int_limit(lt_pid_t pid,float limit);		/* set integral limit */

float lt_pid_get_control(lt_pid_t pid);
float lt_pid_control(lt_pid_t pid,float curr_val);
float lt_pid_incre_control(lt_pid_t pid, float curr_val);
/*******************************************************************************/


/***********************************************************/
/* driver part */
#define DRIVER_TYPE_UNKNOWN		0x00
#define DRIVER_TYPE_DC			0x01
#define DRIVER_TYPE_BLDC		0x02
#define DRIVER_TYPE_STEPPER		0x03
/* rotation direction */
#define ROT_FORWARD				0x01
#define ROT_REVERSAL			0x02
#define ROT_DEFAULT				0x00

#define PWM_NUM_1 0x01
#define PWM_NUM_2 0x02
#define PWM_NUM_3 0x03
#define PWM_NUM_4 0x04
#define PWM_NUM_5 0x05
#define PWM_CHANNEL_1 0x01
#define PWM_CHANNEL_2 0x02
#define PWM_CHANNEL_3 0x03
#define PWM_CHANNEL_4 0x04

#define PWM_PHASE_DEFAULT	0x00
#define PWM_PHASE_A	  		0x01
#define PWM_PHASE_B	  		0x02
#define PWM_PHASE_C	  		0x03

#define FLAG_DEFAULT			0x00
#define FLAG_CONFIG				0x01
#define FLAG_CONFIG_ACC			0x02
#define FLAG_CONFIG_INTERP		0x04

struct lt_driver_object
{
	/* three phase PWM A,B,C */
	struct rt_device_pwm* pwm_A;
	struct rt_device_pwm* pwm_B;
	struct rt_device_pwm* pwm_C;
	rt_uint8_t pwm_channel_A;
	rt_uint8_t pwm_channel_B;
	rt_uint8_t pwm_channel_C;
	/* control pins */
	rt_base_t forward_pin;
	rt_base_t reversal_pin;
	rt_base_t enable_pin;
	/* three phase have same period and pulse */
	
	rt_uint8_t type;
	const struct lt_driver_ops *ops;		/* driver control operators */
	void *user_data;
};
typedef struct lt_driver_object* lt_driver_t;

struct lt_driver_ops
{
	rt_err_t (*set_pins)(lt_driver_t driver);
	rt_err_t (*enable)(lt_driver_t driver,rt_uint8_t dir);				
	rt_err_t (*disable)(lt_driver_t driver);
};

lt_driver_t lt_driver_create(rt_uint8_t type);
rt_err_t lt_driver_set_pins(lt_driver_t driver,rt_base_t forward_pin,rt_base_t reversal_pin,rt_base_t enable_pin);
rt_err_t lt_driver_set_pwm(lt_driver_t driver,rt_uint8_t pwm_num,rt_uint8_t pwm_channel,rt_uint8_t phase);
rt_err_t lt_driver_set_output(lt_driver_t driver,rt_uint32_t period,float duty_cycle,rt_uint8_t phase);
rt_err_t lt_driver_enable(lt_driver_t driver,rt_uint8_t dir);
rt_err_t lt_driver_disable(lt_driver_t driver);
rt_err_t lt_driver_delete(lt_driver_t driver);

/*********************************************************/
/* sensor part */
#define SENSOR_TYPE_ENCODER		0x01
#define SENSOR_TYPE_MAGNETIC	0x02
#define SENSOR_TYPE_HALL		0x03
#define SENSOR_TYPE_UNKNOWN		0x00

#define SENSOR_NUM_ENCODER_1 0x01
#define SENSOR_NUM_ENCODER_2 0x02
#define SENSOR_NUM_ENCODER_3 0x03
#define SENSOR_NUM_ENCODER_4 0x04
#define SENSOR_NUM_ENCODER_5 0x05

struct lt_sensor_object
{
	rt_device_t dev;					
	rt_uint8_t type;
	rt_uint16_t resolution;
	rt_int32_t curr_val;					/* current value  */
	/* pins */
	rt_base_t pin_A;
	rt_base_t pin_B;
	rt_base_t pin_C;
	
	lt_filter_t lpf;						/* low pass filter */
	const struct lt_sensor_ops *ops;		/* sensor control operators */
	void *user_data;
};
typedef struct lt_sensor_object* lt_sensor_t;

struct lt_sensor_ops
{
	lt_sensor_t (*create)(rt_uint16_t sensor_num, rt_uint16_t resolution, rt_uint8_t type);
	rt_err_t (*set_pins)(lt_sensor_t sensor);
	float (*get_angle)(lt_sensor_t sensor);
	float (*get_velocity)(lt_sensor_t sensor,rt_uint32_t measure_time_us );
	rt_err_t (*calibrate)(lt_sensor_t sensor);
};

lt_sensor_t lt_sensor_create(rt_uint16_t sensor_num, rt_uint16_t resolution, rt_uint8_t type);
rt_err_t lt_sensor_set_pins(lt_sensor_t sensor,rt_base_t pin_A,rt_base_t pin_B,rt_base_t pin_C );
float lt_sensor_get_angle(lt_sensor_t sensor);
float lt_sensor_get_velocity(lt_sensor_t sensor, rt_uint32_t measure_time_us);
rt_err_t lt_sensor_calibrate(lt_sensor_t sensor);
rt_err_t lt_sensor_delete(lt_sensor_t sensor);


/*******************************************************************************/
/* motor part!!! */
#define MOTOR_TYPE_DC		0x01
#define MOTOR_TYPE_BLDC		0x02
#define MOTOR_TYPE_STEPPER	0x03
#define MOTOR_TYPE_UNKNOWN	0x00

/* motor status */
#define MOTOR_STATUS_STOP				0x00
#define MOTOR_STATUS_RUN				0x01
#define MOTOR_STATUS_ACCELERATE			0x02
#define MOTOR_STATUS_INTERP				0x04
extern char * _status[4];

/* motor control command */
#define MOTOR_CTRL_OUTPUT				0x01
#define MOTOR_CTRL_OUTPUT_ANGLE			0x02
#define MOTOR_CTRL_GET_STATUS			0x03
#define MOTOR_CTRL_GET_VELOCITY			0x04
#define MOTOR_CTRL_GET_POSITION			0x05
#define MOTOR_CTRL_ENABLE_PID			0x06
#define MOTOR_CTRL_DISABLE_PID			0x07

struct lt_motor_ops;

/* define motor struct */
struct lt_motor_object{
	
	char  name[LT_NAME_MAX]; 
	rt_uint8_t reduction_ratio;
	lt_driver_t driver;	/* motor driver */
	lt_sensor_t sensor;	/* position sensor */
	rt_uint8_t type;
	rt_uint8_t status;					/* motor status */
	
	const struct lt_motor_ops *ops;		/* motor control operators */
	rt_err_t (*callback)(void*);		/* done callback function, used for accel and interp */
	void* user_data;					
};
typedef struct lt_motor_object* lt_motor_t;

struct lt_motor_info
{
	char name[LT_NAME_MAX];
	rt_uint8_t type;
	rt_uint8_t status;
	float position;
	//float velocity;
};
/* motor control operator */
struct lt_motor_ops
{	
	lt_motor_t(*create)(char* name,rt_uint8_t reduction_ration,rt_uint8_t type);
	rt_err_t (*control)(lt_motor_t motor, int cmd,void* arg);
	rt_err_t(*_delete)(lt_motor_t motor);
};

lt_motor_t lt_motor_create(char* name,rt_uint8_t reduction_ration,rt_uint8_t type);
rt_err_t lt_motor_set_driver(lt_motor_t motor, lt_driver_t driver);
rt_err_t lt_motor_set_sensor(lt_motor_t motor, lt_sensor_t sensor);
rt_err_t lt_motor_set_callback(lt_motor_t, rt_err_t (*callback)(void*) );

float lt_motor_get_velocity(lt_motor_t, rt_uint32_t measure_time_ms);
float lt_motor_get_position(lt_motor_t);
rt_err_t lt_motor_get_info(lt_motor_t, struct lt_motor_info*);
rt_err_t lt_motor_control(lt_motor_t, int cmd, void* arg);
rt_err_t lt_motor_enable(lt_motor_t,rt_uint8_t dir);
rt_err_t lt_motor_disable(lt_motor_t);
rt_err_t lt_motor_delete(lt_motor_t);					/* delete a motor! */

/*******************************************************************************/
/* stepper motor part */

#define STEPPER_CTRL_OUTPUT					MOTOR_CTRL_OUTPUT
#define STEPPER_CTRL_OUTPUT_ANGLE			MOTOR_CTRL_OUTPUT_ANGLE
#define STEPPER_CTRL_GET_STATUS				MOTOR_CTRL_GET_STATUS
#define STEPPER_CTRL_GET_VELOCITY			MOTOR_CTRL_GET_VELOCITY
#define STEPPER_CTRL_GET_POSITION			MOTOR_CTRL_GET_POSITION
#define STEPPER_CTRL_ENABLE_PID				MOTOR_CTRL_ENABLE_PID
#define STEPPER_CTRL_DISABLE_PID			MOTOR_CTRL_DISABLE_PID
/* basic control commanda are the same */
#define STEPPER_CTRL_CONFIG					(STEPPER_CTRL_DISABLE_PID + 0x10)
#define STEPPER_CTRL_TRAPZOID_ACCELERATE	(STEPPER_CTRL_DISABLE_PID + 0x13)
#define STEPPER_CTRL_S_CURVE_ACCELERATE		(STEPPER_CTRL_DISABLE_PID + 0x14)
#define STEPPER_CTRL_LINE_INTERPOLATION		(STEPPER_CTRL_DISABLE_PID + 0x15)
#define STEPPER_CTRL_CIRCULAR_INTERPOLATION	(STEPPER_CTRL_DISABLE_PID + 0x16)

#define TIMER_NUM_1 0x01
#define TIMER_NUM_2 0x02
#define TIMER_NUM_3 0x03
#define TIMER_NUM_4 0x04
#define TIMER_NUM_5 0x05

#define DIR_UNKNOWN	0
#define DIR_CW		1
#define DIR_CCW		2

struct lt_motor_stepper_object
{
	struct lt_motor_object parent;
	/* config structures */
	struct lt_stepper_config *config;
	struct lt_stepper_config_accel	*config_acc;
	struct lt_stepper_config_interp	*config_interp;
	rt_uint8_t config_flag;		/* config flag */
	
};
typedef struct lt_motor_stepper_object * lt_stepper_t;

struct lt_stepper_config
{
	rt_uint16_t period;			/* pulse period  ms */
	float stepper_angle;		/* unit: degree */
	rt_uint16_t subdivide;		/* subdivide number */
	rt_uint8_t timer_num;		/* hardware timer num */
	rt_uint32_t timer_freq;		/* hardware timer frequency */
	rt_device_t hw_timer;		/* hardware timer */
};

struct lt_stepper_config_accel
{
	int step;					/* +: forward, -: reversal */
	float accel;				/* accelerarte, unit : rad/s^2 */
	float decel;				/* decelerate, unit : rad/s^2 */
	float speed;				/* speed, unit : rad/s  */
	float freq_max;				/* max frequency */
	float freq_min;				/* min frequency */
	float flexible;				/* curve shape factor: flexible = 0 --> constant accel */
	rt_uint16_t index;	    	/* current index */
	rt_uint16_t max_index;		/* max index */
	rt_uint32_t *acc_series;	/* accel series */
};

struct lt_stepper_config_interp
{
	int x_start;
	int y_start;
	int x_end;
	int y_end;
	rt_uint8_t dir;				/* interpolation direction, 0: clockwise, 1: counter-clockwise */
	rt_uint32_t freq;
	rt_uint32_t num_pulse;
	lt_motor_t x_stepper;
	lt_motor_t y_stepper;
	int deviation;				/* position deviation */
	rt_uint8_t quadrant;		/* circular quadrant */
};


/*****************************************************************************************************/
/* motor mananger */
#define MANAGER_CTRL_SHOW_MOTOR			0x00
#define MANAGER_CTRL_SHOW_ALL_MOTORS	0x01

typedef struct lt_motor_node_object* lt_node_t;
struct lt_motor_node_object
{
	lt_node_t prev;
	lt_node_t next;
	lt_motor_t motor;
	char name[LT_NAME_MAX];
};

struct lt_motor_manager_object
{
	lt_node_t list;					/* motor list */
	struct lt_motor_info info;		/* motor information */
};
typedef struct lt_motor_manager_object* lt_manager_t;

/* when created a motor, it would be added to the manager, when deleting a motor, it would be removed from the manager */
/* there is only one motor manager */
int lt_manager_create(void);
rt_err_t lt_manager_add_motor(lt_motor_t motor);
rt_err_t lt_manager_delete_motor(lt_motor_t motor);
lt_motor_t lt_manager_get_motor(char* name);
rt_err_t lt_manager_delete(void);
/*****************************************************************************************************/
void test_motor_output(lt_motor_t motor,float val);
void test_motor_output_angle(lt_motor_t motor, float val);
void test_motor_get_position(lt_motor_t motor);
void test_motor_get_velocity(lt_motor_t motor);
void test_stepper_trapzoid(lt_motor_t motor, int step,float acc, float dec,float speed);
void test_stepper_s_curve(lt_motor_t motor, int step, float freq_max, float freq_min, float flexible);
void test_stepper_line_interp(lt_motor_t x_motor, lt_motor_t y_motor, int x_pos, int y_pos);
void test_stepper_circular_interp(lt_motor_t x_motor, lt_motor_t y_motor, int x_start, int y_start, int x_end, int y_end, rt_uint8_t dir);

#endif
