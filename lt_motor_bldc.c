#include "ltmotorlib.h"

static void _motor_bldc_output(lt_bldc_t, float input);
static void _motor_bldc_output_angle(lt_bldc_t,float angle);
static void _motor_bldc_config(lt_bldc_t bldc,struct lt_bldc_config* config);
static rt_uint8_t _bldc_check_end(lt_bldc_t bldc,float angle);	
static void _bldc_output(lt_bldc_t bldc, float input);

static lt_motor_t _motor_bldc_create(char* name,rt_uint8_t reduction_ration,rt_uint8_t type)
{
	lt_bldc_t _motor;
	_motor = rt_malloc(sizeof(struct lt_motor_bldc_object));
	if(_motor == RT_NULL) return RT_NULL;
	rt_memset(_motor,0,sizeof(struct lt_motor_bldc_object));
	rt_strcpy(_motor->parent.name,name);
	
	_motor->parent.reduction_ratio = reduction_ration;
	_motor->parent.type = type;
	_motor->parent.ops = &_motor_bldc_ops;		/* set operators */
	
	/* create foc object */
	_motor->foc = lt_foc_create();
	
	return (lt_motor_t)_motor;
}

static rt_err_t _motor_bldc_control(lt_motor_t motor, int cmd, void*arg)
{
	RT_ASSERT(motor != RT_NULL);
	lt_bldc_t bldc = (lt_bldc_t)motor;
	switch(cmd)
	{
		case MOTOR_CTRL_OUTPUT:
		{
			float input = *(float *)arg;
			_motor_bldc_output(bldc,input);
			break;
		}
		case MOTOR_CTRL_OUTPUT_ANGLE:
		{
			float angle = *(float *)arg;
			_motor_bldc_output_angle(bldc,angle);
			break;
		}
		case BLDC_CTRL_CONFIG:
		{
			struct lt_bldc_config* config = (struct lt_bldc_config*)arg;
			_motor_bldc_config(bldc,config);
		}
		default:break;
	}
	
	return RT_EOK;
}


static rt_err_t _motor_bldc_delete(lt_motor_t motor)
{
	lt_bldc_t bldc = (lt_bldc_t)motor;
	
	if(bldc->foc != RT_NULL)
	{
		lt_foc_delete(bldc->foc);
		bldc->foc = RT_NULL;
	}
	rt_free(bldc);
	
	return RT_EOK;
}

struct lt_motor_ops _motor_bldc_ops = {
										_motor_bldc_create,
										_motor_bldc_control,
										_motor_bldc_delete,
									 };

static void _bldc_open_loop_timeout(lt_timer_t timer)
{
	lt_bldc_t bldc = (lt_bldc_t)timer->user_data;
	lt_driver_t driver = bldc->parent.driver;
	lt_sensor_t sensor = bldc->parent.sensor;
	lt_foc_t foc = bldc->foc;
	rt_uint8_t res;
	float angle;
	float Uq = bldc->target_vel/bldc->KV/2;						/* get Uq, half of peak-to-peak value */
	float duty_A, duty_B, duty_C;
	
	/* disable output at first */
	lt_driver_disable(driver);
	angle = lt_sensor_get_angle(sensor);						/* get mechanical angle */
	
	if(bldc->flag & FLAG_BLDC_OPEN_POS)							/* open loop output angle */
	{
		res = _bldc_check_end(bldc,angle);
	}
	if(res)														/* reach target pos, stop output*/ 
	{
		lt_timer_disable(timer,TIMER_TYPE_HW);
		if(bldc->parent.callback)
		{
			bldc->parent.callback(RT_NULL);						/* call callback function */
		}
		bldc->parent.status = MOTOR_STATUS_STOP;
		return;
	}
	
	lt_foc_process(foc,0,Uq,angle*bldc->poles);					/* transform to electric angle */
	lt_foc_map_duty(foc,&duty_A,&duty_B,&duty_C);
	lt_driver_3pwm_output(driver,BLDC_OUTPUT_PERIOD,duty_A,duty_B,duty_C);		/* frequency: 10kHz <==> 100us */
}

static void _motor_bldc_output(lt_bldc_t bldc, float input)
{
	if(!(bldc->flag & FLAG_BLDC_CONFIG)) return;				/* not configured !!! */
	/* clear other flags at first */
	bldc->flag = FLAG_BLDC_CONFIG;
	_bldc_output(bldc,input);
}

static void _motor_bldc_output_angle(lt_bldc_t bldc,float angle)
{
	if(!(bldc->flag & FLAG_BLDC_CONFIG)) return;				/* not configured !!! */
	/* set flag at first */
	bldc->flag |= FLAG_BLDC_OPEN_POS;
	float curr_pos = lt_sensor_get_angle(bldc->parent.sensor);
	rt_int8_t dir;		

	/* compare current pos and disired pos */
	angle = angle * PI/180.0f;										/* degree to rad */
	bldc->target_pos = angle;										/* record target */
	if(curr_pos > angle)
	{
		bldc->flag |= FLAG_BLDC_OPEN_POS_BIAS;						/* 1: initial angle > target */
		dir = -1;													/* reversal rotation */
	}
	else if(curr_pos < angle)
	{
		bldc->flag = CLEAR_BIT(bldc->flag,FLAG_BLDC_OPEN_POS_BIAS);	/* initial angle < target */
		dir = 1;													/* forward rotation */
	}
	else
	{
		return;
	}
	
	_bldc_output(bldc,dir*BLDC_OPEN_SPEED);
}

static void _motor_bldc_config(lt_bldc_t bldc,struct lt_bldc_config* config)
{
	if(config->KV <= 0 || config->poles == 0 || config->max_volt == 0 ) return;
	
	bldc->inductance = config->inductance;
	bldc->max_volt = _absf(config->max_volt);
	bldc->poles = config->poles;
	bldc->resistance = config->resistance;
	bldc->KV = config->KV;
	bldc->pid_current = config->pid_current;
	bldc->current = config->current;
	bldc->flag = FLAG_BLDC_CONFIG;				/* set config! */
	
	lt_foc_set_maxval(bldc->foc,config->max_volt);
	lt_foc_set_type(bldc->foc,config->foc_type);
}

void _bldc_torque_timeout(lt_timer_t timer)
{
	lt_bldc_t bldc = (lt_bldc_t)timer->user_data;
	lt_driver_t driver = bldc->parent.driver;
	lt_pid_t pid = bldc->pid_current;
	lt_foc_t foc = bldc->foc;
	float angle_el = lt_sensor_get_angle(bldc->parent.sensor) * bldc->poles;
	float Iq  = lt_current_get_iq(bldc->current,angle_el,pid->dt*1000000);	/* unit: s --> us */
	float control_u;
	float duty_A, duty_B, duty_C;
	
	control_u = PID_CURR_CONST * lt_pid_control(pid,Iq);
	lt_foc_process(foc,0,control_u,angle_el);
	lt_foc_map_duty(foc,&duty_A,&duty_B,&duty_C);
	lt_driver_3pwm_output(driver,BLDC_OUTPUT_PERIOD,duty_A,duty_B,duty_C);		/* frequency: 10kHz <==> 100us */	
}

static rt_err_t _motor_bldc_output_torque(lt_bldc_t bldc, float input)
{
	if(!(bldc->flag & FLAG_BLDC_CONFIG)) return RT_ERROR;				/* not configured !!! */
	if(bldc->current == RT_NULL || bldc->pid_current == RT_NULL) return RT_ERROR;
	
	lt_timer_t timer = bldc->parent.timer;
	float Iq = input*bldc->KV/8.27;										/* get desired Iq, T[N.m] = 8.27*Iq[A]/KV */
	Iq = _constrains(Iq,BLDC_CURRENT_LIMIT, -BLDC_CURRENT_LIMIT);
	
	/* disable output at first */
	lt_driver_disable(bldc->parent.driver);
	lt_timer_disable(timer,TIMER_TYPE_HW);
	lt_pid_set_target(bldc->pid_current,Iq);							/* set target Iq */
	lt_timer_period_call(timer,BLDC_PERIOD,_bldc_torque_timeout,bldc,TIMER_TYPE_HW);
}

static rt_uint8_t _bldc_check_end(lt_bldc_t bldc,float angle)
{
	rt_uint8_t res;
	if(bldc->flag & FLAG_BLDC_OPEN_POS_BIAS)				/* 1: initial angle > target */
	{
		if(angle <= bldc->target_pos)
		{
			bldc->flag = FLAG_BLDC_CONFIG;					/* clear other flags */
			res = 1;
		}
	}
	else													/*  initial angle < target */
	{
		if(angle >= bldc->target_pos)
		{
			bldc->flag = FLAG_BLDC_CONFIG;					/* clear other flags */
			res = 1;
		}
	}
	
	return res;
}

static void _bldc_output(lt_bldc_t bldc, float input)
{
	float max_speed = bldc->max_volt*bldc->KV;	
	float duty_A, duty_B, duty_C;
	lt_driver_t driver = bldc->parent.driver;
	lt_timer_t timer = bldc->parent.timer;
	lt_foc_t foc = bldc->foc;
	/* check input boundary with dead region */
	input = _constrains(input,max_speed,-max_speed);
	/* disable output at first */
	lt_driver_disable(driver);
	lt_timer_disable(timer,TIMER_TYPE_HW);
	
	if(input == 0) 
	{
		lt_driver_disable(driver);	
		bldc->parent.status = MOTOR_STATUS_STOP;	/* change motor status */
	}
	else
	{
		bldc->target_vel = input;
		bldc->parent.status = MOTOR_STATUS_RUN;
	}
	lt_timer_period_call(timer,BLDC_PERIOD,_bldc_open_loop_timeout,bldc,TIMER_TYPE_HW);	/* start timer */
}