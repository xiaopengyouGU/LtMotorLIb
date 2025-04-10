#include "lt_motor_control.h"
#include "math.h"
/* use sortware timer to help output given number pulse! */
static void _timer_output_angle(void *parameter)
{
	lt_stepper_t stepper = (lt_stepper_t)parameter;
	rt_pwm_disable(stepper->parent.pwm,stepper->parent.pwm_channel);	/* disable pwm output */
}

static void _motor_stepper_output_angle(lt_stepper_t stepper,float angle);
static void _motor_stepper_output(lt_stepper_t stepper, float input);
static void _motor_stepper_config(lt_stepper_t,struct lt_stepper_config* config);
static void _motor_stepper_trapzoid_accelerate(lt_stepper_t stepper,struct lt_stepper_config_accel* config);
static void _motor_stepper_s_curve_accelerate(lt_stepper_t stepper,struct lt_stepper_config_accel* config);
static void _motor_stepper_line_interp(lt_stepper_t stepper,struct lt_stepper_config_interp* config);
static void _motor_stepper_circular_interp(lt_stepper_t stepper,struct lt_stepper_config_interp* config);
//static rt_thread_t _process;			/* process thread for acceleration and  interplation */

static lt_motor_t _motor_stepper_create(char* name,rt_uint8_t reduction_ration,rt_uint8_t type)
{
	lt_stepper_t _motor;
	_motor = rt_malloc(sizeof(struct lt_motor_stepper_object));
	rt_memset(_motor,0,sizeof(struct lt_motor_stepper_object));
	rt_strcpy(_motor->parent.name,name);
	
	_motor->parent.reduction_ratio = reduction_ration;
	_motor->parent.type = type;
	_motor->parent.ops = &_motor_stepper_ops;/* set operators */
	return (lt_motor_t)_motor;				 /* type transform */
}

static float _motor_stepper_measure_speed(lt_motor_t motor, rt_uint32_t measure_time_ms)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(motor->encoder != RT_NULL);
	
	rt_device_t encoder = motor->encoder;
	rt_int32_t curr_count;
	float speed;
	
	rt_device_open(encoder,RT_DEVICE_OFLAG_RDONLY);
	rt_device_read(encoder,0,&curr_count,1);
	/* use M method to measure velocity */
	speed = (float)(motor->encoder_count - curr_count)*1000/measure_time_ms;
	speed /= (motor->resolution * motor->reduction_ratio);				/* result transform */
	/* refrech encoder_count! */
	motor->encoder_count = curr_count;

	return speed;
}

static float _motor_stepper_measure_position(lt_motor_t motor)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(motor->encoder != RT_NULL);
	
	rt_device_t encoder = motor->encoder;
	rt_int32_t count;
	float position;
	
	rt_device_open(encoder,RT_DEVICE_OFLAG_RDONLY);
	rt_device_read(encoder,0,&count,1);
	
	position = (float)(count)/(motor->resolution * motor->reduction_ratio);
	position *= 360;
	
	return position;
}


static rt_err_t _motor_stepper_control(lt_motor_t motor, int cmd, void*arg)
{
	RT_ASSERT(motor != RT_NULL);
	lt_stepper_t stepper = (lt_stepper_t)motor;			/* type transform */
	switch(cmd)
	{
		case MOTOR_CTRL_OUTPUT:
		{
			float input = *(float *)arg;
			_motor_stepper_output(stepper,input);
			break;
		}
		case MOTOR_CTRL_OUTPUT_ANGLE:
		{
			float angle = *(float *)arg;
			_motor_stepper_output_angle(stepper,angle);
			break;
		}
		case MOTOR_CTRL_MEASURE_SPEED:
		{
			float *res_ms = (float *)arg;
			*res_ms = _motor_stepper_measure_speed(motor,(rt_uint32_t)(*res_ms));
			break;
		}
		case MOTOR_CTRL_MEASURE_POSITION:
		{
			float *res = (float *)arg;
			*res = _motor_stepper_measure_position(motor);
			break;
		}
		case STEPPER_CTRL_CONFIG:		/* config stepper parameters */
		{
			struct lt_stepper_config*  config = (struct lt_stepper_config*)arg;
			_motor_stepper_config(stepper,config);
			break;
		}
		case STEPPER_CTRL_ENABLE:
		{
			rt_pin_write(stepper->enable_pin,PIN_HIGH);
			break;
		}
		case STEPPER_CTRL_DISABLE:
		{
			rt_pin_write(stepper->enable_pin,PIN_LOW);
			rt_pwm_disable(stepper->parent.pwm,stepper->parent.pwm_channel);
			stepper->parent.status = MOTOR_STATUS_STOP;
			break;
		}
		case STEPPER_CTRL_TRAPZOID_ACCELERATE:
		{
			struct lt_stepper_config_accel* config = (struct lt_stepper_config_accel*)arg;
			_motor_stepper_trapzoid_accelerate(stepper,config);
			break;
		}
		case STEPPER_CTRL_S_CURVE_ACCELERATE:
		{
			struct lt_stepper_config_accel* config = (struct lt_stepper_config_accel*)arg;
			_motor_stepper_s_curve_accelerate(stepper,config);
			break;
		}
		case STEPPER_CTRL_LINE_INTERPOLATION:
		{
			struct lt_stepper_config_interp* config = (struct lt_stepper_config_interp*)arg;
			_motor_stepper_line_interp(stepper,config);
			break;
		}
		case STEPPER_CTRL_CIRCULAR_INTERPOLATION:
		{
			struct lt_stepper_config_interp* config = (struct lt_stepper_config_interp*)arg;
			_motor_stepper_circular_interp(stepper,config);
			break;
		}
		default:break;
	}
	
	return RT_EOK;
		
}

struct lt_motor_ops _motor_stepper_ops = {
										_motor_stepper_create,
										_motor_stepper_control,
										_motor_stepper_measure_speed,
										_motor_stepper_measure_position,
									 };

static void _motor_stepper_output(lt_stepper_t stepper, float input)
{
	/* rotate at a constant speed */
	/* speed is proportional to frequency */
	if(!stepper->config_flag)	return;					/* not configured */
	rt_pin_write(stepper->enable_pin,PIN_LOW);			/* enable output */
	stepper->parent.status = MOTOR_STATUS_RUN;			/* change motor status */
	float output;
	
	if(input >= 0 )		/* forward rotation */
	{
		rt_pin_write(stepper->parent.forward_pin,PIN_HIGH);
		//rt_pin_write(motor->reversal_pin,PIN_LOW);
	}
	else if(input < 0)
	{					/* reversal rotation */
		input = -input;
		rt_pin_write(stepper->parent.forward_pin,PIN_LOW);
		//rt_pin_write(motor->reversal_pin,PIN_HIGH);
	}

	if(input > STEPPER_MAX_SPEED)			/* actuator saturation */
	{
		output = STEPPER_MAX_SPEED;
	}
	else if(input < STEPPER_MIN_SPEED)		/* dead region */
	{
		output = 0;
		stepper->parent.status = MOTOR_STATUS_STOP;
	}
	else
	{
		output = input;
	}
	
	if(output == 0)
	{
		rt_pwm_disable(stepper->parent.pwm,stepper->parent.pwm_channel);		/* no output */
		rt_pin_write(stepper->enable_pin,PIN_HIGH);								/* disable output */
	}
	else
	{
		output = STEPPER_MAX_PERIOD/output;										/* transform */
		if(output > STEPPER_MAX_PERIOD) output = STEPPER_MAX_PERIOD;			/* slowest rotation */
		rt_pwm_set(stepper->parent.pwm,stepper->parent.pwm_channel,(rt_uint32_t)output,(rt_uint32_t)(output*STEPPER_DUTY_CYCLE));
		rt_pwm_enable(stepper->parent.pwm,stepper->parent.pwm_channel);			/* output */
	}
}

static void _motor_stepper_output_angle(lt_stepper_t stepper, float angle)
{
	if(!stepper->config_flag) return;					/* stepper is not configured */
	
	rt_pin_write(stepper->enable_pin,PIN_LOW);			/* enable output */
	stepper->parent.status = MOTOR_STATUS_RUN;		/* change status */
	if(angle > 0)		/* forward rotation */
	{
		rt_pin_write(stepper->parent.forward_pin,PIN_HIGH);
		//rt_pin_write(stepper->parent.reversal_pin,PIN_LOW);
	}
	else if(angle < 0)
	{
		angle = - angle;
		rt_pin_write(stepper->parent.forward_pin,PIN_LOW);
		//rt_pin_write(stepper->parent.reversal_pin,PIN_HIGH);
	}
	else
	{
		rt_pin_write(stepper->parent.forward_pin,PIN_LOW);
		rt_pin_write(stepper->enable_pin,PIN_HIGH);		/* disable output */
		stepper->parent.status = MOTOR_STATUS_STOP;
		//rt_pin_write(stepper->parent.reversal_pin,PIN_LOW);
	}
	/* get number of square pulse */
	rt_uint32_t n = angle/(stepper->stepper_angle/stepper->subdivide);
	rt_pwm_disable(stepper->parent.pwm,stepper->parent.pwm_channel);		/* stop pwm output */
	/* create a software timer to help process angle output */			
	rt_uint32_t period = stepper->period*1000000;							/* unit: ns */	
	rt_tick_t time = stepper->period*n;										/* unit: tick or ms */	
	if(stepper->soft_timer != RT_NULL)											/* we already created a timer before */
	{
		rt_pwm_set(stepper->parent.pwm,stepper->parent.pwm_channel,period,period*STEPPER_DUTY_CYCLE);
		rt_pwm_enable(stepper->parent.pwm,stepper->parent.pwm_channel);
		rt_timer_control(stepper->soft_timer,RT_TIMER_CTRL_SET_TIME,&time);		/* change time */
		rt_timer_start(stepper->soft_timer);										/* start timer */		
	}
	else
	{	/* we have no timer, create one! */
		stepper->soft_timer = rt_timer_create(stepper->parent.name,_timer_output_angle,stepper,time,RT_TIMER_FLAG_ONE_SHOT);
		if(stepper->soft_timer != RT_NULL)
		{
			rt_pwm_set(stepper->parent.pwm,stepper->parent.pwm_channel,period,period*STEPPER_DUTY_CYCLE);
			rt_pwm_enable(stepper->parent.pwm,stepper->parent.pwm_channel);
			rt_timer_start(stepper->soft_timer);									/* start timer! */
		}
	}
}

static void _motor_stepper_config(lt_stepper_t stepper,struct lt_stepper_config* config)
{
	stepper->stepper_angle = config->stepper_angle;
	stepper->period = config->period;
	stepper->subdivide = config->subdivide;
	stepper->enable_pin = config->enable_pin;
	stepper->config_flag = 1;								/* finish config */
	rt_pin_mode(config->enable_pin,PIN_MODE_OUTPUT);		/* set enable pin */
	if(stepper->subdivide == 0)
		stepper->subdivide = 1;
	/* config hardware timer */
	rt_device_t _timer;
	switch(config->timer_num)
	{
		case TIMER_NUM_1:
			_timer = (rt_device_t)rt_device_find(TIMER_DEV_NAME_1);
			break;
		case TIMER_NUM_2:
			_timer = (rt_device_t)rt_device_find(TIMER_DEV_NAME_2);
			break;
		case TIMER_NUM_3:
			_timer = (rt_device_t)rt_device_find(TIMER_DEV_NAME_3);
			break;
		case TIMER_NUM_4:
			_timer = (rt_device_t)rt_device_find(TIMER_DEV_NAME_4);
			break;
		case TIMER_NUM_5:
			_timer = (rt_device_t)rt_device_find(TIMER_DEV_NAME_5);
			break;
		default: break;
	}
	
	if(_timer != RT_NULL)
	{
		stepper->hw_timer = _timer;
		if(config->timer_freq >= 50000)		/* hwtimer's frequency must higher enough, eg: 50kHz */
		{
			rt_device_control(_timer,HWTIMER_CTRL_FREQ_SET,&(config->timer_freq));
		}
	}
	/* default timer frequency: 1MHz */
	
}

rt_err_t _hw_timer_callback_trapzoid(rt_device_t dev,rt_size_t size)
{
	lt_stepper_t stepper = (lt_stepper_t)(dev->user_data);
	rt_pwm_disable(stepper->parent.pwm,stepper->parent.pwm_channel);	/* disable pwm output at first */
	stepper->index++;	
	if(stepper->index > stepper->max_index)		/* finish all work, stop timer! */
	{
		stepper->index = 0;
		stepper->max_index = 0;
		rt_device_close(stepper->hw_timer);		/* close hwtimer */
		stepper->hw_timer->user_data = stepper->parent.user_data;		/* change user_data */
		rt_free(stepper->accel_series);									/* release memory */
		stepper->parent.status = MOTOR_STATUS_STOP;
	}
	else
	{
		rt_uint16_t index = stepper->index;
		rt_uint32_t period = stepper->accel_series[index] * 1000;		/* unit : ns */
		rt_hwtimerval_t timeout_s;
		timeout_s.usec = stepper->accel_series[index];					/* unit: us */
		/* start timer at first */
		rt_device_write(stepper->hw_timer,0,&timeout_s,sizeof(timeout_s));
		rt_pwm_set(stepper->parent.pwm,stepper->parent.pwm_channel,period,0.5*period);
		rt_pwm_enable(stepper->parent.pwm,stepper->parent.pwm_channel);	 /* start pwm, ouptut a pulse! */
	}
	return RT_EOK;
}

static void _motor_stepper_trapzoid_accelerate(lt_stepper_t stepper,struct lt_stepper_config_accel* config)
{
	if(!stepper->config_flag) return;				/* stepper is not configured */
	RT_ASSERT(stepper->hw_timer != RT_NULL);		/* must config hardware timer */
	rt_pwm_disable(stepper->parent.pwm,stepper->parent.pwm_channel);			/* disable pwm output first */
	/* use hardware timer to trapzoid accelerate */
	rt_uint32_t acc_step, dec_step,dec_start;		/* acc_step: actual accelerate step, dec_step: decelerate step, dec_start: decelerate start step */
	rt_uint32_t T0,T_min;							/* T0: initial period, T_min: min pulse period */
	rt_uint32_t acc_max_step;						/* max accelerate step in theory */
	rt_uint16_t i,j;
	
	if(config->step == 0)
	{
		return;
	}
	else if(config->step > 0) 	/* forward rotate */
	{
		rt_pin_write(stepper->parent.forward_pin,PIN_HIGH);
	}
	else if(config->step < 0)
	{
		rt_pin_write(stepper->parent.forward_pin,PIN_LOW);
		config->step = -config->step;
	}
	/* get min period ang initial period */
	T_min =  1000000*(stepper->stepper_angle/180.0f*PI)/(config->speed)/(stepper->subdivide);	    		/* speed unit: rad/s, T_min unit: us */
	T0 = 0.69*sqrtf(1000000 * 2*stepper->stepper_angle/180.0f*PI/config->accel)*1000.0f/(stepper->subdivide);	/* initial period, t = sqrt(2*S/accel), unit: us, multiply error coefficiency */
	if(T0 < T_min) T0 = T_min;
	/* get accel step and max accel step */
	acc_step = (config->speed*config->decel)/(config->accel + config->decel);
	acc_max_step = (config->speed)*(config->speed)/(2.0f * config->accel);			/* S = v^2/(2*accel) */
	
	if(acc_step <= acc_max_step)	/* stepper can't reach desired speed, so accel --> decel */
	{ 
		dec_step = config->step - acc_step;
	}
	else							/* accel --> constant --> decel */
	{
		dec_step = 1.0f*(acc_max_step*config->accel)/config->decel;
	}

	if(dec_step == 0)				/* in this case, stepper must decelerate */
	{
		dec_step = 1;				
	}
	
	dec_start = config->step - dec_step;
	
	/* we create a series of pulse period */
	rt_uint32_t *T_nums = rt_malloc((config->step+1)* sizeof(rt_uint32_t));
	RT_ASSERT(T_nums != RT_NULL);					/* check res */
	T_nums[0] = T0;		/* change pulse period to compare value, ms --> compare value */
	for(i = 1; i <= acc_step; i++)					/* accel */
	{
		T_nums[i] = T_nums[i-1] - 2.0f*T_nums[i-1]/(4*i + 1);
	}
	for(i = i; i < dec_start; i++)					/* const */
	{
		T_nums[i] = T_nums[i-1];
	}
	for(i = i; i < config->step; i++)				/* decel */
	{
		j = config->step - i;
		T_nums[i] = T_nums[i-1] + 2.0f*T_nums[i-1]/(4*j - 1);
	}
	T_nums[config->step] = T_nums[config->step-1] * 1.5;
	
	if(stepper->accel_series != RT_NULL)
	{
		rt_free(stepper->accel_series);				/* free malloc memory */
	}
	stepper->accel_series = T_nums;					/* record period series */
	stepper->max_index = config->step;
	stepper->index = 0;								/* current index */
	/* open hwtimer,set timeout callback function */ 		
	rt_device_open(stepper->hw_timer,RT_DEVICE_OFLAG_RDWR);	/* open hwtimer */
	rt_device_set_rx_indicate(stepper->hw_timer,_hw_timer_callback_trapzoid);	/* set timeout callback function */
	/* here we use user_data variation to transport stepper object! */
	stepper->parent.user_data = stepper->hw_timer->user_data;		/* record timer's user_data */
	stepper->hw_timer->user_data = stepper;							
	/* we disable pwm at first, then start hwtimer */
	rt_hwtimerval_t timeouts_s;
	timeouts_s.usec = T0;
	rt_pin_write(stepper->enable_pin,PIN_LOW);			/* enable output */
	rt_pwm_disable(stepper->parent.pwm,stepper->parent.pwm_channel);
	rt_device_write(stepper->hw_timer,0,&timeouts_s,sizeof(timeouts_s));
	stepper->parent.status = MOTOR_STATUS_ACCELERATE;
}

rt_err_t _hw_timer_callback_s_curve(rt_device_t dev,rt_size_t size)
{
	lt_stepper_t stepper = (lt_stepper_t)(dev->user_data);
	rt_pwm_disable(stepper->parent.pwm,stepper->parent.pwm_channel);	/* disable pwm output at first */
	stepper->index++;	
	if(stepper->index >= stepper->max_index)		/* finish all work, keep constant pulse period */
	{
		stepper->index = 0;
		stepper->max_index = 0;
		stepper->hw_timer->user_data = stepper->parent.user_data;		/* change user_data */
		rt_pwm_enable(stepper->parent.pwm,stepper->parent.pwm_channel);	/*  pwm output at first */
		rt_free(stepper->accel_series);									/* release memory */
		stepper->parent.status = MOTOR_STATUS_RUN;
	}
	else
	{
		rt_uint16_t index = stepper->index;
		rt_uint32_t period = stepper->accel_series[index] * 1000;		/* unit : ns */
		rt_hwtimerval_t timeout_s;
		timeout_s.usec = stepper->accel_series[index];					/* unit: us */
		/* start timer at first */
		rt_device_write(stepper->hw_timer,0,&timeout_s,sizeof(timeout_s));
		rt_pwm_set(stepper->parent.pwm,stepper->parent.pwm_channel,period,0.5*period);
		rt_pwm_enable(stepper->parent.pwm,stepper->parent.pwm_channel);	 /* start pwm, ouptut a pulse! */
	}
	return RT_EOK;
}

static void _motor_stepper_s_curve_accelerate(lt_stepper_t stepper,struct lt_stepper_config_accel* config)
{
	rt_pwm_disable(stepper->parent.pwm,stepper->parent.pwm_channel);		/* disable pwm output at first */
	/* use cubic curves to calculate */
	float delt_f = config->freq_max - config->freq_min;
	float num,den,freq;
	rt_uint16_t i;
	if(config->step <= 0)	/* reversal rotation */
	{
		rt_pin_write(stepper->parent.forward_pin,PIN_LOW);
		config->step = -config->step;
	}
	else					/* forward rotation */
	{
		rt_pin_write(stepper->parent.forward_pin,PIN_HIGH);
	}
	
	rt_uint32_t *T_nums = rt_malloc((config->step)* sizeof(rt_uint32_t)); /* create a speed  period list, unit: us */
	RT_ASSERT(T_nums != RT_NULL);					/* check res */
	
	for(i = 0; i < config->step; i++)
	{
		num = config->flexible * (i - config->step/2)/(config->step/2);
		den = 1.0f + expf(-num);
		freq = config->freq_min + delt_f*den;
		T_nums[i] = 1000000.0f/freq;				/* get period, unit: us */
	}
	
	if(stepper->accel_series != RT_NULL)
	{
		rt_free(stepper->accel_series);				/* free malloc memory */
	}
	stepper->accel_series = T_nums;					/* record period series */
	stepper->max_index = config->step;
	stepper->index = 0;								/* current index */
	
	/* open hwtimer,set timeout callback function */ 		
	rt_device_open(stepper->hw_timer,RT_DEVICE_OFLAG_RDWR);	/* open hwtimer */
	rt_device_set_rx_indicate(stepper->hw_timer,_hw_timer_callback_s_curve);	/* set timeout callback function */
	/* here we use user_data variation to transport stepper object! */
	stepper->parent.user_data = stepper->hw_timer->user_data;		/* record timer's user_data */
	stepper->hw_timer->user_data = stepper;							
	/* we start hwtimer, then enable pwm output */
	rt_hwtimerval_t timeouts_s;
	timeouts_s.usec = T_nums[0];
	rt_pin_write(stepper->enable_pin,PIN_LOW);			/* enable output */
	rt_device_write(stepper->hw_timer,0,&timeouts_s,sizeof(timeouts_s));
	rt_pwm_enable(stepper->parent.pwm,stepper->parent.pwm_channel);			/* enable pwm output first */
	stepper->parent.status = MOTOR_STATUS_ACCELERATE;
}

static void _stepper_interp_config(lt_stepper_t stepper, struct lt_stepper_config_interp* config)
{
	lt_stepper_t x_stepper = (lt_stepper_t)config->x_stepper;
	lt_stepper_t y_stepper = (lt_stepper_t)config->y_stepper;
	rt_pwm_disable(x_stepper->parent.pwm,x_stepper->parent.pwm_channel);		/* disable pwm output at first */
	rt_pwm_disable(y_stepper->parent.pwm,y_stepper->parent.pwm_channel);		/* disable pwm output at first */
	rt_pin_write(x_stepper->enable_pin,PIN_LOW);		/* enable output */
	rt_pin_write(y_stepper->enable_pin,PIN_LOW);		/* enable output */
	config->deviation = 0;								/* clear deviation */
	
	/* configure hardware timer */
	rt_hwtimerval_t timeouts_s;
	rt_uint32_t period = stepper->period*1000000;					/* unit: ns */
	int mode = HWTIMER_MODE_PERIOD;								
	timeouts_s.usec = stepper->period*1000;							/* unit: us */
	rt_device_open(stepper->hw_timer,RT_DEVICE_OFLAG_RDWR);			/* open hwtimer */
	rt_device_control(stepper->hw_timer,HWTIMER_CTRL_MODE_SET,&mode);			/* set hwtimer mode period */
	rt_device_write(stepper->hw_timer,0,&timeouts_s,sizeof(timeouts_s));
	/* set pwm info */
	rt_pwm_set(x_stepper->parent.pwm,x_stepper->parent.pwm_channel,period,0.5*period);
	rt_pwm_set(y_stepper->parent.pwm,y_stepper->parent.pwm_channel,period,0.5*period);
	/* here we use user_data variation to transport stepper object! */
	stepper->parent.user_data = stepper->hw_timer->user_data;		/* record timer's user_data */
	stepper->hw_timer->user_data = config;
	stepper->parent.status = MOTOR_STATUS_INTERP;
}

rt_err_t _hw_timer_callback_line_interp(rt_device_t dev,rt_size_t size)
{
	struct lt_stepper_config_interp* config = (struct lt_stepper_config_interp*)(dev->user_data);
	rt_pwm_disable(config->x_stepper->pwm,config->x_stepper->pwm_channel);		/* disable pwm output at first */
	rt_pwm_disable(config->y_stepper->pwm,config->y_stepper->pwm_channel);		/* disable pwm output at first */
	
	if(config->num_pulse == 0)
	{									/* finish all work, close hwtimer */
		rt_device_close(dev);
		config->x_stepper->status = MOTOR_STATUS_STOP;
		config->y_stepper->status = MOTOR_STATUS_STOP;
		/* change user_data */
	}
	if(config->deviation >= 0)
	{
		rt_pwm_enable(config->x_stepper->pwm,config->x_stepper->pwm_channel);	/* X + 1 */
		config->deviation -= config->y_end;
	}
	else
	{
		rt_pwm_enable(config->y_stepper->pwm,config->y_stepper->pwm_channel);	/* Y + 1 */
		config->deviation += config->x_end;
	}
	config->num_pulse--;				/* reduce pulse num */
	return RT_EOK;
}

static void _motor_stepper_line_interp(lt_stepper_t stepper,struct lt_stepper_config_interp* config)
{
	if(!stepper->config_flag)	return;								/* stepper is not configured */
	lt_stepper_t x_stepper = (lt_stepper_t)config->x_stepper;
	lt_stepper_t y_stepper = (lt_stepper_t)config->y_stepper;
	if(config->x_end < 0)
	{
		rt_pin_write(x_stepper->parent.forward_pin,PIN_LOW);		/* X reversal rotate */
		config->x_end = - config->x_end;
	}
	else
	{
		rt_pin_write(x_stepper->parent.forward_pin,PIN_HIGH);		/* X forward rotate */
	}
	
	if(config->y_end < 0)
	{
		rt_pin_write(y_stepper->parent.forward_pin,PIN_LOW);		/* Y reversal rotate */
		config->y_end = - config->y_end;
	}
	else
	{
		rt_pin_write(y_stepper->parent.forward_pin,PIN_HIGH);		/* Y forward rotate */
	}
	config->num_pulse = config->x_end + config->y_end;				/* get pulse number */
	rt_device_set_rx_indicate(stepper->hw_timer,_hw_timer_callback_line_interp);	/* set timeout callback function */
	_stepper_interp_config(stepper,config);							/* config interpolation  */
}

static void _circuler_interp_map(struct lt_stepper_config_interp* config)
{
	lt_stepper_t x_stepper = (lt_stepper_t)config->x_stepper;
	lt_stepper_t y_stepper = (lt_stepper_t)config->y_stepper;
	if(config->dir > 0)			/* clockwise interpolation */
	{	
		if(config->x_start > 0)
		{
			if(config->y_start > 0)
			{
				rt_pin_write(x_stepper->parent.forward_pin,PIN_HIGH);	/* CW */
				rt_pin_write(y_stepper->parent.forward_pin,PIN_LOW);	/* CCW */
			}
			else
			{
				rt_pin_write(x_stepper->parent.forward_pin,PIN_LOW);	/* CCW */
				rt_pin_write(y_stepper->parent.forward_pin,PIN_LOW);	/* CCW */
			}
		}
		else
		{
			if(config->y_start > 0)
			{
				rt_pin_write(x_stepper->parent.forward_pin,PIN_HIGH);	/* CW */
				rt_pin_write(y_stepper->parent.forward_pin,PIN_HIGH);	/* CW */
			}
			else
			{
				rt_pin_write(x_stepper->parent.forward_pin,PIN_LOW);	/* CCW */
				rt_pin_write(y_stepper->parent.forward_pin,PIN_HIGH);	/* CW */
			}
		}
	}
	else
	{
		if(config->x_start > 0)
		{
			if(config->y_start > 0)
			{
				rt_pin_write(x_stepper->parent.forward_pin,PIN_LOW);	/* CCW */
				rt_pin_write(y_stepper->parent.forward_pin,PIN_HIGH);	/* CW */
			}
			else
			{
				rt_pin_write(x_stepper->parent.forward_pin,PIN_HIGH);	/* CW */
				rt_pin_write(y_stepper->parent.forward_pin,PIN_HIGH);	/* CW */
			}
		}
		else
		{
			if(config->y_start > 0)
			{
				rt_pin_write(x_stepper->parent.forward_pin,PIN_LOW);	/* CCW */
				rt_pin_write(y_stepper->parent.forward_pin,PIN_LOW);	/* CCW */
			}
			else
			{
				rt_pin_write(x_stepper->parent.forward_pin,PIN_HIGH);	/* CW */
				rt_pin_write(y_stepper->parent.forward_pin,PIN_LOW);	/* CCW */
			}
		}
	}
	
}
static void _circular_interp_cw(struct lt_stepper_config_interp* config)
{
	if(config->deviation >= 0 )
	{
		switch(config->quadrant)
		{
			case 1:
			{
				rt_pwm_enable(config->y_stepper->pwm,config->y_stepper->pwm_channel);	/* Y - 1 */
				config->deviation = config->deviation - 2*config->y_start + 1;								
				config->y_start -= 1;										      		/* refresh pos info */
				break;
			}
			case 2:
			{
				rt_pwm_enable(config->x_stepper->pwm,config->x_stepper->pwm_channel);	/* X + 1 */
				config->deviation = config->deviation + 2*config->x_start + 1;								
				config->x_start += 1;										      		/* refresh pos info */
				break;
			}
			case 3:																		/* Y + 1 */
			{
				rt_pwm_enable(config->y_stepper->pwm,config->y_stepper->pwm_channel);	
				config->deviation = config->deviation + 2*config->y_start + 1;								
				config->y_start += 1;										      		/* refresh pos info */
				break;
			}
			case 4:																		/* X - 1*/
			{
				rt_pwm_enable(config->x_stepper->pwm,config->x_stepper->pwm_channel);	
				config->deviation = config->deviation - 2*config->y_start + 1;								
				config->x_start -= 1;										      		/* refresh pos info */
				break;
			}
			default:break;
			
		}
	}
	else
	{
		switch(config->quadrant)
			{
				case 1:
				{
					rt_pwm_enable(config->x_stepper->pwm,config->x_stepper->pwm_channel);	/* X + 1 */
					config->deviation = config->deviation + 2*config->x_start + 1;								
					config->x_start += 1;										      		/* refresh pos info */
					break;
				}
				case 2:
				{
					rt_pwm_enable(config->y_stepper->pwm,config->y_stepper->pwm_channel);	/* Y + 1 */
					config->deviation = config->deviation + 2*config->y_start + 1;								
					config->y_start += 1;										      		/* refresh pos info */
					break;
				}
				case 3:																		/* X - 1 */
				{
					rt_pwm_enable(config->x_stepper->pwm,config->x_stepper->pwm_channel);	
					config->deviation = config->deviation - 2*config->x_start + 1;								
					config->x_start -= 1;										      		/* refresh pos info */
					break;
				}
				case 4:																		/* Y - 1 */
				{
					rt_pwm_enable(config->y_stepper->pwm,config->y_stepper->pwm_channel);	
					config->deviation = config->deviation - 2*config->y_start + 1;								
					config->y_start -= 1;										      		/* refresh pos info */
					break;
				}
				default:break;
				
			}
	}		
}

static void _circular_interp_ccw(struct lt_stepper_config_interp* config)
{
	if(config->deviation >= 0 )
	{
		switch(config->quadrant)
		{
			case 1:
			{
				rt_pwm_enable(config->x_stepper->pwm,config->x_stepper->pwm_channel);	/* X - 1 */
				config->deviation = config->deviation - 2*config->x_start + 1;								
				config->x_start -= 1;										      		/* refresh pos info */
				break;
			}
			case 2:
			{
				rt_pwm_enable(config->y_stepper->pwm,config->y_stepper->pwm_channel);	/* Y - 1 */
				config->deviation = config->deviation - 2*config->y_start + 1;								
				config->y_start -= 1;										      		/* refresh pos info */
				break;
			}
			case 3:																		/* X + 1 */
			{
				rt_pwm_enable(config->x_stepper->pwm,config->x_stepper->pwm_channel);	
				config->deviation = config->deviation + 2*config->x_start + 1;								
				config->x_start += 1;										      		/* refresh pos info */
				break;
			}
			case 4:																		/* Y + 1*/
			{
				rt_pwm_enable(config->y_stepper->pwm,config->y_stepper->pwm_channel);	
				config->deviation = config->deviation + 2*config->y_start + 1;								
				config->y_start += 1;										      		/* refresh pos info */
				break;
			}
			default:break;
			
		}
	}
	else
	{
		switch(config->quadrant)
			{
				case 1:
				{
					rt_pwm_enable(config->y_stepper->pwm,config->y_stepper->pwm_channel);	/* Y + 1 */
					config->deviation = config->deviation + 2*config->y_start + 1;								
					config->y_start += 1;										      		/* refresh pos info */
					break;
				}
				case 2:
				{
					rt_pwm_enable(config->x_stepper->pwm,config->x_stepper->pwm_channel);	/* X - 1 */
					config->deviation = config->deviation - 2*config->x_start + 1;								
					config->x_start -= 1;										      		/* refresh pos info */
					break;
				}
				case 3:																		/* Y - 1 */
				{
					rt_pwm_enable(config->y_stepper->pwm,config->y_stepper->pwm_channel);	
					config->deviation = config->deviation - 2*config->y_start + 1;								
					config->y_start -= 1;										      		/* refresh pos info */
					break;
				}
				case 4:																		/* X + 1*/
				{
					rt_pwm_enable(config->x_stepper->pwm,config->x_stepper->pwm_channel);	
					config->deviation = config->deviation + 2*config->x_start + 1;								
					config->x_start += 1;										      		/* refresh pos info */
					break;
				}
				default:break;
			}
	}		
}


rt_err_t _hw_timer_callback_circular_interp(rt_device_t dev,rt_size_t size)
{
	struct lt_stepper_config_interp* config = (struct lt_stepper_config_interp*)(dev->user_data);
	rt_pwm_disable(config->x_stepper->pwm,config->x_stepper->pwm_channel);		/* disable pwm output at first */
	rt_pwm_disable(config->y_stepper->pwm,config->y_stepper->pwm_channel);		/* disable pwm output at first */
	
	if(config->num_pulse == 0)
	{									/* finish all work, close hwtimer */
		rt_device_close(dev);
		config->x_stepper->status = MOTOR_STATUS_STOP;
		config->y_stepper->status = MOTOR_STATUS_STOP;
	}
	
	if(config->dir == STEPPER_INTERP_DIR_CW)
	{
		_circular_interp_cw(config);
	}
	else
	{
		_circular_interp_ccw(config);
	}
	
	config->num_pulse--;				/* reduce pulse num */
	return RT_EOK;
}

static void _motor_stepper_circular_interp(lt_stepper_t stepper, struct lt_stepper_config_interp* config)
{
	
	rt_int32_t tmp1 =  config->x_start - config->x_end, tmp2 = config->y_start - config->y_end;
	if(tmp1 < 0) tmp1 = -tmp1;
	if(tmp2 < 0) tmp2 = -tmp2;
	config->num_pulse = tmp1 + tmp2;
	/* map pos to first quadrent */
	_circuler_interp_map(config);
	rt_device_set_rx_indicate(stepper->hw_timer,_hw_timer_callback_circular_interp);	/* set timeout callback function */
	_stepper_interp_config(stepper,config);							/* config interpolation  */
	/* we should refresh start pos in each loop because we need pos info */
}



