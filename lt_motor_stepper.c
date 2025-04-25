#include "ltmotorlib.h"
#include "math.h"
/* use hardware timer to help output given number pulse! */
static rt_err_t _timer_output_angle(rt_device_t dev, rt_size_t size)
{
	lt_stepper_t stepper = (lt_stepper_t)(dev->user_data);
	lt_driver_disable(stepper->parent.driver);		/* disable output */
	rt_device_close(dev);							/* close hwtimer */
	dev->user_data = stepper->parent.user_data;		/* change user_data */
	stepper->parent.status = MOTOR_STATUS_STOP;
	if(!stepper->parent.callback)					/* call done callback function */
	{
		stepper->parent.callback(RT_NULL);
	}
}

static void _motor_stepper_output_angle(lt_stepper_t stepper,float angle);
static void _motor_stepper_output(lt_stepper_t stepper, float input);
static void _motor_stepper_config(lt_stepper_t,struct lt_stepper_config* config);
static void _motor_stepper_trapzoid_accelerate(lt_stepper_t stepper,struct lt_stepper_config_accel* config);
static void _motor_stepper_s_curve_accelerate(lt_stepper_t stepper,struct lt_stepper_config_accel* config);
static void _motor_stepper_line_interp(lt_stepper_t stepper,struct lt_stepper_config_interp* config);
static void _motor_stepper_circular_interp(lt_stepper_t stepper,struct lt_stepper_config_interp* config);

static lt_motor_t _motor_stepper_create(char* name,rt_uint8_t reduction_ration,rt_uint8_t type)
{
	lt_stepper_t _motor;
	_motor = rt_malloc(sizeof(struct lt_motor_stepper_object));
	if(_motor == RT_NULL) return RT_NULL;
	
	rt_memset(_motor,0,sizeof(struct lt_motor_stepper_object));
	rt_strcpy(_motor->parent.name,name);
	
	_motor->parent.reduction_ratio = reduction_ration;
	_motor->parent.type = type;
	_motor->parent.ops = &_motor_stepper_ops;/* set operators */
	/* create stepper config struct */
	_motor->config = rt_malloc(sizeof(struct lt_stepper_config));
	rt_memset(_motor->config,0,sizeof(struct lt_stepper_config));
	
	return (lt_motor_t)_motor;				 /* type transform */
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
		case STEPPER_CTRL_CONFIG:		/* config stepper parameters */
		{
			struct lt_stepper_config*  config = (struct lt_stepper_config*)arg;
			_motor_stepper_config(stepper,config);
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

static rt_err_t _motor_stepper_delete(lt_motor_t motor)
{
	lt_stepper_t stepper = (lt_stepper_t)motor;
	if(stepper->config != RT_NULL)
	{
		rt_free(stepper->config);
		stepper->config = RT_NULL;
	}
	rt_free(stepper);
	return RT_EOK;
}

struct lt_motor_ops _motor_stepper_ops = {
										_motor_stepper_create,
										_motor_stepper_control,
										_motor_stepper_delete,
									 };

static void _motor_stepper_output(lt_stepper_t stepper, float input)
{
	/* rotate at a constant speed */
	/* speed is proportional to frequency */
	if(!stepper->config_flag)	return;											/* not configured */
	rt_uint8_t dir = _get_rotation_dir(&input);	/* get rotation direction and change sign of input if input < 0 */
	lt_driver_t driver = stepper->parent.driver;
	/* check input */
	input = _constrains_dead_region(input,STEPPER_MAX_SPEED,STEPPER_MIN_SPEED);
	
	if(input == 0)
	{
		lt_driver_disable(driver);												/* disable output */
		stepper->parent.status = MOTOR_STATUS_STOP;
	}
	else
	{
		input = STEPPER_MAX_PERIOD/input;										/* transform */
		if(input > STEPPER_MAX_PERIOD) input = STEPPER_MAX_PERIOD;			    /* slowest rotation */
		lt_driver_set_output(driver,input,STEPPER_DUTY_CYCLE,0);				/* default phase */
		stepper->parent.status = MOTOR_STATUS_RUN;
	}
}

static void _motor_stepper_output_angle(lt_stepper_t stepper, float angle)
{
	if(!stepper->config_flag) return;					/* stepper is not configured */
	rt_uint8_t dir = _get_rotation_dir(&angle);			/* get rotation direction and change sign of input if input < 0 */
	lt_driver_t driver = stepper->parent.driver;
	struct lt_stepper_config* config = stepper->config;
	rt_device_t hw_timer = config->hw_timer;
	lt_driver_disable(driver);							/* disable driver at first */
	
	if(angle == 0)
	{
		stepper->parent.status = MOTOR_STATUS_STOP;
		return;
	}
	/* get number of square pulse */
	rt_uint32_t n = angle/(config->stepper_angle/config->subdivide);
	/* create a software timer to help process angle output */			
	rt_uint32_t period = config->period*1000;									/* unit: us */	
	rt_tick_t time = config->period*n;											/* unit: tick or ms */	
	
	rt_hwtimerval_t timeout_s;
	timeout_s.usec = time*1000;													/* uint: us */
	/* open hwtimer,set timeout callback function */ 		
	rt_device_open(hw_timer,RT_DEVICE_OFLAG_RDWR);								/* open hwtimer */
	rt_device_set_rx_indicate(hw_timer,_timer_output_angle);					/* set timeout callback function */
	/* here we use user_data variation to transport stepper object! */
	stepper->parent.user_data = hw_timer->user_data;							/* record timer's user_data */
	hw_timer->user_data = stepper;							
	rt_device_write(hw_timer,0,&timeout_s,sizeof(timeout_s));					/* start hw_timer */
	
	lt_driver_set_output(driver,period,STEPPER_DUTY_CYCLE,0);
	lt_driver_enable(driver,dir);												/* output */
}

static void _motor_stepper_config(lt_stepper_t stepper,struct lt_stepper_config* config)
{
	struct lt_stepper_config* _config = stepper->config;
	
	_config->stepper_angle = config->stepper_angle;
	_config->period = config->period;
	_config->subdivide = config->subdivide;
	stepper->config_flag = 1;								/* finish config */
	
	if(_config->subdivide == 0)
		_config->subdivide = 1;
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
		_config->hw_timer = _timer;
		if(config->timer_freq >= 50000)		/* hwtimer's frequency must higher enough, eg: 50kHz */
		{
			rt_device_control(_timer,HWTIMER_CTRL_FREQ_SET,&(config->timer_freq));
		}	/* default timer frequency: 1MHz */
	}
}

rt_err_t _hw_timer_callback_accel(rt_device_t dev,rt_size_t size)
{
	lt_stepper_t stepper = (lt_stepper_t)(dev->user_data);
	lt_driver_t  driver = stepper->parent.driver;
	struct lt_stepper_config_accel * config = stepper->config_acc;
	rt_device_t hw_timer = stepper->config->hw_timer;
	/* disable output at first */
	lt_driver_disable(driver);
	++config->index;	
	if(config->index >= config->max_index)						/* finish all work, stop timer! */
	{
		config->index = 0;
		config->max_index = 0;
		rt_device_close(hw_timer);								/* close hwtimer */
		hw_timer->user_data = stepper->parent.user_data;		/* change user_data */
		rt_free(config->acc_series);							/* release memory */
		config->acc_series = RT_NULL;							/* pointer points NULL */
		stepper->parent.status = MOTOR_STATUS_STOP;
		if(!stepper->parent.callback)							/* call done callback function */
		{
			stepper->parent.callback(RT_NULL);
		}
	}
	else
	{
		rt_uint16_t index = config->index;
		rt_uint32_t period = config->acc_series[index] ;		/* unit : us */
		rt_hwtimerval_t timeout_s;
		timeout_s.usec = config->acc_series[index];				/* unit: us */
		/* start timer at first, then output */
		rt_device_write(hw_timer,0,&timeout_s,sizeof(timeout_s));
		lt_driver_set_output(driver,period,STEPPER_DUTY_CYCLE,0);
		lt_driver_enable(driver,0);									
	}
	return RT_EOK;
}

static void _stepper_accel_config(lt_stepper_t stepper,struct lt_stepper_config_accel* config,rt_uint32_t* T_nums,rt_uint32_t T0,rt_uint8_t dir)
{
	RT_ASSERT(stepper->config->hw_timer != RT_NULL);				/* must config hardware timer */
	lt_driver_t  driver = stepper->parent.driver;
	rt_device_t hw_timer = stepper->config->hw_timer;
	stepper->config_acc = config;
	
	if(config->acc_series != RT_NULL)
	{
		rt_free(config->acc_series);				/* free malloc memory */
	}
	config->acc_series = T_nums;					/* record period series */
	config->max_index = config->step;
	config->index = 0;												/* current index */
	/* open hwtimer,set timeout callback function */ 		
	rt_device_open(hw_timer,RT_DEVICE_OFLAG_RDWR);					/* open hwtimer */
	rt_device_set_rx_indicate(hw_timer,_hw_timer_callback_accel);	/* set timeout callback function */
	/* here we use user_data variation to transport stepper object! */
	stepper->parent.user_data = hw_timer->user_data;				/* record timer's user_data */
	hw_timer->user_data = stepper;							
	/* start timer and output !!! */
	rt_hwtimerval_t timeouts_s;
	timeouts_s.usec = T0;
	rt_device_write(hw_timer,0,&timeouts_s,sizeof(timeouts_s));		/* start timer */
	lt_driver_set_output(driver,T0,STEPPER_DUTY_CYCLE,0);
	lt_driver_enable(driver,dir);
	stepper->parent.status = MOTOR_STATUS_ACCELERATE;
}

static void _motor_stepper_trapzoid_accelerate(lt_stepper_t stepper,struct lt_stepper_config_accel* config)
{
	if(!stepper->config_flag) return;						/* stepper is not configured */
	lt_driver_disable(stepper->parent.driver);			    /* disable pwm output at first */
	
	rt_uint32_t acc_step, dec_step,dec_start;				/* acc_step: actual accelerate step, dec_step: decelerate step, dec_start: decelerate start step */
	rt_uint32_t T0,T_min;									/* T0: initial period, T_min: min pulse period */
	rt_uint32_t acc_max_step;								/* max accelerate step in theory */
	rt_uint16_t i,j;
	float angle = stepper->config->stepper_angle;
	float input = config->step;
	rt_uint8_t dir = _get_rotation_dir(&input);				/* get rotation direction */
	rt_uint16_t subdivide = stepper->config->subdivide;
	config->step = (int)(input);							

	if(input == 0) return;									/* no need to output */
	
	/* get min period ang initial period */
	T_min =  1000000.0f*(angle/180.0f*PI)/(config->speed)/subdivide;	    		  	/* speed unit: rad/s, T_min unit: us */
	T0 = 0.69*sqrtf(1000000 * 2*angle/180.0f*PI/config->accel)*1000.0f/subdivide;	/* initial period, t = sqrt(2*S/accel), unit: us, multiply error coefficiency */
	if(T0 < T_min) T0 = T_min;
	/* get accel step and max accel step */
	acc_step = (config->speed*config->decel)/(config->accel + config->decel);
	acc_max_step = (config->speed)*(config->speed)/(2.0f * config->accel);			/* S = v^2/(2*accel) */
	
	if(acc_step <= acc_max_step)					/* stepper can't reach desired speed, so accel --> decel */
	{ 
		dec_step = config->step - acc_step;
	}
	else											/* accel --> constant --> decel */
	{
		dec_step = 1.0f*(acc_max_step*config->accel)/config->decel;
	}

	if(dec_step == 0)								/* in this case, stepper must decelerate */
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
	/* common accel config part */
	_stepper_accel_config(stepper,config,T_nums,T0,dir);	
}

static void _motor_stepper_s_curve_accelerate(lt_stepper_t stepper,struct lt_stepper_config_accel* config)
{
	if(!stepper->config_flag) return;									/* stepper is not configured */
	lt_driver_disable(stepper->parent.driver);			    			/* disable driver at first */
	/* use cubic curves to calculate */
	float delt_f = config->freq_max - config->freq_min;
	float num,den,freq;
	rt_uint16_t i;
	float input = config->step;
	rt_uint8_t dir = _get_rotation_dir(&input);							/* get rotation direction and change sign of input if input < 0 */
	if(input == 0) return;												/* no need to output */
	config->step = (int)input;
	
	rt_uint32_t *T_nums = rt_malloc((config->step)* sizeof(rt_uint32_t)); /* create a speed  period list, unit: us */
	RT_ASSERT(T_nums != RT_NULL);										/* check res */
	
	for(i = 0; i < config->step; i++)
	{
		num = config->flexible * (i - config->step/2)/(config->step/2);
		den = 1.0f + expf(-num);
		freq = config->freq_min + delt_f*den;
		T_nums[i] = 1000000.0f/freq;									/* get period, unit: us */
	}
	/* commom accel config part */
	_stepper_accel_config(stepper,config,T_nums,T_nums[0],dir);	
}

static void _stepper_interp_config(lt_stepper_t stepper, struct lt_stepper_config_interp* config,rt_uint8_t x_dir,rt_uint8_t y_dir)
{
	lt_stepper_t x_stepper = (lt_stepper_t)config->x_stepper;
	lt_stepper_t y_stepper = (lt_stepper_t)config->y_stepper;
	lt_driver_t x_driver = x_stepper->parent.driver;
	lt_driver_t y_driver = y_stepper->parent.driver;
	rt_device_t hw_timer = stepper->config->hw_timer;
	stepper->config_interp = config;									/* record interp config */
	/* we first set default rotation direction, in interp case, we don't need to change direction
	*  and care rotation direction in callback function, we just need to call like this:
	* lt_driver_enable(driver,0);	//	param "0" means we don't change rotation direction in this call
	* this is so convenient !!!
	*/
	lt_driver_set_output(x_driver,0,0,0);								/* zero output */
	lt_driver_set_output(y_driver,0,0,0);								/* zero output */
	lt_driver_enable(x_driver,x_dir);									/* save direction */
	lt_driver_enable(y_driver,y_dir);									/* save direction */
	lt_driver_disable(x_driver);										/* disable output */
	lt_driver_disable(y_driver);										/* disable output */
	
	config->deviation = 0;												/* clear deviation */	
	/* configure hardware timer */
	rt_hwtimerval_t timeouts_s;
	rt_uint32_t period = stepper->config->period*1000;					/* unit: us */
	int mode = HWTIMER_MODE_PERIOD;								
	timeouts_s.usec = period;											/* unit: us */
	rt_device_open(hw_timer,RT_DEVICE_OFLAG_RDWR);						/* open hwtimer */
	rt_device_control(hw_timer,HWTIMER_CTRL_MODE_SET,&mode);			/* set hwtimer mode period */
	rt_device_write(hw_timer,0,&timeouts_s,sizeof(timeouts_s));
	/* set driver output */		
	lt_driver_set_output(x_driver,period,STEPPER_DUTY_CYCLE,0);		
	lt_driver_set_output(y_driver,period,STEPPER_DUTY_CYCLE,0);

	/* here we use user_data variation to transport stepper object! */
	stepper->parent.user_data = hw_timer->user_data;					/* record timer's user_data */
	hw_timer->user_data = config;
	stepper->parent.status = MOTOR_STATUS_INTERP;
}

static rt_uint8_t _callback_interp_commom(rt_device_t dev, struct lt_stepper_config_interp* config)
{
	/* disable output at first */
	lt_driver_disable(config->x_stepper->driver);
	lt_driver_disable(config->y_stepper->driver);
	
	if(config->num_pulse == 0)
	{									/* finish all work, close hwtimer */
		rt_device_close(dev);
		config->x_stepper->status = MOTOR_STATUS_STOP;
		config->y_stepper->status = MOTOR_STATUS_STOP;
		if(!config->x_stepper->callback)	/* call done callback function */
		{
			config->x_stepper->callback(RT_NULL);
		}
		return 1;						
	}
	config->num_pulse--;				/* reduce pulse num */
	return 0;
}

static rt_err_t _hw_timer_callback_line_interp(rt_device_t dev,rt_size_t size)
{
	struct lt_stepper_config_interp* config = (struct lt_stepper_config_interp*)(dev->user_data);
	rt_uint8_t res = _callback_interp_commom(dev,config);		/* interp callback common part */
	if(res) return RT_EOK;										/* finish interp, return */
	
	if(config->deviation >= 0)
	{
		lt_driver_enable(config->x_stepper->driver,0);			/* X + 1 */
		config->deviation -= config->y_end;
	}
	else
	{
		lt_driver_enable(config->y_stepper->driver,0);			/* Y + 1 */
		config->deviation += config->x_end;
	}
	return RT_EOK;
}

static void _motor_stepper_line_interp(lt_stepper_t stepper,struct lt_stepper_config_interp* config)
{
	if(!stepper->config_flag)	return;								/* stepper is not configured */
	float input1 = config->x_end;
	float input2 = config->y_end;
	
	rt_uint8_t x_dir = _get_rotation_dir(&input1);					/* get rotation direction and change sign of input if input < 0 */	
	rt_uint8_t y_dir = _get_rotation_dir(&input2);					/* get rotation direction and change sign of input if input < 0 */
	config->x_end = (int)input1;									
	config->y_end = (int)input2;
	
	config->num_pulse = config->x_end + config->y_end;					/* get pulse number */
	rt_device_set_rx_indicate(stepper->config->hw_timer,_hw_timer_callback_line_interp);	/* set timeout callback function */
	_stepper_interp_config(stepper,config,x_dir,y_dir);					/* config interpolation  */
}

rt_uint8_t _map_table[4][4] = {	{ 0x6,	 0x5,	0xB,	0x8},
								{ 0xF,	 0xE,	0x0,	0x1 },
								{ 0x8,	 0xB,	0x5,	0x6},
								{ 0x1,	 0x0,	0xE,	0xF}
};

/* each value has 4 bits, eg: 0x6 = 2#0110, bit3~bit0					
*  	bit0: current axis, 			0: X, 		1: Y
*	bit1: expression sign			0: +, 		1: -
*	bit2: x rotation direction, 	0: forward, 1:reversal
*	bit3: y rotation direction, 	0: forward, 1:reversal
*	
*	the x axis of this _map_table corresponds to four quadrant: 0~3 => 1th ~ 4th
* 	the y axis of this _map_table corresponds to interp direction plus deviation 
*	0: CCW + devia >= 0;	1: CCW + devia < 0;		2: CW + devia >= 0;		3: CW + devia < 0
*/

rt_err_t _hw_timer_callback_circular_interp(rt_device_t dev,rt_size_t size)
{
	struct lt_stepper_config_interp* config = (struct lt_stepper_config_interp*)(dev->user_data);
	rt_uint8_t res = _callback_interp_commom(dev,config);		/* interp callback common part */
	if(res) return RT_EOK;										/* finish interp, return */
	
	rt_uint8_t _x = config->quadrant, _y;
	int devia = config->deviation;
	rt_uint8_t sign,curr_axis,val;	
	
	if(config->dir == DIR_CCW)
	{
		if(devia >= 0) _y = 0;
		else 		   _y = 1;
	}
	else
	{
		if(devia >= 0) _y = 0;
		else 		   _y = 1;
	}
	
	val = _map_table[_x][_y];
	sign = GET_BIT(val,1);
	curr_axis = GET_BIT(val,0);
	
	if(curr_axis == 0)		/* X axis move */
	{
		if(sign == 0)		/* + */
		{
			config->deviation = config->deviation + 2*config->x_start + 1;
			config->x_start += 1;
		}
		else				/* - */
		{
			config->deviation = config->deviation - 2*config->x_start + 1;
			config->x_start -= 1;
		}
		lt_driver_enable(config->x_stepper->driver,0);
	}
	else					/* Y axis move */
	{
		if(sign == 0)		/* + */
		{
			config->deviation = config->deviation + 2*config->y_start + 1;
			config->y_start += 1;
		}
		else				/* - */
		{
			config->deviation = config->deviation - 2*config->y_start + 1;
			config->y_start -= 1;
		}
		lt_driver_enable(config->y_stepper->driver,0);
	}
	
	return RT_EOK;
}

static void _motor_stepper_circular_interp(lt_stepper_t stepper, struct lt_stepper_config_interp* config)
{
	/* check whether start pos and end pos in the same quarent and circle */
	rt_uint8_t res = _check_pos(config->x_start,config->y_start,config->x_end,config->y_end);
	if(!res) return;														/* the two positions are unvalid!!! */
	rt_uint8_t x_dir, y_dir;
	rt_uint8_t _x, _y, val;
	rt_int32_t tmp1 =  config->x_start - config->x_end, tmp2 = config->y_start - config->y_end;
	
	if(tmp1 < 0) tmp1 = -tmp1;
	if(tmp2 < 0) tmp2 = -tmp2;
	config->num_pulse = tmp1 + tmp2;										/* get pulse number */
	/* map pos to first quadrent */
	_x = _get_quard(config->x_start,config->y_start);		/* get interpolation quadrant, x axis of _map_table */
	if(config->dir == DIR_CCW)								/* counter-clockwise interpolation */
	{	
		_y = 0;												/* get correspond y axis of _map_table */
	}
	else if(config->dir == DIR_CW)							/* clockwise interpolation */
	{
		_y = 2;
	}
	else	return;											/* unvalid direction */
			
	val = _map_table[_x][_y];
	/* save rotation directions */
	x_dir = GET_BIT(val,2) + 1;			/* get bit2 */
	y_dir = GET_BIT(val,3) + 1;			/* get bit3 */
	
	rt_device_set_rx_indicate(stepper->config->hw_timer,_hw_timer_callback_circular_interp);	/* set timeout callback function */
	_stepper_interp_config(stepper,config,x_dir,y_dir);						/* config interpolation  */
	/* we should refresh start pos in each loop because we need pos info */
}
