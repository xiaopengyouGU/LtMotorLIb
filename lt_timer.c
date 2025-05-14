#include "ltmotorlib.h"

static void _timer_soft_timeout(void*parameter)
{
	lt_timer_t timer = (lt_timer_t)parameter;
	if(timer->soft_timeout != RT_NULL)
	{
		timer->soft_timeout(timer);
	}
}

static rt_err_t _timer_hw_timeout(rt_device_t dev, rt_size_t size)
{
	lt_timer_t timer = (lt_timer_t)dev->user_data;	/* use user_data to transport data */
	if(timer->hw_timeout != RT_NULL)
	{
		timer->hw_timeout(timer);
	}
	return RT_EOK;
}

static rt_err_t _enable_timer(lt_timer_t timer, rt_uint8_t type);
static rt_err_t _disable_timer(lt_timer_t timer,rt_uint8_t type);
static rt_err_t _set_timer(lt_timer_t timer, rt_uint32_t period, rt_uint8_t mode,rt_uint8_t type);
static rt_err_t _set_timeout(lt_timer_t timer,void(*timeout)(lt_timer_t),rt_uint8_t type);

lt_timer_t lt_timer_create(char*soft_name,char* hw_name,rt_uint32_t freq)
{
	lt_timer_t _timer;
	rt_device_t hw_timer;
	rt_timer_t soft_timer;
	
	_timer = rt_malloc(sizeof(struct lt_timer_object));
	if(_timer == RT_NULL) return RT_NULL;
	rt_memset(_timer,0,sizeof(struct lt_timer_object));
	/* find hardware timer device */
	hw_timer = (rt_device_t)rt_device_find(hw_name);
	
	if(hw_timer != RT_NULL)
	{	
		if(freq >= 30000)		/* hwtimer's frequency must higher enough, eg: 30kHz */
		{
			rt_device_control(hw_timer,HWTIMER_CTRL_FREQ_SET,&freq);
		}	/* default timer frequency: 1MHz */
		_timer->hw_timer = hw_timer;
	}
	/* create software timer */
	soft_timer = rt_timer_create(soft_name,_timer_soft_timeout,_timer,0,RT_TIMER_FLAG_ONE_SHOT);
	if(soft_timer != RT_NULL)
	{
		_timer->soft_timer = soft_timer;
	}
	
	return _timer;
}

rt_err_t lt_timer_set(lt_timer_t timer,rt_uint32_t period, rt_uint8_t mode,rt_uint8_t type)
{
	RT_ASSERT(timer != RT_NULL);
	return _set_timer(timer,period,mode,type);
}

rt_err_t lt_timer_set_timeout(lt_timer_t timer,void(*timeout)(lt_timer_t),rt_uint8_t type)
{
	RT_ASSERT(timer != RT_NULL);
	return _set_timeout(timer,timeout,type);
}

rt_err_t lt_timer_enable(lt_timer_t timer,rt_uint8_t type)
{
	RT_ASSERT(timer != RT_NULL);
	return _enable_timer(timer,type);
}

rt_err_t lt_timer_disable(lt_timer_t timer,rt_uint8_t type)
{
	RT_ASSERT(timer != RT_NULL);
	return _disable_timer(timer,type);
}

rt_err_t lt_timer_delete(lt_timer_t timer)
{
	RT_ASSERT(timer != RT_NULL);
	if(timer->soft_timer != RT_NULL)
	{
		rt_timer_delete(timer->soft_timer);
		timer->soft_timer = RT_NULL;
	}
	rt_free(timer);
	return RT_EOK;
}

rt_err_t lt_timer_period_call(lt_timer_t timer,rt_uint32_t period, void(*timeout)(lt_timer_t),void* user_data,rt_uint8_t type)
{
	RT_ASSERT(timer != RT_NULL);
	rt_err_t res;
	timer->user_data = user_data;
	res |= _disable_timer(timer,type);			/* disable timer at first */
	res |= _set_timer(timer,period,TIMER_MODE_PERIODIC,type);
	res |=_set_timeout(timer,timeout,type);
	res |= _enable_timer(timer,type);			/* eable timer finally */
	
	return res;
}


static rt_err_t _enable_timer(lt_timer_t timer, rt_uint8_t type)
{
	if(type == TIMER_TYPE_HW)
	{
		if(timer->hw_timer == RT_NULL) return RT_ERROR;
		static rt_hwtimerval_t timeout_s;
		timeout_s.usec = timer->hw_period;			/* unit: us */
		/* start timer at first, then output */
		rt_device_open(timer->hw_timer,RT_DEVICE_OFLAG_RDWR);					/* open hwtimer */
		rt_device_write(timer->hw_timer,0,&timeout_s,sizeof(timeout_s));
	}
	else
	{
		if(timer->soft_timer == RT_NULL) return RT_ERROR;
		rt_timer_start(timer->soft_timer);
	}
	return RT_EOK;
}

static rt_err_t _disable_timer(lt_timer_t timer, rt_uint8_t type)
{
	if(type == TIMER_TYPE_HW)
	{
		if(timer->hw_timer == RT_NULL) return RT_ERROR;
		rt_device_close(timer->hw_timer);
	}
	else
	{
		if(timer->soft_timer == RT_NULL) return RT_ERROR;
		rt_timer_stop(timer->soft_timer);
	}
	return RT_EOK;
}

static rt_err_t _set_timer(lt_timer_t timer, rt_uint32_t period, rt_uint8_t mode,rt_uint8_t type)
{
	if(type == TIMER_TYPE_HW)		/* set hardware timer */
	{
		if(timer->hw_timer == RT_NULL) return RT_ERROR;
		timer->hw_period = period;
		rt_hwtimer_mode_t  hw_mode;		/* hardward timer mode, period/single */
		if(mode == TIMER_MODE_SINGLE_SHOT)
		{
			hw_mode = HWTIMER_MODE_ONESHOT;
		}
		else if(mode == TIMER_MODE_PERIODIC)
		{
			hw_mode = HWTIMER_MODE_PERIOD;
		}
		else
		{
			return RT_EOK;
		}
		rt_device_control(timer->hw_timer,HWTIMER_CTRL_MODE_SET,&hw_mode);
	}
	else
	{
		if(timer->soft_timer == RT_NULL) return RT_ERROR;
		if(period < 1000)
		{
			period = 0;
		}
		else
		{
			period /= 1000;		/* unit trans: us --> ms */
		}
		rt_timer_control(timer->soft_timer,RT_TIMER_CTRL_SET_TIME,&period);
		if(mode == TIMER_MODE_SINGLE_SHOT)
		{
			rt_timer_control(timer->soft_timer,RT_TIMER_CTRL_SET_ONESHOT,RT_NULL);
		}
		else if(mode == TIMER_MODE_PERIODIC)
		{
			rt_timer_control(timer->soft_timer,RT_TIMER_CTRL_SET_PERIODIC,RT_NULL);
		}
	}
	return RT_EOK;
}

static rt_err_t _set_timeout(lt_timer_t timer,void(*timeout)(lt_timer_t),rt_uint8_t type)
{
	if(timeout == RT_NULL) return RT_ERROR;
	
	if(type == TIMER_TYPE_HW)
	{
		rt_device_t hw_timer = timer->hw_timer;
		if(hw_timer == RT_NULL) return RT_ERROR;
		timer->hw_timeout = timeout;		
		hw_timer->user_data = timer;		/* use user_data to transport data */
		rt_device_set_rx_indicate(timer->hw_timer,_timer_hw_timeout);
	}
	else
	{
		if(timer->soft_timer == RT_NULL) return RT_ERROR;
		timer->soft_timeout = timeout;
	}
	return RT_EOK;
}
