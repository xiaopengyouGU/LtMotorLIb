#include "ltmotorlib.h"


extern struct lt_driver_ops _driver_dc_ops;
//extern struct lt_motor_ops _motor_bldc_ops;
extern struct lt_driver_ops _driver_stepper_ops;

lt_driver_t lt_driver_create(rt_uint8_t type)
{
	lt_driver_t _driver;
	_driver = rt_malloc(sizeof(struct lt_driver_object));
	rt_memset(_driver,0,sizeof(struct lt_driver_object));
	_driver->type = type;
	
	switch(type)
	{
		case DRIVER_TYPE_DC:
			_driver->ops = &_driver_dc_ops;
			break;
		case DRIVER_TYPE_STEPPER:
			_driver->ops = &_driver_stepper_ops;
			break;
		//case DRIVER_TYPE_STEPPER:
		//	_driver->ops = &_driver_bldc_ops;
		//	break;
		default: break;
	}
	return _driver;
}

rt_err_t lt_driver_set_pins(lt_driver_t driver,rt_base_t forward_pin,rt_base_t reversal_pin,rt_base_t enable_pin)
{
	RT_ASSERT(driver != RT_NULL);
	RT_ASSERT(driver->ops!= RT_NULL);
	RT_ASSERT(driver->ops->set_pins != RT_NULL);
	
	driver->forward_pin = forward_pin;
	driver->reversal_pin = reversal_pin;
	driver->enable_pin = enable_pin;
	
	return driver->ops->set_pins(driver);
}

rt_err_t lt_driver_set_pwm(lt_driver_t driver,rt_uint8_t pwm_num,rt_uint8_t pwm_channel,rt_uint8_t phase)
{
	RT_ASSERT(driver != RT_NULL);
	struct rt_device_pwm* pwm;
	switch(pwm_num)
	{
		case PWM_NUM_1:
			pwm = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME_1);
			break;
		case PWM_NUM_2:
			pwm = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME_2);
			break;
		case PWM_NUM_3:
			pwm = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME_3);
			break;
		case PWM_NUM_4:
			pwm = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME_4);
			break;
		case PWM_NUM_5:
			pwm = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME_5);
			break;
		default: break;
	}

	if(pwm != RT_NULL)
	{
		rt_pwm_disable(pwm,pwm_channel);
	}
	else return RT_ERROR;
	
	switch(phase)
	{
		case PWM_PHASE_DEFAULT:
			driver->pwm_A = pwm;
			driver->pwm_channel_A = pwm_channel;
			break;
		case PWM_PHASE_A:
			driver->pwm_A = pwm;
			driver->pwm_channel_A = pwm_channel;
			break;
		case PWM_PHASE_B:
			driver->pwm_B = pwm;
			driver->pwm_channel_B = pwm_channel;
			break;
		case PWM_PHASE_C:
			driver->pwm_C = pwm;
			driver->pwm_channel_C = pwm_channel;
			break;
		default : return RT_ERROR;
	}
	return RT_EOK;
}

rt_err_t lt_driver_set_output(lt_driver_t driver,rt_uint32_t period,float duty_cycle,rt_uint8_t phase)
{
	RT_ASSERT(driver != RT_NULL);
	struct rt_device_pwm *pwm;
	rt_uint8_t channel;			
	if(duty_cycle > 1) duty_cycle = 1;
	else if(duty_cycle < 0) duty_cycle = 0;
	
	period = period*1000;		/* unit: us-> ns */
	rt_uint32_t pulse = (rt_uint32_t)(period*duty_cycle);
	
	switch(phase)
	{
		case PWM_PHASE_DEFAULT:
		{
			if(driver->pwm_A != RT_NULL && driver->pwm_channel_A != RT_NULL)
			{
				pwm = driver->pwm_A;
				channel = driver->pwm_channel_A;
			}
			break;
		}
		case PWM_PHASE_A:
		{
			if(driver->pwm_A != RT_NULL && driver->pwm_channel_A != RT_NULL)
			{
				pwm = driver->pwm_A;
				channel = driver->pwm_channel_A;
			}
			break;
		}
		case PWM_PHASE_B:
		{
			if(driver->pwm_B != RT_NULL && driver->pwm_channel_B != RT_NULL)
			{
				pwm = driver->pwm_B;
				channel = driver->pwm_channel_B;
			}
			break;
		}
		case PWM_PHASE_C:
		{
			if(driver->pwm_C != RT_NULL && driver->pwm_channel_C != RT_NULL)
			{
				pwm = driver->pwm_C;
				channel = driver->pwm_channel_C;
			}
			break;
		}
		default: break;
	}
	rt_pwm_disable(pwm,channel);							/* disable pwm first */
	rt_pwm_set(pwm,channel,period,pulse);					/* set pwm output */
	
	return RT_EOK;
}

rt_err_t lt_driver_enable(lt_driver_t driver,rt_uint8_t dir)
{
	RT_ASSERT(driver != RT_NULL);
	RT_ASSERT(driver->ops!= RT_NULL);
	RT_ASSERT(driver->ops->enable != RT_NULL);
	return driver->ops->enable(driver,dir);
}

rt_err_t lt_driver_disable(lt_driver_t driver)
{
	RT_ASSERT(driver != RT_NULL);
	RT_ASSERT(driver->ops!= RT_NULL);
	RT_ASSERT(driver->ops->disable != RT_NULL);
	return driver->ops->disable(driver);
}

rt_err_t lt_driver_delete(lt_driver_t driver)
{
	RT_ASSERT(driver != RT_NULL);
	rt_free(driver);
	return RT_EOK;
}

