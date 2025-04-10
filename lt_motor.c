#include "lt_motor_control.h"

lt_motor_t lt_motor_create(char* name, rt_uint8_t reduction_ration, rt_uint8_t type)
{
	lt_motor_t motor;
	switch(type)
	{
		case MOTOR_TYPE_DC:
		{
			motor = _motor_dc_ops.create(name,reduction_ration,type);
			break;
		}
//		case MOTOR_TYPE_BLDC:
//		{
//			motor = _motor_bldc_ops.create(name,reduction_ration,type);
//			break;
//		}
		case MOTOR_TYPE_STEPPER:
		{
			motor = _motor_stepper_ops.create(name,reduction_ration,type);
		}
	}
	return motor;
}

rt_err_t lt_motor_set_pwm(lt_motor_t motor, rt_uint16_t pwm_num,rt_uint8_t pwm_channel)
{
	RT_ASSERT(motor != RT_NULL);
	struct rt_device_pwm* _pwm;
	switch(pwm_num)
	{
		case PWM_NUM_1:
			_pwm = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME_1);
			break;
		case PWM_NUM_2:
			_pwm = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME_2);
			break;
		case PWM_NUM_3:
			_pwm = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME_3);
			break;
		case PWM_NUM_4:
			_pwm = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME_4);
			break;
		case PWM_NUM_5:
			_pwm = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME_5);
			break;
		default: break;
	}
	
	if(_pwm != RT_NULL)
	{
		rt_pwm_set(_pwm,pwm_channel,MOTOR_PERIOD,0);
	}
	else return RT_ERROR;
	
	motor->pwm = _pwm;
	motor->pwm_channel = pwm_channel;
	return RT_EOK;
}

rt_err_t lt_motor_set_encoder(lt_motor_t motor, rt_uint16_t encoder_num,rt_uint16_t resolution)
{
	rt_device_t _encoder;
	switch(encoder_num)
	{
		case PULSE_ENCODER_NUM_1:
			_encoder = rt_device_find(PULSE_ENCODER_DEV_NAME_1);
			break;
		case PULSE_ENCODER_NUM_2:
			_encoder = rt_device_find(PULSE_ENCODER_DEV_NAME_2);
			break;
		case PULSE_ENCODER_NUM_3:
			_encoder = rt_device_find(PULSE_ENCODER_DEV_NAME_3);
			break;
		case PULSE_ENCODER_NUM_4:
			_encoder = rt_device_find(PULSE_ENCODER_DEV_NAME_4);
			break;
		case PULSE_ENCODER_NUM_5:
			_encoder = rt_device_find(PULSE_ENCODER_DEV_NAME_5);
			break;
		default: break;
	}
	
	if(_encoder != RT_NULL)
	{
		rt_device_open(_encoder,RT_DEVICE_OFLAG_RDONLY);
		rt_device_control(_encoder, PULSE_ENCODER_CMD_CLEAR_COUNT,RT_NULL);	/* clear count */
	}
	else return RT_ERROR;
	
	motor->encoder = _encoder;
	motor->resolution = resolution;
	motor->encoder_count = 0;
	return RT_EOK;
}

rt_err_t lt_motor_set_dir_pins(lt_motor_t motor, rt_base_t forward_pin, rt_base_t reversal_pin)
{
	RT_ASSERT(motor != RT_NULL);
	rt_pin_mode(forward_pin,PIN_MODE_OUTPUT);
	rt_pin_mode(reversal_pin,PIN_MODE_OUTPUT);
	
	motor->forward_pin = forward_pin;
	motor->reversal_pin = reversal_pin;
	return RT_EOK;
}


float lt_motor_measure_speed(lt_motor_t motor, rt_uint32_t measure_time_ms)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(motor->ops != RT_NULL);
	RT_ASSERT(motor->ops->measure_speed != RT_NULL);
	
	return motor->ops->measure_speed(motor,measure_time_ms);
}

float lt_motor_measure_position(lt_motor_t motor)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(motor->ops != RT_NULL);
	RT_ASSERT(motor->ops->measure_position != RT_NULL);

	return motor->ops->measure_position(motor);
}

rt_err_t lt_motor_control(lt_motor_t motor, int cmd, void* arg)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(motor->ops != RT_NULL);
	RT_ASSERT(motor->ops->control != RT_NULL);
	if(cmd == MOTOR_CTRL_GET_STATUS)
	{
		rt_uint8_t* status = (rt_uint8_t *)arg;
		*status = motor->status;
		return RT_EOK;
	}
	else
	{
		return motor->ops->control(motor,cmd,arg);
	}
}

rt_err_t lt_motor_set_ops(lt_motor_t motor, struct lt_motor_ops * ops)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(ops != RT_NULL);
	motor->ops = ops;
	return RT_EOK;
}

char * _status[4] = {"STOP","RUN","ACCELERATE","INTERP"};
