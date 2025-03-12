#include "lt_motor_control.h"


static lt_motor_t _motor_dc_create(char* name,rt_uint8_t reduction_ration,rt_uint8_t type)
{
	lt_motor_t _motor;
	_motor = rt_malloc(sizeof(struct lt_motor_object));
	rt_memset(_motor,0,sizeof(struct lt_motor_object));
	rt_strcpy(_motor->name,name);
	
	_motor->reduction_ratio = reduction_ration;
	_motor->type = type;
	
	return _motor;
}

static float _motor_dc_measure_speed(lt_motor_t motor, rt_uint32_t measure_time_ms)
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

static float _motor_dc_measure_position(lt_motor_t motor)
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


static rt_err_t _motor_dc_control(lt_motor_t motor, int cmd, void*arg)
{
	RT_ASSERT(motor != RT_NULL);
	
	switch(cmd)
	{
		case MOTOR_CTRL_OUTPUT:
		{
			//struct rt_device_pwm* pwm = motor->pwm;
			float input,output;
			input = *(float *)arg;
	
			if(input > 0 )		/* forward rotation */
			{
				rt_pin_write(motor->forward_pin,PIN_HIGH);
				rt_pin_write(motor->reversal_pin,PIN_LOW);
			}
			else if(input < 0)
			{					/* reversal rotation */
				input = -input;
				rt_pin_write(motor->forward_pin,PIN_LOW);
				rt_pin_write(motor->reversal_pin,PIN_HIGH);
			}
			else
			{
				rt_pin_write(motor->forward_pin,PIN_LOW);
				rt_pin_write(motor->reversal_pin,PIN_LOW);
			}

			if(input > PWM_MAX_VAL)			/* actuator saturation */
			{
				output = PWM_MAX_VAL;
			}
			else if(input < PWM_MIN_VAL)	/* dead region */
			{
				output = 0;
			}
			else
			{
				output = input;
			}
			output = output*PWM_PERIOD/PWM_MAX_VAL;
			rt_pwm_set(motor->pwm,motor->pwm_channel,PWM_PERIOD,(rt_uint32_t)(output));
			//rt_pwm_set_pulse(motor->pwm,motor->pwm_channel,(rt_uint32_t)(output));
			rt_pwm_enable(motor->pwm,motor->pwm_channel);
			break;
		}
		case MOTOR_CTRL_OUTPUT_ANGLE:
		{
			//struct rt_device_pwm* pwm = motor->pwm;
			float angle = *(float *)arg;
			if(angle < 0) angle = 0;
			else if(angle >= MOTOR_MAX_ANGLE) angle = MOTOR_MAX_ANGLE;
			/* transform angle to pulse */
			angle = 500000+(angle/MOTOR_MAX_ANGLE)*2000000;		/* correspond to 0.5ms - 2.5ms */
			rt_pwm_set(motor->pwm,motor->pwm_channel,20000000,(rt_uint32_t)angle);	/* period : 20ms */
			rt_pwm_enable(motor->pwm,motor->pwm_channel);
			break;
		}
		case MOTOR_CTRL_MEASURE_SPEED:
		{
			float *res_ms = (float *)arg;
			*res_ms = _motor_dc_measure_speed(motor,(rt_uint32_t)(*res_ms));
			break;
		}
		case MOTOR_CTRL_MEASURE_POSITION:
		{
			float *res = (float *)arg;
			*res = _motor_dc_measure_position(motor);
			break;
		}
		default:break;
	}
	
	return RT_EOK;
		
}

struct lt_motor_ops _motor_dc_ops = {
										_motor_dc_create,
										_motor_dc_control,
										_motor_dc_measure_speed,
										_motor_dc_measure_position,
									 };