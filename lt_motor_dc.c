#include "lt_motor_control.h"

static void _motor_dc_output(lt_motor_t, float input);
static void _motor_dc_output_angle(lt_motor_t,float angle);

static lt_motor_t _motor_dc_create(char* name,rt_uint8_t reduction_ration,rt_uint8_t type)
{
	lt_motor_t _motor;
	_motor = rt_malloc(sizeof(struct lt_motor_object));
	rt_memset(_motor,0,sizeof(struct lt_motor_object));
	rt_strcpy(_motor->name,name);
	
	_motor->reduction_ratio = reduction_ration;
	_motor->type = type;
	_motor->ops = &_motor_dc_ops;		/* set operators */
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
			float input = *(float *)arg;
			_motor_dc_output(motor,input);
			break;
		}
		case MOTOR_CTRL_OUTPUT_ANGLE:
		{
			float angle = *(float *)arg;
			_motor_dc_output_angle(motor,angle);
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

static void _motor_dc_output(lt_motor_t motor, float input)
{
	float output;
	motor->status = MOTOR_STATUS_RUN;
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

	if(input > MOTOR_MAX_SPEED)			/* actuator saturation */
	{
		output = MOTOR_MAX_SPEED;
	}
	else if(input < MOTOR_MIN_SPEED)	/* dead region */
	{
		output = 0;
		motor->status = MOTOR_STATUS_STOP;
	}
	else
	{
		output = input;
	}
	output = output*MOTOR_PERIOD/MOTOR_MAX_VAL;
	rt_pwm_set(motor->pwm,motor->pwm_channel,MOTOR_PERIOD,(rt_uint32_t)(output));
	rt_pwm_enable(motor->pwm,motor->pwm_channel);
}

static void _motor_dc_output_angle(lt_motor_t motor,float angle)
{
	motor->status = MOTOR_STATUS_RUN;
	if(angle < 0){
		angle = 0;
		motor->status = MOTOR_STATUS_STOP;
	}
	else if(angle >= MOTOR_MAX_ANGLE)
	{
		angle = MOTOR_MAX_ANGLE;
	}
	/* transform angle to pulse */
	angle = MOTOR_ANGLE_BASE+(angle/MOTOR_MAX_ANGLE)*MOTOR_ANGLE_PERIOD;		/* correspond to 0.5ms - 2.5ms */
	rt_pwm_set(motor->pwm,motor->pwm_channel,MOTOR_ANGLE_PERIOD,(rt_uint32_t)angle);	/* period : 20ms */
	rt_pwm_enable(motor->pwm,motor->pwm_channel);
}