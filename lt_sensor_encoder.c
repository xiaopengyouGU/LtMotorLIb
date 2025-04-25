#include "ltmotorlib.h"

#define _TF				0.003	/* unit: s */

struct lt_sensor_ops _sensor_encoder_ops;
static lt_sensor_t _sensor_encoder_create(rt_uint16_t sensor_num,rt_uint16_t resolution, rt_uint8_t type)
{
	lt_sensor_t _sensor;
	lt_filter_t _filter;
	rt_device_t _encoder;
	
	switch(sensor_num)
	{
		case SENSOR_NUM_ENCODER_1:
			_encoder = rt_device_find(ENCODER_DEV_NAME_1);
			break;
		case SENSOR_NUM_ENCODER_2:
			_encoder = rt_device_find(ENCODER_DEV_NAME_2);
			break;
		case SENSOR_NUM_ENCODER_3:
			_encoder = rt_device_find(ENCODER_DEV_NAME_3);
			break;
		case SENSOR_NUM_ENCODER_4:
			_encoder = rt_device_find(ENCODER_DEV_NAME_4);
			break;
		case SENSOR_NUM_ENCODER_5:
			_encoder = rt_device_find(ENCODER_DEV_NAME_5);
			break;
		default: break;
	}
	
	if(_encoder != RT_NULL)
	{
		rt_device_open(_encoder,RT_DEVICE_OFLAG_RDONLY);
		rt_device_control(_encoder, PULSE_ENCODER_CMD_CLEAR_COUNT,RT_NULL);	/* clear count */
	}
	else
	{
		return RT_NULL;
	}
	
	_sensor = rt_malloc(sizeof(struct lt_sensor_object));
	_filter = lt_filter_create(_TF,0);										/* create a low pass filter */
	if(_sensor == RT_NULL) return RT_NULL;
	if(_filter == RT_NULL) return RT_NULL;
	
	if(_sensor != RT_NULL)
	{
		rt_memset(_sensor,0,sizeof(struct lt_sensor_object));
		_sensor->curr_val = 0;
		_sensor->resolution = resolution;
		_sensor->type = type;
		_sensor->dev = _encoder;
		_sensor->ops = &_sensor_encoder_ops;
		_sensor->lpf = _filter;
	}
		
	return _sensor;
}

rt_err_t _sensor_encoder_set_pins(lt_sensor_t sensor)
{
	return RT_EOK;
}

float _sensor_encoder_get_angle(lt_sensor_t sensor)
{
	
	rt_device_t encoder = sensor->dev;
	rt_int32_t count;
	float position;
	
	rt_device_open(encoder,RT_DEVICE_OFLAG_RDONLY);
	rt_device_read(encoder,0,&count,1);
	
	position = (float)(count)/(sensor->resolution);		/* notice direction! */
	position *= 2*PI;									/* unit: rad */
	
	return position;
}

float _sensor_encoder_get_velocity(lt_sensor_t sensor,rt_uint32_t measure_time_us)
{
	rt_device_t encoder = sensor->dev;
	rt_int32_t curr_count;
	float velocity;
	
	rt_device_open(encoder,RT_DEVICE_OFLAG_RDONLY);
	rt_device_read(encoder,0,&curr_count,1);
	sensor->curr_val = curr_count;						/* refrech encoder count! */
	/* use M method to measure velocity */
	if(measure_time_us == 0) return 0;					/* unvalid measure timer */
	velocity = (float)(sensor->curr_val - curr_count)*1000000.0f/measure_time_us;
	velocity /= (sensor->resolution)*2*PI;				/* unit: rad/s */
	/* low pass filter process */
	lt_filter_set_dt(sensor->lpf,measure_time_us/1000000.0f);	/* uint: s */
	velocity = lt_filter_process(sensor->lpf,velocity);
	
	return velocity;
}

rt_err_t _sensor_encoder_calibrate(lt_sensor_t sensor)
{
	rt_device_t encoder = sensor->dev;
	sensor->curr_val = 0;
	
	rt_device_open(encoder,RT_DEVICE_OFLAG_RDONLY);
	rt_device_control(encoder,PULSE_ENCODER_CMD_CLEAR_COUNT,RT_NULL);	/* clear pulse encoder */
	rt_device_close(encoder);
	
	return RT_EOK;
}

struct lt_sensor_ops _sensor_encoder_ops = {	_sensor_encoder_create,
												_sensor_encoder_set_pins,
												_sensor_encoder_get_angle,
												_sensor_encoder_get_velocity,
												_sensor_encoder_calibrate,
};