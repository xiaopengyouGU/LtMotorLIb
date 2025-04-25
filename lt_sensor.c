#include "ltmotorlib.h"

extern struct lt_sensor_ops _sensor_encoder_ops;
//extern struct lt_sensor_ops _sensor_magnetic_ops;

lt_sensor_t lt_sensor_create(rt_uint16_t sensor_num, rt_uint16_t resolution, rt_uint8_t type)
{
	lt_sensor_t _sensor;
	
	switch(type)
	{
		case SENSOR_TYPE_ENCODER:
			_sensor = _sensor_encoder_ops.create(sensor_num,resolution,type);
			break;
//		case SENSOR_TYPE_MAGNETIC:
//			_sensor = _sensor_magnetic_ops.create(sensor_num,resolution,type);
//			break;
		default: break;
	}
	return _sensor;
}


rt_err_t lt_sensor_set_pins(lt_sensor_t sensor,rt_base_t pin_A,rt_base_t pin_B,rt_base_t pin_C )
{
	RT_ASSERT(sensor != RT_NULL);
	RT_ASSERT(sensor->ops != RT_NULL);
	RT_ASSERT(sensor->ops->set_pins != RT_NULL);
	
	sensor->pin_A = pin_A;
	sensor->pin_B = pin_B;
	sensor->pin_C = pin_C;
	
	return sensor->ops->set_pins(sensor);
}

float lt_sensor_get_angle(lt_sensor_t sensor)
{
	RT_ASSERT(sensor != RT_NULL);
	RT_ASSERT(sensor->ops != RT_NULL);
	RT_ASSERT(sensor->ops->get_angle != RT_NULL);
	return sensor->ops->get_angle(sensor);
}

float lt_sensor_get_velocity(lt_sensor_t sensor, rt_uint32_t measure_time_us)
{
	RT_ASSERT(sensor != RT_NULL);
	RT_ASSERT(sensor->ops != RT_NULL);
	RT_ASSERT(sensor->ops->get_velocity != RT_NULL);
	return sensor->ops->get_velocity(sensor,measure_time_us);
}

rt_err_t lt_sensor_calibrate(lt_sensor_t sensor)
{
	RT_ASSERT(sensor != RT_NULL);
	RT_ASSERT(sensor->ops != RT_NULL);
	RT_ASSERT(sensor->ops->calibrate != RT_NULL);
	sensor->curr_val = 0;
	
	return sensor->ops->calibrate(sensor);
}

rt_err_t lt_sensor_delete(lt_sensor_t sensor)
{
	RT_ASSERT(sensor != RT_NULL);
	if(sensor->lpf != RT_NULL)
	{
		lt_filter_delete(sensor->lpf);
		sensor->lpf = RT_NULL;
	}
	rt_free(sensor);
	
	return RT_EOK;
}
