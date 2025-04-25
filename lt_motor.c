#include "ltmotorlib.h"

extern struct lt_motor_manager_object * _manager;
lt_motor_t lt_motor_create(char* name, rt_uint8_t reduction_ration, rt_uint8_t type)
{
	lt_motor_t _motor;
	if(reduction_ration == 0) reduction_ration = 1;
	
#ifdef LT_USING_MOTOR_MSH_TEST
	/* check whether motor is in manager */
	_motor = lt_manager_get_motor(name);
#endif
	if(_motor != RT_NULL)	return RT_NULL;		/* motor is already in the mananger */
	
	switch(type)
	{
		case MOTOR_TYPE_DC:
		{
			_motor = _motor_dc_ops.create(name,reduction_ration,type);
			break;
		}
//		case MOTOR_TYPE_BLDC:
//		{
//			motor = _motor_bldc_ops.create(name,reduction_ration,type);
//			break;
//		}
		case MOTOR_TYPE_STEPPER:
		{
			_motor = _motor_stepper_ops.create(name,reduction_ration,type);
		}
	}
#ifdef LT_USING_MOTOR_MSH_TEST
	lt_manager_add_motor(_motor);		/* add motor to manager */
#endif
	return _motor;
}

rt_err_t lt_motor_set_driver(lt_motor_t motor, lt_driver_t driver)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(driver != RT_NULL);
	motor->driver = driver;
	return RT_EOK;
}

rt_err_t lt_motor_set_callback(lt_motor_t motor, rt_err_t (*callback)(void*) )
{
	RT_ASSERT(motor != RT_NULL);
	motor->callback = callback;
	return RT_EOK;
}

rt_err_t lt_motor_set_sensor(lt_motor_t motor, lt_sensor_t sensor)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(sensor != RT_NULL);
	lt_sensor_calibrate(sensor);		/* calibrate sensor before we use it */
	motor->sensor = sensor;
	
	return RT_EOK;
}

float lt_motor_get_velocity(lt_motor_t motor, rt_uint32_t measure_time_ms)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(motor->sensor != RT_NULL);
	float speed = lt_sensor_get_velocity(motor->sensor,measure_time_ms*1000)/motor->reduction_ratio;	/* ms --> us */
	return speed;								/* unit: rad/s */		
}

float lt_motor_get_position(lt_motor_t motor)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(motor->sensor != RT_NULL);
	return lt_sensor_get_angle(motor->sensor);	/* unit: rad */
}

rt_err_t lt_motor_get_info(lt_motor_t motor, struct lt_motor_info* info)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(info != RT_NULL);
	rt_strcpy(info->name,motor->name);
	info->status = motor->status;
	info->type = motor->type;
	if(motor->sensor == RT_NULL)
	{
		info->position = 0;
		return RT_ERROR;
	}
	else
	{
		info->position = lt_sensor_get_angle(motor->sensor);
		return RT_EOK;
	}
}

rt_err_t lt_motor_control(lt_motor_t motor, int cmd, void* arg)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(motor->ops != RT_NULL);
	RT_ASSERT(motor->ops->control != RT_NULL);
	switch(cmd)
	{
		case MOTOR_CTRL_GET_STATUS:
		{
			rt_uint8_t* status = (rt_uint8_t *)arg;
			*status = motor->status;
			break;
		}
		case MOTOR_CTRL_GET_POSITION:
		{
			float* position = (float*)arg;
			*position = lt_motor_get_position(motor);
			break;
		}
		case MOTOR_CTRL_GET_VELOCITY:
		{
			float* velocity = (float*)arg;
			*velocity = lt_motor_get_velocity(motor,*velocity);
			break;
		}
		default:
		{
			motor->ops->control(motor,cmd,arg);
			break;
		}
	}
	
	return RT_EOK;
}

rt_err_t lt_motor_enable(lt_motor_t motor,rt_uint8_t dir)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(motor->driver != RT_NULL);
	motor->status = MOTOR_STATUS_RUN;
	return lt_driver_enable(motor->driver,dir);
}

rt_err_t lt_motor_disable(lt_motor_t motor)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(motor->driver != RT_NULL);
	motor->status = MOTOR_STATUS_STOP;
	return lt_driver_disable(motor->driver);
}

rt_err_t lt_motor_delete(lt_motor_t motor)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(motor->ops != RT_NULL);
	RT_ASSERT(motor->ops->_delete != RT_NULL);
	if(motor->driver != RT_NULL)
	{
		lt_driver_disable(motor->driver);
	}
#ifdef LT_USING_MOTOR_MSH_TEST
	lt_manager_delete_motor(motor);				/* move motor from the manager */
#endif
	return motor->ops->_delete(motor);
}

char * _status[4] = {"STOP","RUN","ACCELERATE","INTERP"};
char* _type[4] = {"UNKNOWN","DC","BLDC","STEPPER"};
