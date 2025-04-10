#include "lt_motor_control.h"

lt_pid_t lt_pid_create(float Kp, float Ki, float Kd, float dt_ms)
{
	lt_pid_t pid;
	pid = rt_malloc(sizeof(struct lt_pid_object));
	rt_memset(pid,0,sizeof(struct lt_pid_object));
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->control_u = 0.0;
	pid->err = 0.0;
	pid->err_last = 0.0;
	pid->err_prev = 0.0;
	pid->integral= 0.0;
	pid->target_val = 0.0;
	pid->dt = dt_ms/1000;
	pid->int_limit = PID_INTEGRAL_LIMIT;			/* default integral limit */
	pid->output_limit = PID_OUTPUT_LIMIT;			/* default output limit */
	return pid;
}

rt_err_t lt_pid_delete(lt_pid_t pid)
{
	RT_ASSERT(pid != RT_NULL);
	rt_free(pid);
	return RT_EOK;
}

void lt_pid_reset(lt_pid_t pid)
{
	RT_ASSERT(pid != RT_NULL);
	pid->control_u = 0.0;
	pid->err = 0.0;
	pid->err_prev = 0.0;
	pid->err_last = 0.0;
	pid->integral = 0.0;
}

void lt_pid_set(lt_pid_t pid, float Kp, float Ki, float Kd)
{
	RT_ASSERT(pid != RT_NULL);
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
}
void lt_pid_set_target(lt_pid_t pid, float target)
{
	RT_ASSERT(pid != RT_NULL);
	pid->target_val = target;
}

void lt_pid_set_dt(lt_pid_t pid, float dt_ms)
{
	RT_ASSERT(pid != RT_NULL);
	pid->dt = dt_ms/1000;
}

void lt_pid_set_output_limit(lt_pid_t pid,float limit)
{
	RT_ASSERT(pid != RT_NULL);
	if(limit < 0) limit = -limit;
	pid->output_limit = limit;
}
void lt_pid_set_int_limit(lt_pid_t pid,float limit)
{
	RT_ASSERT(pid != RT_NULL);
	if(limit < 0) limit = -limit;
	pid->int_limit = limit;
}

float lt_pid_get_control(lt_pid_t pid)
{
	RT_ASSERT(pid != RT_NULL);
	return pid->control_u;
}

float lt_pid_control(lt_pid_t pid,float curr_val)
{
	RT_ASSERT(pid != RT_NULL);
	pid->err = pid->target_val - curr_val;
	pid->integral += pid->err;
	
	if(pid->integral >= pid->int_limit)		/* anti-windup */
	{
		pid->integral = pid->int_limit;
	}
	else if(-pid->integral >= pid->int_limit)
	{
		pid->integral = -pid->int_limit;
	}
	/* output limit */
	pid->control_u = pid->Kp*pid->err + pid->Ki*pid->integral*pid->dt + pid->Kd*(pid->err - pid->err_prev)/pid->dt;
	if(pid->control_u >= pid->output_limit)
	{
		pid->control_u = pid->output_limit;
	}
	else if(-pid->control_u >= pid->output_limit)
	{
		pid->control_u = -pid->output_limit;
	}
	
	pid->err_prev = pid->err;
	
	return pid->control_u;
}

float lt_pid_incre_control(lt_pid_t pid, float curr_val)
{
	RT_ASSERT(pid != RT_NULL);
	pid->err = pid->target_val - curr_val;
	
	float increment = pid->Kp*(pid->err - pid->err_prev)+ pid->Ki*pid->err*pid->dt + pid->Kd*(pid->err - 2*pid->err_prev + pid->err_last)/pid->dt;
	
	pid->err_last = pid->err_prev;
	pid->err_prev = pid->err;
	
	pid->control_u += increment;
	/* output limit */
	if(pid->control_u >= pid->output_limit)
	{
		pid->control_u = pid->output_limit;
	}
	else if(-pid->control_u >= pid->output_limit)
	{
		pid->control_u = -pid->output_limit;
	}
	
	return pid->control_u;
}
