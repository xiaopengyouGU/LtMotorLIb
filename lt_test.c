#include "ltmotorlib.h"
#include "protocol.h"
#include "stdlib.h"

/* when communicating with upper computer, FINSH is forbidden */
/* basic command, eg: MOTOR_CTRL_OUTPUT	works on all motor
*  in fact, MOTOR_CTRL_OUTPUT = STEPPER_CTRL_OUPUT = BLDC_CTRL_OUTPUT
*/
/*************************************  setup table  **************************************************/

#ifdef RT_USING_FINSH 
#define TIMEOUT_FUNC					pid_test			/* hardware timer callback function */
//#define TIMEOUT_FUNC					velocity_loop				
//#define TIMEOUT_FUNC					position_loop
//#define TIMEOUT_FUNC					position_velocity_loop
#endif
/*************************************  setup table  **************************************************/
/* ADC --> PA6
*  DAC --> PA4
*  DAC --> PA5
*  PWM --> PA3
*  Encoder --> PB6 £¬ PB7
*  stepper motor driver is common-cathode
*/

rt_align(RT_ALIGN_SIZE)

/* dc motor config */
static lt_motor_t motor_dc;
static lt_driver_t driver_dc;
static lt_sensor_t sensor_dc;
static lt_timer_t timer_dc;

static rt_thread_t process_thread;
static lt_pid_t pid_dc;			/* motor pid */
static lt_pid_t pid_pos_dc;		/* position pid */
static lt_pid_t pid_vel_dc;		/* velocity pid */

rt_err_t dc_callback(void *parameter)
{
	return rt_kprintf("motor_dc finished a output! \n");
}

rt_err_t test_motor_dc_config(void)
{
	/* create motor, driver, sensor, get pin number */
	rt_base_t forward = rt_pin_get("PF.8");
	rt_base_t reversal = rt_pin_get("PF.7");
	
	motor_dc = lt_motor_create("motor_dc",34,MOTOR_TYPE_DC);
	driver_dc = lt_driver_create(DRIVER_TYPE_DC);
	sensor_dc = lt_sensor_create(ENCODER_NAME_1,4*13,SENSOR_TYPE_ENCODER);
	timer_dc = lt_timer_create("dc_timer",TIMER_NAME_1,0);				
	pid_vel_dc = lt_pid_create(1.96,1.6,0.01,100);							/* sample time: 100ms, unit: ms */
	pid_pos_dc = lt_pid_create(10.1,23,0.01,100);							
	/* timer is not needed in all case, so as sensor */
	/* check res */
	if(motor_dc == RT_NULL || driver_dc == RT_NULL || sensor_dc == RT_NULL) return RT_ERROR;
	if(timer_dc == RT_NULL || pid_vel_dc == RT_NULL || pid_pos_dc) return RT_ERROR;	

	/* config driver, here we use default config */
	lt_driver_set_pwm(driver_dc,PWM_NAME_1,4,0);					/* set pwm and channel */
	lt_driver_set_pins(driver_dc,forward, reversal,0);						/* set pins */
	//lt_driver_set_output(driver,1000,0.5,0);							/* set pwm period and duty_cycle, unit: us */
	lt_sensor_calibrate(sensor_dc);										/* calibrate sensor */
	
	/* config motor */
	lt_motor_set_driver(motor_dc,driver_dc);
	lt_motor_set_sensor(motor_dc,sensor_dc);
	lt_motor_set_timer(motor_dc,timer_dc);
	lt_motor_set_pid(motor_dc,pid_vel_dc,PID_TYPE_VEL);							/* Ltmotorlib provided simple pid interface for velocity and position pid */
	lt_motor_set_pid(motor_dc,pid_pos_dc,PID_TYPE_POS);
	lt_motor_set_callback(motor_dc,dc_callback);									/* when finished a output, motor object will call callback function */
	
	return RT_EOK;
}


/* x stepper config */
static lt_motor_t x_stepper;
static lt_driver_t x_driver; 
static lt_sensor_t x_encoder;
static lt_timer_t x_timer;
static lt_pid_t pid_vel_x;
static lt_pid_t pid_pos_x;
/* when finish output, this function would be called */
rt_err_t x_callback(void *parameter)
{
	return rt_kprintf("x_stepper finished a output! \n");
}

rt_err_t test_stepper_x_config(void)
{
	/* create motor, driver, sensor, get pin number */
	rt_base_t forward = rt_pin_get("PD.13");
	rt_base_t enable = rt_pin_get("PD.12");
	struct lt_stepper_config config;
	
	x_stepper = lt_motor_create("x_stepper",1,MOTOR_TYPE_STEPPER);
	x_driver = lt_driver_create(DRIVER_TYPE_STEPPER);
	x_encoder = lt_sensor_create(ENCODER_NAME_1,4*600,SENSOR_TYPE_ENCODER);
	x_timer = lt_timer_create("x_timer",TIMER_NAME_2,0);				
	pid_vel_x = lt_pid_create(1,8.2,0.3,50);							   /* sample time: 50ms, unit: ms */
	pid_pos_x = lt_pid_create(1,8.2,0.3,50);							   /* sample time: 50ms, unit: ms */
	/* timer is not needed in all case, so as sensor */
	/* check res */
	if(x_stepper == RT_NULL || x_driver == RT_NULL || x_encoder == RT_NULL) return RT_ERROR;
	if(x_timer == RT_NULL || pid_vel_x == RT_NULL  || pid_pos_x) return RT_ERROR;	
	
	/* config driver, here we use default config */
	lt_driver_set_pwm(x_driver,PWM_NAME_1,2,0);				/* set pwm and channel */
	lt_driver_set_pins(x_driver,forward,0,enable);						/* set pins */
	//lt_driver_set_output(driver,1000,0.5,0);							/* set pwm period and duty_cycle, unit: us */
	lt_sensor_calibrate(x_encoder);										/* calibrate sensor */
	
	/* config motor */
	lt_motor_set_driver(x_stepper,x_driver);
	lt_motor_set_sensor(x_stepper,x_encoder);
	lt_motor_set_timer(x_stepper,x_timer);
	lt_motor_set_pid(x_stepper,pid_vel_x,PID_TYPE_VEL);					/* Ltmotorlib provided simple pid interface for velocity and position pid */
	lt_motor_set_pid(x_stepper,pid_pos_x,PID_TYPE_POS);
	lt_motor_set_callback(x_stepper,x_callback);						/* when finished a output, motor object will call callback function */
	
	/* stepper part config */
	config.period = 20;													/* 20ms unit: ms */
	config.stepper_angle = 1.8;											/* unit: degree */
	config.subdivide = 2;												
	return lt_motor_control(x_stepper,STEPPER_CTRL_CONFIG,&config);
}

/* y stepper config:
*  y_stepper is open loop output, so we don't configure position sensor and pid
*  therefore accleration function and pid output function are forbiddened;
*/
static lt_motor_t y_stepper;
static lt_driver_t y_driver; 
static lt_timer_t y_timer;
/* when finish output, this function would be called */
rt_err_t y_callback(void *parameter)
{
	return rt_kprintf("y_stepper finished a output! \n");
}

rt_err_t test_stepper_y_config(void)
{
	/* create motor, driver, sensor, get pin number */
	rt_base_t forward = rt_pin_get("PE.11");
	rt_base_t enable = rt_pin_get("PF.13");
	struct lt_stepper_config config;
	
	y_stepper = lt_motor_create("y_stepper",1,MOTOR_TYPE_STEPPER);
	y_driver = lt_driver_create(DRIVER_TYPE_STEPPER);
	y_timer = lt_timer_create("y_timer","",0);						/* we don't select hardware timer */			
	/* timer is not needed in all case, so as sensor */
	/* check res */
	if(y_stepper == RT_NULL || y_driver == RT_NULL || y_timer == RT_NULL) return RT_ERROR;

	/* config driver, here we use default config */
	lt_driver_set_pwm(y_driver,PWM_NAME_1,1,0);				/* set pwm and channel */
	lt_driver_set_pins(y_driver,forward,0,enable);						/* set pins */
	//lt_driver_set_output(driver,1000,0.5,0);							/* set pwm period and duty_cycle, unit: us */
	/* we use default stepper timer config */
	
	/* config motor */
	lt_motor_set_driver(y_stepper,y_driver);
	lt_motor_set_timer(y_stepper,y_timer);
	lt_motor_set_callback(y_stepper,y_callback);						/* when finished a output, motor object will call callback function */
	
	/* stepper part config */
	config.period = 20;													/* 20ms unit: ms */
	config.stepper_angle = 1.8;											/* unit: degree */
	config.subdivide = 2;												
	return lt_motor_control(y_stepper,STEPPER_CTRL_CONFIG,&config);
}

//#ifndef LT_USING_MOTOR_MSH_TEST
//#define TEST_PID
//#define TEST_PID_VEL
//#define TEST_PID_POS
#define TEST_PID_TWO_LOOP

static rt_thread_t process_thread;
#ifdef TEST_PID
static void pid_test(lt_timer_t timer)
{
	/* eg: KP = 0.26, KI = 0.9, KD = 0.02 */
	lt_motor_t motor = (lt_motor_t)timer->user_data;
	lt_pid_t pid = (lt_pid_t)motor->user_data;
	float read_val = lt_pid_get_control(pid);
	float curr_val = lt_pid_control(pid,read_val);
	int temp = curr_val;									/* transform data */
	lt_communicator_send(SEND_FACT_CMD,CURVES_CH1,&temp,1);	/* send fact value to upper computer */
}
#endif

#ifdef TEST_PID_VEL
static void velocity_loop(lt_timer_t timer)
{
	/* eg: KP = 1.96, KI = 1.6, KD = 0.01 */
	lt_motor_t motor = (lt_motor_t)timer->user_data;
	lt_pid_t pid = (lt_pid_t)motor->user_data;
	float vel = lt_motor_get_velocity(motor,pid->dt) * 9.55;		/* get rpm */
	float control_u = PID_VEL_CONST * lt_pid_control(pid,vel);		/* multiply a coefficiency, default: 0.1f */
	int t_vel = vel, t_control_u = control_u;							
	lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&control_u);			/* output */
	/* send data to upper computer */
	lt_communicator_send(SEND_FACT_CMD,CURVES_CH1,&t_vel,1);
	lt_communicator_send(SEND_FACT_CMD,CURVES_CH2,&t_control_u,1);
}
#endif

#ifdef TEST_PID_POS
static void position_loop(lt_timer_t timer)
{
	/* eg: KP = 10.1, KI = 23, KD = 0.01 */
	lt_motor_t motor = (lt_motor_t)timer->user_data;
	lt_pid_t pid = (lt_pid_t)motor->user_data;
	float position = lt_motor_get_position(motor)*180.0f/PI;		/* get motor angle, unit: degree */
	float control_u = PID_POS_CONST * lt_pid_control(pid,position);	/* multiply 0.01f as default  */
	lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&control_u);
	int t_position = position, t_control_u = control_u * 100;	    /* let control curve smoother */	
	/* send data to upper computer */
	lt_communicator_send(SEND_FACT_CMD,CURVES_CH1,&t_position,1);
	lt_communicator_send(SEND_FACT_CMD,CURVES_CH2,&t_control_u,1);
}
#endif

#ifdef TEST_PID_TWO_LOOP
static void position_velocity_loop(lt_timer_t timer)
{
	lt_motor_t motor = (lt_motor_t)timer->user_data;
	lt_pid_t pid_vel = (lt_pid_t)motor->user_data;
	lt_pid_t pid_pos = (lt_pid_t)pid_vel->user_data;
	/* we adjust vel_pid first */
	/* DC motor: Kp = 3.9, Ki = 1.5, Kd = 1.05 */
	/* DC motor: Kp = 1.8, Ki = 2, Kd = 0.3 */
	static rt_uint32_t count = 0;
	/* process position loop first, sample time 3T */
	if(count % 3 == 0)
	{
		float position = lt_motor_get_position(motor)*180.0f/PI;	/* unit: degree */
		float desired_vel = lt_pid_control(pid_pos,position);
		lt_pid_set_target(pid_vel,desired_vel);
		int t_position = position;
		lt_communicator_send(SEND_FACT_CMD,CURVES_CH1,&t_position,1);
	}
	/* velocity loop follow, sample time T */
	float vel = lt_motor_get_velocity(motor,pid_vel->dt)*9.55;	/* get rpm */
	float control_u = 0.01f * lt_pid_control(pid_vel,vel);
	lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&control_u);		/* output! */
	int t_speed = vel*100, t_control_u = control_u * 100;		/* let control and speed curves smoother */
	
	lt_communicator_send(SEND_FACT_CMD,CURVES_CH2,&t_speed,1);
	lt_communicator_send(SEND_FACT_CMD,CURVES_CH3,&t_control_u,1);
	count++;
	
}
#endif
/*************************************************************/
/* communicate with upper computer */
static void process_thread_entry(void* parameter)
{
	lt_timer_t timer = (lt_timer_t)parameter;
#ifndef TEST_PID	
	lt_motor_t motor = (lt_motor_t)timer->user_data;
	lt_pid_t pid = (lt_pid_t)motor->user_data;
#endif
	
#ifdef TEST_PID_TWO_LOOP	
	pid = (lt_pid_t)pid->user_data;
#endif
	rt_uint8_t cmd_type = CMD_NONE;     /* command type */
	struct lt_pid_info info;			/* pid info */
	
	while(1)
	{
		cmd_type = lt_communicator_receive(&info);
		switch (cmd_type)
		{
			case SET_PID_CMD:
			{
				lt_pid_set(pid,info.Kp, info.Ki, info.Kd);    /* set pid */
				break;
			}
			case SET_TARGET_CMD:
			{
				lt_pid_set_target(pid,info.target);    
				break;
			}
			case START_CMD:
			{
				/* in fact, we control motor in timer's timeout function */
				lt_timer_enable(timer,TIMER_TYPE_HW);
				break;
			}
			case STOP_CMD:
			{
				lt_motor_disable(motor);				/* disable motor and timer */
				lt_timer_disable(timer,TIMER_TYPE_HW);
				lt_communicator_send(SEND_STOP_CMD,CURVES_CH1,RT_NULL,0);
				break;
			}
			case RESET_CMD:
			{
				lt_pid_reset(pid);				 		/* reset pid! */
				lt_motor_disable(motor);				 /* disable motor and timer */
				lt_timer_disable(timer,TIMER_TYPE_HW);
				break;
			}
			case SET_PERIOD_CMD:
			{
				lt_pid_set_dt(pid,info.dt);
				lt_timer_set(timer,info.dt*1000,TIMER_MODE_PERIODIC,TIMER_TYPE_HW);
				break;
			}
			default:break;
		}
		rt_thread_mdelay(100);
  }
}
//#endif

/* motor test frame */
rt_err_t test_close_loop_pid(lt_motor_t motor,lt_timer_t timer,lt_pid_t pid_vel,lt_pid_t pid_pos,void(*timeout)(lt_timer_t))
{
	/* check parameter */
	if(timer == RT_NULL || timeout == RT_NULL) return RT_ERROR;
	if(pid_vel == RT_NULL && pid_pos == RT_NULL) return RT_ERROR;
	/* we already config motor and use user_data variable to transport data
	* software timer can be used if sample time is relatively large, such as 100ms */	
#ifdef TEST_PID
	lt_timer_set(timer,pid_vel->dt*1000,TIMER_MODE_PERIODIC,TIMER_TYPE_HW);	/* ms --> us */
#endif	
	
#ifdef TEST_PID_VEL	
	/* config velocity pid */	
	lt_timer_set(timer,pid_vel->dt*1000,TIMER_MODE_PERIODIC,TIMER_TYPE_HW);	/* ms --> us */
	motor->user_data = pid_vel;
#endif
	
#ifdef TEST_PID_POS
	lt_timer_set(timer,pid_vel->dt*1000,TIMER_MODE_PERIODIC,TIMER_TYPE_HW);	/* ms --> us */
	motor->user_data = pid_pos;
#endif
	
#ifdef TEST_PID_TWO_LOOP/* config two loop pid */
	lt_timer_set(timer,pid_vel->dt*1000,TIMER_MODE_PERIODIC,TIMER_TYPE_HW);	/* ms --> us */
	motor->user_data = pid_vel;
	pid_vel->user_data = pid_pos;
#endif

	timer->user_data = motor;
	lt_timer_set_timeout(timer,timeout,TIMER_TYPE_HW);					/* set timer timeout function */
	lt_timer_enable(timer,TIMER_TYPE_HW);								/* start hardware timer */
	
/* create process thread */
	process_thread = rt_thread_create("process",		/* create a process thread */
								process_thread_entry,
								timer,
								1024,
								9,				
								20);
	if(process_thread != RT_NULL)
	{
		rt_thread_startup(process_thread);			/* start thread */
	}
}


///* stepper for interpolation */
//static lt_motor_t x_stepper;
//static lt_motor_t y_stepper;
//static lt_driver_t x_driver;
//static lt_driver_t y_driver; 
//static lt_sensor_t x_encoder;
////static lt_sensor_t y_encoder;
//static rt_sem_t sem_test;				/* semaphore for test */
//static rt_thread_t stepper_thread;		/* stepper thread */
///* when finish output, this function would be called */
//rt_err_t done_callback(void *parameter)
//{
//	return rt_sem_release(sem_test);
//}

//static void stepper_thread_entry(void *parameter)
//{
//	rt_uint8_t i;
//	float output = 360, res;
//	for (i = 0; i < 5; i++)			/* 5 forward rotations */
//	{
//		lt_motor_control(x_stepper,STEPPER_CTRL_OUTPUT_ANGLE,&output);
//		rt_sem_take(sem_test,RT_WAITING_FOREVER);				/* waiting output finish */
//		rt_thread_mdelay(500);									/* delay */
//	}
//	output = -360;
//	for (i = 0; i < 5; i++)			/* 5 reversal rotations */
//	{
//		lt_motor_control(x_stepper,STEPPER_CTRL_OUTPUT_ANGLE,&output);
//		rt_sem_take(sem_test,RT_WAITING_FOREVER);				/* waiting output finish */
//		rt_thread_mdelay(500);									/* delay */
//	}
//	/* we only set x_stepper position sensor */
//	test_motor_get_position(x_stepper);
//	test_motor_get_velocity(x_stepper);
//	
//	/* line interpolation path: /\ -->  ---
//								\/     |   |
//										---	
//	*/
//	rt_int32_t line_path[8][2] = {{500,800},{500,-800},{-500,-800},{-500,800},
//								  {0,1000},{1000,0},{0,-1000},{-1000,0}};
//	rt_int32_t circular_path[9][2] = {{707,0},{500,500},{0,707},{-500,500},{-707,0},{-500,-500},{0,-707},{500,-500},{0,707}};
//	/* draw a circle with radius = 707 steps */
//	/* line interp test */							  
//	for(i = 0; i < 8; i++)
//	{
//		test_stepper_line_interp(x_stepper,y_stepper,line_path[i][0],line_path[i][1]);
//		rt_sem_take(sem_test,RT_WAITING_FOREVER);				/* waiting output finish */
//		rt_thread_mdelay(1000);									
//	}
//	/* circular interp test */
//	/* in this case, the circular should lie in one quadrant and interp direction should be reasonable 
//	*  eg: start:(50, 50), end:(71,0), direction should be CW since intep circular lies in 1th quadrant
//	*/
//	for(i = 0; i < 8; i++)
//	{
//		test_stepper_circular_interp(x_stepper,y_stepper,circular_path[i][0],circular_path[i][1],circular_path[i+1][0],circular_path[i+1][1],DIR_CCW);
//		rt_sem_take(sem_test,RT_WAITING_FOREVER);				/* waiting output finish */
//		rt_thread_mdelay(1000);									
//	}
//	/* clock-wise */
//	for(i = 8; i > 1; i++)
//	{
//		test_stepper_circular_interp(x_stepper,y_stepper,circular_path[i][0],circular_path[i][1],circular_path[i+1][0],circular_path[i+1][1],DIR_CW);
//		rt_sem_take(sem_test,RT_WAITING_FOREVER);				/* waiting output finish */
//		rt_thread_mdelay(1000);									
//	}
//	
//	/* trapzoid accelerate */
//	test_stepper_trapzoid(x_stepper,1000,0.002,0.006,0.4);
//	rt_sem_take(sem_test,RT_WAITING_FOREVER);	/* wait accel finish */
//	/* run 1000 step! reversal speed */
//	test_stepper_trapzoid(x_stepper,-1000,0.002,0.006,0.4);
//	rt_sem_take(sem_test,RT_WAITING_FOREVER);	/* wait accel finish */
//	
//	/* s curve acclerate */
//	test_stepper_s_curve(x_stepper,1000,300,20,2);	/* for stepper, 300Hz is relatively big */
//	/* the smaller flexible, the closer to const accel */
//	rt_sem_take(sem_test,RT_WAITING_FOREVER);		/* wait accel finish */
//	rt_thread_mdelay(2000);							/* delay 2000ms, then rotates reversally */
//	test_stepper_s_curve(x_stepper,-1000,300,20,2);/* for stepper, 300Hz is relatively big */
//}

//static void stepper_test(void)
//{
//	struct lt_stepper_config config;
//	rt_base_t x_pin = rt_pin_get("PF.8"), y_pin = rt_pin_get("PE.11");
//	rt_base_t x_enable = rt_pin_get("PF.7"), y_enable = rt_pin_get("PF.13");
// 	lt_driver_t x_driver, y_driver;
//	lt_timer_t x_timer, y_timer;
//	/* create steppers and drivers */
//	x_stepper = lt_motor_create("x_stepper",1,MOTOR_TYPE_STEPPER);
//	y_stepper = lt_motor_create("y_stepper",1,MOTOR_TYPE_STEPPER);
//	x_driver = lt_driver_create(DRIVER_TYPE_STEPPER);
//	y_driver = lt_driver_create(DRIVER_TYPE_STEPPER);
//	x_encoder = lt_sensor_create(SENSOR_NUM_ENCODER_1,4*600,SENSOR_TYPE_ENCODER);
//	x_timer = lt_timer_create("x_timer",TIMER_NUM_1,0);
//	y_timer = lt_timer_create("y_timer",TIMER_NUM_2,0);
//	if(x_stepper == RT_NULL)  return;
//	if(y_stepper == RT_NULL)  return;
//	if(x_driver == RT_NULL)  return;
//	if(y_driver == RT_NULL)  return;
//	if(x_encoder == RT_NULL) return;
//	if(x_timer == RT_NULL)  return;
//	if(y_timer == RT_NULL) return;
//	/* set stepper drivers! */
//	lt_driver_set_pins(x_driver,x_pin,0,x_enable);
//	lt_driver_set_pins(y_driver,y_pin,0,y_enable);
//	lt_driver_set_pwm(x_driver,PWM_NUM,OUTPUT_CHANNEL-1,0);
//	lt_driver_set_pwm(y_driver,PWM_NUM,OUTPUT_CHANNEL,0);
//	lt_motor_set_driver(x_stepper,x_driver);
//	lt_motor_set_driver(y_stepper,y_driver);
//	lt_motor_set_callback(x_stepper,done_callback);
//	lt_motor_set_timer(x_stepper,x_timer);
//	lt_motor_set_timer(y_stepper,y_timer);
//	//lt_motor_set_callback(y_stepper,done_callback);
//	/* set stepper position sensor */
//	//lt_sensor_set_pins(x_encoder,0,0,0);
//	lt_motor_set_sensor(x_stepper,x_encoder);
//	/* config stepper params */
//	config.period = STEPPER_PERIOD;
//	config.stepper_angle = STEPPER_ANGLE;
//	config.subdivide = STEPPER_SUBDIVIDE;
//	
//	lt_motor_control(x_stepper,STEPPER_CTRL_CONFIG,&config);
//	lt_motor_control(y_stepper,STEPPER_CTRL_CONFIG,&config);
//	/* config two steppers successfully */
//	sem_test = rt_sem_create("sem_test",0,RT_IPC_FLAG_PRIO);		/* create semaphore! */
//	if(sem_test == RT_NULL) return;
//	
//	stepper_thread = rt_thread_create("stepper",stepper_thread_entry,RT_NULL,2048,10,20);
//	
//	if(stepper_thread != RT_NULL)
//	{
//		rt_thread_startup(stepper_thread);
//	}
//}

/****************************************************************************************/
#ifdef LT_USING_MOTOR_MSH_TEST
/* test functions for convenience and as examples */
void test_motor_output(lt_motor_t motor,float input)
{
	if(motor == RT_NULL) return;
	lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&input);
	rt_kprintf("motor output: %.2f \n",input);
}

void test_motor_output_angle(lt_motor_t motor, float input)
{
	if(motor == RT_NULL) return;
	lt_motor_control(motor,MOTOR_CTRL_OUTPUT_ANGLE,&input);
	rt_kprintf("motor output angle %.2f degree \n",input);
}

void test_motor_output_pid(lt_motor_t motor, float input)
{
	if(motor == RT_NULL) return;
	lt_motor_control(motor,MOTOR_CTRL_OUTPUT_PID,&input);
	rt_kprintf("motor output pid : %.2f rpm \n",input);
}

void test_motor_output_angle_pid(lt_motor_t motor, float input)
{
	if(motor == RT_NULL) return;
	lt_motor_control(motor,MOTOR_CTRL_OUTPUT_ANGLE_PID,&input);
	rt_kprintf("motor output angle pid : %.2f degree \n",input);
}

void test_motor_get_position(lt_motor_t motor)
{
	if(motor == RT_NULL) return;
	float pos = lt_motor_get_position(motor);
	rt_kprintf("motor position: %.2f rad, %.2 degree \n",pos,pos*180.0f/PI);
}

void test_motor_get_velocity(lt_motor_t motor)
{
	if(motor == RT_NULL) return;
	rt_tick_t tick = rt_tick_get();
	float vel;
	lt_motor_get_velocity(motor,0);			/* refresh sensor info */
	rt_thread_delay(100);					/* delay 100ms */
	tick = rt_tick_get() - tick;			/* get measure time, unit: ms */
	vel = lt_motor_get_velocity(motor,tick);
	rt_kprintf("motor velocity: %.2f rad/s, %.2 rpm \n",vel,vel*9.55);
}

void test_stepper_trapzoid(lt_motor_t motor, int step,float acc, float dec,float speed)
{
	if(motor == RT_NULL) return;	
	static struct lt_curve_config config; 		/* use static variable to avoid being distroyed */
	config.acc = acc;
	config.dec = dec;
	config.target = speed;
	config.step = step;
	config.type = CURVE_TYPE_TRAPZOID;
	lt_motor_control(motor,STEPPER_CTRL_ACCELERATE,&config);
	rt_kprintf("stepper trapzoid accel ==> step:%d,  accel:%.2f Hz/ms,  decel:%.2f Hz/ms,  speed:%.2f Hz \n",step,acc,dec,speed);
}

void test_stepper_s_curve(lt_motor_t motor, int step, float acc_t,float freq_max, float freq_min, float flexible)
{
	if(motor == RT_NULL) return;	
	static struct lt_curve_config config; 		/* use static variable to avoid being distroyed */
	config.acc = acc_t;
	config.target = freq_max;
	config.initial = freq_min;
	config.flexible = flexible;
	config.step = step;
	config.type = CURVE_TYPE_S_CURVE;
	lt_motor_control(motor,STEPPER_CTRL_ACCELERATE,&config);
	rt_kprintf("stepper s_curve accel ==> step:%d,  acc_t:%.2f ms, freq_max:%.2f Hz,  freq_min:%.2f Hz,  flexible:%.2f \n",step,acc_t,freq_max,freq_min,flexible);
}

void test_stepper_5_section(lt_motor_t motor,int step,float acc_t, float speed)
{
	if(motor == RT_NULL) return;	
	static struct lt_curve_config config; 		/* use static variable to avoid being distroyed */
	config.acc = acc_t;
	config.target = speed;
	config.step = step;
	config.type = CURVE_TYPE_5_SECTION;
	lt_motor_control(motor,STEPPER_CTRL_ACCELERATE,&config);
	rt_kprintf("stepper 5_section accel ==> step:%d,  acc_t:%.2f ms, speed:%.2f Hz \n",step,acc_t,speed);
}

void test_stepper_line_interp(lt_motor_t x_motor, lt_motor_t y_motor, int x_start, int y_start, int x_end, int y_end)
{
	if(x_motor == RT_NULL) return;	
	if(y_motor == RT_NULL) return;	
	static struct lt_interp_config config;		/* use static variable to avoid being distroyed */
	config.x_start = x_start;
	config.y_start = y_start;
	config.x_end = x_end;
	config.y_end = y_end;
	config.radius = 0;							/* means line interp */
	config.y_motor = y_motor;
	lt_motor_control(x_motor,STEPPER_CTRL_INTERPOLATION,&config);
	rt_kprintf("stepper line interp ==> start:(%d, %d), end:(%d, %d) \n",x_start, y_start);
}

void test_stepper_circular_interp(lt_motor_t x_motor, lt_motor_t y_motor, int x_start, int y_start, int x_end, int y_end,rt_uint16_t radius, rt_uint8_t dir)
{
	if(x_motor == RT_NULL) return;	
	if(y_motor == RT_NULL) return;	
	static struct lt_interp_config config;		/* use static variable to avoid being distroyed */
	char* direction;
	config.x_start = x_start;
	config.y_start = y_start;
	config.x_end = x_end;
	config.y_end = y_end;
	config.dir = dir;
	config.radius = radius;
	
	if(dir == DIR_CW)	direction = "CW";
	else				direction = "CCW";
	lt_motor_control(x_motor,STEPPER_CTRL_INTERPOLATION,&config);
	rt_kprintf("stepper circular interp ==> start:(%d, %d), end:(%d, %d), radius:%d	dir:%s \n",x_start,y_start,x_end,y_end,radius,direction);
}
/****************************************************************************************/
#endif


