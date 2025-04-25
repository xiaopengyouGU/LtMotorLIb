#include "ltmotorlib.h"
#include "protocol.h"
#include "stdlib.h"

/* when communicating with upper computer, FINSH is forbidden */
/* basic command, eg: MOTOR_CTRL_OUTPUT	works on all motor
*  in fact, MOTOR_CTRL_OUTPUT = STEPPER_CTRL_OUPUT = BLDC_CTRL_OUTPUT
*/
/*************************************  setup table  **************************************************/
//#define TEST_MOTOR_OUTPUT_ANGLE	
//#define MOTOR_TYPE						MOTOR_TYPE_DC
#define MOTOR_TYPE							MOTOR_TYPE_STEPPER
//#define MOTOR_TYPE						MOTOR_TYPE_BLDC
#define MOTOR_NAME						"motor1"
#define TIMER_NAME						"timer_m"
#define STEPPER_TEST					
//#define BLDC_TEST						
/* PF6 is bad */
/* basic config param */
#define FORWARD_PIN						rt_pin_get("PF.8")
#define REVERSAL_PIN					rt_pin_get("PF.7")		/* in stepper case, FORWARD_PIN equals to REVERSAL_PIN, only handle FORWARD_PIN */
#define OUTPUT_CHANNEL					PWM_CHANNEL_4
#define PWM_NUM							PWM_NUM_1
#define ENCODER_NUM						PULSE_ENCODER_NUM_1
#define PIN_A							0
#define PIN_B							0
#define PIN_C							0
/* stepper's specific config param */
#define ENABLE_PIN						rt_pin_get("PF.7")		
#define STEPPER_PERIOD					10					/* unit: ms */
#define STEPPER_ANGLE					1.8
#define STEPPER_SUBDIVIDE				2
#define STEPPER_TIMER_NUM				TIMER_NUM_1			/* timer 1-5*/
#define STEPPER_TIMER_FREQ				1000000
	
#define TARGET_SPEED					120
#define TARGET_POSITION 				150
#define REDUCTION_RATIO					1					/* DC motor: 34 ; stepper motor: 1*/
#define ENCODER_TOTAL_RESOLUTION		4*600 				/*DC motor: 4*13; stepper motor: 4*600 */

#define SAMPLE_TIME						100					/* unit: ms, 20 for stepper motor */
#define KP								2.1
#define KI								0.1
#define KD								0.5

#ifdef RT_USING_FINSH 
#ifndef TEST_MOTOR_OUTPUT_ANGLE
#define TIMEOUT_FUNC					timeout_func
#endif
#else
//#define TIMEOUT_FUNC					pid_test		/* software timer callback function */
//#define TIMEOUT_FUNC					velocity_loop				
//#define TIMEOUT_FUNC					position_loop
#define TIMEOUT_FUNC					position_velocity_loop
#endif
/*************************************  setup table  **************************************************/
/* ADC --> PA6
*  DAC --> PA4
*  DAC --> PA5
*  PWM --> PA3
*  Encoder --> PB6 ， PB7
*  stepper motor driver is common-cathode
*/

rt_align(RT_ALIGN_SIZE)	

static lt_motor_t motor;
static lt_driver_t driver;
static lt_sensor_t sensor;
static rt_timer_t motor_timer;


static rt_thread_t process_t;
static lt_pid_t motor_pid;		/* motor pid */
static lt_pid_t pos_pid;		/* position pid */
static lt_pid_t vel_pid;		/* velocity pid */

static void pid_test(void *parameter)
{
	/* eg: KP = 0.26, KI = 0.9, KD = 0.02 */
	float read_val = lt_pid_get_control(motor_pid);
	float curr_val = lt_pid_control(motor_pid,read_val);
	int temp = curr_val;									/* transform data */
	set_computer_value(SEND_FACT_CMD,CURVES_CH1,&temp,1);	/* send fact value to upper computer */
}

static void velocity_loop(void *parameter)
{
	/* eg: KP = 1.96, KI = 1.6, KD = 0.01 */
	float vel = lt_motor_get_velocity(motor,SAMPLE_TIME) * 9.55;	/* get rpm */
	float control_u = 0.1f * lt_pid_control(motor_pid,vel);		/* multiply a coefficiency */
	int t_vel = vel, t_control_u = control_u;							
	/* transform control force to duty circle of PWM output */
	lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&control_u);			/* output */
	/* send data to upper computer */
	set_computer_value(SEND_FACT_CMD,CURVES_CH1,&t_vel,1);
	set_computer_value(SEND_FACT_CMD,CURVES_CH2,&t_control_u,1);
}

static void position_loop(void *parameter)
{
	/* eg: KP = 10.1, KI = 23, KD = 0.01 */
	float position = lt_motor_get_position(motor);				/* get motor angle */
	float control_u = 0.01f * lt_pid_control(motor_pid,position);
	lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&control_u);
	int t_position = position, t_control_u = control_u * 100;	/* let control curve smoother */	
	/* send data to upper computer */
	set_computer_value(SEND_FACT_CMD,CURVES_CH1,&t_position,1);
	set_computer_value(SEND_FACT_CMD,CURVES_CH2,&t_control_u,1);
}

static void position_velocity_loop(void *parameter)
{
	/* we adjust vel_pid first */
	motor_pid = pos_pid;	/* DC motor: Kp = 3.9, Ki = 1.5, Kd = 1.05 */
	//motor_pid = vel_pid; 	/* DC motor: Kp = 1.8, Ki = 2, Kd = 0.3 */
	static rt_uint32_t count = 0;
	/* process position loop first, sample time 3T */
	if(count % 3 == 0)
	{
		float position = lt_motor_get_position(motor)*180.0f/PI;	/* unit: degree */
		float desired_vel = lt_pid_control(pos_pid,position);
		lt_pid_set_target(vel_pid,desired_vel);
		int t_position = position;
		set_computer_value(SEND_FACT_CMD,CURVES_CH1,&t_position,1);
	}
	/* velocity loop follow, sample time T */
	float vel = lt_motor_get_velocity(motor,SAMPLE_TIME)*9.55;	/* get rpm */
	float control_u = 0.01f * lt_pid_control(vel_pid,vel);
	lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&control_u);		/* output! */
	int t_speed = vel*100, t_control_u = control_u * 100;		/* let control and speed curves smoother */
	
	set_computer_value(SEND_FACT_CMD,CURVES_CH2,&t_speed,1);
	set_computer_value(SEND_FACT_CMD,CURVES_CH3,&t_control_u,1);
	count++;
	
}
	
static void process_thread_entry(void* parameter)
{
  rt_uint8_t frame_data[128];         // 要能放下最长的帧
  rt_uint16_t frame_len = 0;          // 帧长度
  rt_uint8_t cmd_type = CMD_NONE;     // 命令类型
  float speed = 0;					  /* stop motor */
  while(1)
  {
    cmd_type = protocol_frame_parse(frame_data, &frame_len);
    switch (cmd_type)
    {
      case SET_PID_CMD:
      {
        rt_uint32_t temp0 = COMPOUND_32BIT(&frame_data[13]);
        rt_uint32_t temp1 = COMPOUND_32BIT(&frame_data[17]);
        rt_uint32_t temp2 = COMPOUND_32BIT(&frame_data[21]);
        
        float p_temp, i_temp, d_temp;
        
        p_temp = *(float *)&temp0;
        i_temp = *(float *)&temp1;
        d_temp = *(float *)&temp2;
        
		lt_pid_set(motor_pid,p_temp, i_temp, d_temp);    /* set pid */
      }
      break;

      case SET_TARGET_CMD:
      {
		int actual_temp = COMPOUND_32BIT(&frame_data[13]);    	// 得到数据
		lt_pid_set_target(motor_pid,actual_temp);    			// 设置目标值
      }
      break;
      
      case START_CMD:
      {
		rt_timer_start(motor_timer);// 启动电机
      }
      break;
      
      case STOP_CMD:
      {
        rt_timer_stop(motor_timer);              // 停止电机
		lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&speed);
		set_computer_value(SEND_STOP_CMD,CURVES_CH1,RT_NULL,0);
      }
      break;
      
      case RESET_CMD:
      {
        rt_timer_stop(motor_timer);              /* stop timer */
		lt_pid_reset(motor_pid);				/* reset pid! */
		lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&speed);
      }
      break;
      
      case SET_PERIOD_CMD:
      {
        rt_uint32_t temp = COMPOUND_32BIT(&frame_data[13]);     	// period
		rt_timer_control(motor_timer,RT_TIMER_CTRL_SET_TIME,&temp); // set timer period 1~1000ms
      }
      break;
    }
	rt_thread_mdelay(100);
  }
}
#endif

/* motor test frame */
void lt_motor_test(void)
{
	/* create motor, driver, sensor */
	motor = lt_motor_create(MOTOR_NAME,REDUCTION_RATIO,MOTOR_TYPE);
	driver = lt_driver_create(DRIVER_TYPE_DC);
	sensor = lt_sensor_create(SENSOR_NUM_ENCODER_1,ENCODER_TOTAL_RESOLUTION,SENSOR_TYPE_ENCODER);
	
	if(motor == RT_NULL) return;
	if(driver == RT_NULL) return;
	if(sensor == RT_NULL) return;
	/* config driver */
	lt_driver_set_pwm(driver,PWM_NUM,OUTPUT_CHANNEL,0);					/* set pwm and channel */
	lt_driver_set_pins(driver,FORWARD_PIN,REVERSAL_PIN,ENABLE_PIN);		/* set pwm and channel */
	//lt_driver_set_output(driver,1000,0.5,0);							/* set pwm period and duty_cycle, unit: us */
	/* config position sensor */
	lt_sensor_set_pins(sensor,PIN_A,PIN_B,PIN_C);
	/* config motor */
	lt_motor_set_driver(motor,driver);
	lt_motor_set_sensor(motor,sensor);
	lt_motor_set_callback(motor,RT_NULL);								/* when finished a output, motor object will call callback function */
#ifdef STEPPER_TEST														/* config a stepper motor */
	struct lt_stepper_config config;
	config.period = STEPPER_PERIOD;										/* unit: 10ms */
	config.stepper_angle = STEPPER_ANGLE;								/* unit: degree */
	config.subdivide = STEPPER_SUBDIVIDE;	
	config.timer_num = STEPPER_TIMER_NUM;								/* hardware timer number */
	//config.timer_freq = STEPPER_TIMER_FREQ;		 					/* default: 1Mhz */
	lt_motor_control(motor,STEPPER_CTRL_CONFIG,&config);				/* config stepper */		
#endif	
	

/* create software timer */	
#ifndef TEST_MOTOR_OUTPUT_ANGLE
	//motor_timer = rt_timer_create(TIMER_NAME,TIMEOUT_FUNC,RT_NULL,SAMPLE_TIME,RT_TIMER_FLAG_PERIODIC);
	if(motor_timer == RT_NULL) return;
#endif
	
#ifdef RT_USING_FINSH
	
#else
	motor_pid = lt_pid_create(KP,KI,KD,SAMPLE_TIME);	/* init pid parameter, start from little parameter! */
	vel_pid = lt_pid_create(1.8,2,0.3,SAMPLE_TIME);		/* creata velocity pid and position pid*/
	pos_pid = lt_pid_create(3.9,1.5,1.05,SAMPLE_TIME*3);
	lt_pid_set_target(motor_pid,TARGET_SPEED);			/* set target speed */
	lt_pid_set_target(pos_pid,TARGET_POSITION);			
	if(motor_pid == RT_NULL) return;
	
	protocol_init();							/* init protocol to communicate with computer */
	process_t = rt_thread_create("process",		/* create a process thread */
								process_thread_entry,
								RT_NULL,
								512,
								9,				
								20);
	if(process_t != RT_NULL)
	{
		rt_thread_startup(process_t);			/* start thread */
	}
#endif
}

MSH_CMD_EXPORT(lt_motor_test, motor test);

/* stepper for interpolation */
static lt_motor_t x_stepper;
static lt_motor_t y_stepper;
static lt_driver_t x_driver;
static lt_driver_t y_driver; 
static lt_sensor_t x_encoder;
//static lt_sensor_t y_encoder;
static rt_sem_t sem_test;				/* semaphore for test */
static rt_thread_t stepper_thread;		/* stepper thread */
/* when finish output, this function would be called */
rt_err_t done_callback(void *parameter)
{
	return rt_sem_release(sem_test);
}

static void stepper_thread_entry(void *parameter)
{
	rt_uint8_t i;
	float output = 360, res;
	for (i = 0; i < 5; i++)			/* 5 forward rotations */
	{
		lt_motor_control(x_stepper,STEPPER_CTRL_OUTPUT_ANGLE,&output);
		rt_sem_take(sem_test,RT_WAITING_FOREVER);				/* waiting output finish */
		rt_thread_mdelay(500);									/* delay */
	}
	output = -360;
	for (i = 0; i < 5; i++)			/* 5 reversal rotations */
	{
		lt_motor_control(x_stepper,STEPPER_CTRL_OUTPUT_ANGLE,&output);
		rt_sem_take(sem_test,RT_WAITING_FOREVER);				/* waiting output finish */
		rt_thread_mdelay(500);									/* delay */
	}
	/* we only set x_stepper position sensor */
	test_motor_get_position(x_stepper);
	test_motor_get_velocity(x_stepper);
	
	/* line interpolation path: /\ -->  ---
								\/     |   |
										---	
	*/
	rt_int32_t line_path[8][2] = {{500,800},{500,-800},{-500,-800},{-500,800},
								  {0,1000},{1000,0},{0,-1000},{-1000,0}};
	rt_int32_t circular_path[9][2] = {{707,0},{500,500},{0,707},{-500,500},{-707,0},{-500,-500},{0,-707},{500,-500},{0,707}};
	/* draw a circle with radius = 707 steps */
	/* line interp test */							  
	for(i = 0; i < 8; i++)
	{
		test_stepper_line_interp(x_stepper,y_stepper,line_path[i][0],line_path[i][1]);
		rt_sem_take(sem_test,RT_WAITING_FOREVER);				/* waiting output finish */
		rt_thread_mdelay(1000);									
	}
	/* circular interp test */
	/* in this case, the circular should lie in one quadrant and interp direction should be reasonable 
	*  eg: start:(50, 50), end:(71,0), direction should be CW since intep circular lies in 1th quadrant
	*/
	for(i = 0; i < 8; i++)
	{
		test_stepper_circular_interp(x_stepper,y_stepper,circular_path[i][0],circular_path[i][1],circular_path[i+1][0],circular_path[i+1][1],DIR_CCW);
		rt_sem_take(sem_test,RT_WAITING_FOREVER);				/* waiting output finish */
		rt_thread_mdelay(1000);									
	}
	/* clock-wise */
	for(i = 8; i > 1; i++)
	{
		test_stepper_circular_interp(x_stepper,y_stepper,circular_path[i][0],circular_path[i][1],circular_path[i+1][0],circular_path[i+1][1],DIR_CW);
		rt_sem_take(sem_test,RT_WAITING_FOREVER);				/* waiting output finish */
		rt_thread_mdelay(1000);									
	}
	
	/* trapzoid accelerate */
	test_stepper_trapzoid(x_stepper,1000,0.002,0.006,0.4);
	rt_sem_take(sem_test,RT_WAITING_FOREVER);	/* wait accel finish */
	/* run 1000 step! reversal speed */
	test_stepper_trapzoid(x_stepper,-1000,0.002,0.006,0.4);
	rt_sem_take(sem_test,RT_WAITING_FOREVER);	/* wait accel finish */
	
	/* s curve acclerate */
	test_stepper_s_curve(x_stepper,1000,300,20,2);	/* for stepper, 300Hz is relatively big */
	/* the smaller flexible, the closer to const accel */
	rt_sem_take(sem_test,RT_WAITING_FOREVER);		/* wait accel finish */
	rt_thread_mdelay(2000);							/* delay 2000ms, then rotates reversally */
	test_stepper_s_curve(x_stepper,-1000,300,20,2);/* for stepper, 300Hz is relatively big */
}

static void stepper_test(void)
{
	struct lt_stepper_config config;
	rt_base_t x_pin = rt_pin_get("PF.8"), y_pin = rt_pin_get("PF.11");
	rt_base_t x_enable = rt_pin_get("PF.7"), y_enable = rt_pin_get("PF.13");
 	lt_driver_t x_driver, y_driver;
	/* create steppers and drivers */
	x_stepper = lt_motor_create("x_stepper",1,MOTOR_TYPE_STEPPER);
	y_stepper = lt_motor_create("y_stepper",1,MOTOR_TYPE_STEPPER);
	x_driver = lt_driver_create(DRIVER_TYPE_STEPPER);
	y_driver = lt_driver_create(DRIVER_TYPE_STEPPER);
	x_encoder = lt_sensor_create(SENSOR_NUM_ENCODER_1,4*600,SENSOR_TYPE_ENCODER);
	if(x_stepper == RT_NULL)  return;
	if(y_stepper == RT_NULL)  return;
	if(x_driver == RT_NULL)  return;
	if(y_driver == RT_NULL)  return;
	if(x_encoder == RT_NULL) return;
	/* set stepper drivers! */
	lt_driver_set_pins(x_driver,x_pin,0,x_enable);
	lt_driver_set_pins(y_driver,y_pin,0,y_enable);
	lt_driver_set_pwm(x_driver,PWM_NUM,OUTPUT_CHANNEL-1,0);
	lt_driver_set_pwm(y_driver,PWM_NUM,OUTPUT_CHANNEL,0);
	lt_motor_set_driver(x_stepper,x_driver);
	lt_motor_set_driver(y_stepper,y_driver);
	lt_motor_set_callback(x_stepper,done_callback);
	//lt_motor_set_callback(y_stepper,done_callback);
	/* set stepper position sensor */
	//lt_sensor_set_pins(x_encoder,0,0,0);
	lt_motor_set_sensor(x_stepper,x_encoder);
	/* config stepper params */
	config.period = STEPPER_PERIOD;
	config.stepper_angle = STEPPER_ANGLE;
	config.subdivide = STEPPER_SUBDIVIDE;
	//config.timer_freq = STEPPER_TIMER_FREQ;		default: 1Mhz
	config.timer_num = STEPPER_TIMER_NUM;
	
	lt_motor_control(x_stepper,STEPPER_CTRL_CONFIG,&config);
	lt_motor_control(y_stepper,STEPPER_CTRL_CONFIG,&config);
	/* config two steppers successfully */
	sem_test = rt_sem_create("sem_test",0,RT_IPC_FLAG_PRIO);		/* create semaphore! */
	if(sem_test == RT_NULL) return;
	
	stepper_thread = rt_thread_create("stepper",stepper_thread_entry,RT_NULL,2048,10,20);
	
	if(stepper_thread != RT_NULL)
	{
		rt_thread_startup(stepper_thread);
	}
}

/****************************************************************************************/
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
	static struct lt_stepper_config_accel config; 		/* use static variable to avoid being distroyed */
	config.accel = acc;
	config.decel = dec;
	config.speed = speed;
	config.step = step;
	lt_motor_control(motor,STEPPER_CTRL_TRAPZOID_ACCELERATE,&config);
	rt_kprintf("stepper trapzoid accel ==> step:%d,  accel:%.2f rad/s^2,  decel:%.2f rad/s^2,  speed:%.2f rad/s \n",step,acc,dec,speed);
}

void test_stepper_s_curve(lt_motor_t motor, int step, float freq_max, float freq_min, float flexible)
{
	if(motor == RT_NULL) return;	
	static struct lt_stepper_config_accel config; 		/* use static variable to avoid being distroyed */
	config.freq_max = freq_max;
	config.freq_min = freq_min;
	config.flexible = flexible;
	config.step = step;
	lt_motor_control(motor,STEPPER_CTRL_S_CURVE_ACCELERATE,&config);
	rt_kprintf("stepper s_curve accel ==> step:%d,  freq_max:%.2f Hz,  freq_min:%.2f Hz,  flexible:%.2f \n",step,freq_max,freq_min,flexible);
}

void test_stepper_line_interp(lt_motor_t x_motor, lt_motor_t y_motor, int x_pos, int y_pos)
{
	if(x_motor == RT_NULL) return;	
	if(y_motor == RT_NULL) return;	
	static struct lt_stepper_config_interp config;		/* use static variable to avoid being distroyed */
	config.x_end = x_pos;
	config.y_end = y_pos;
	config.x_stepper = x_motor;
	config.y_stepper = y_motor;
	
	lt_motor_control(x_motor,STEPPER_CTRL_LINE_INTERPOLATION,&config);			/* call x_motor's callback function if finished */
	rt_kprintf("stepper line interp ==> x_pos:%d,  y_pos:%d \n",x_pos, y_pos);
}

void test_stepper_circular_interp(lt_motor_t x_motor, lt_motor_t y_motor, int x_start, int y_start, int x_end, int y_end, rt_uint8_t dir)
{
	if(x_motor == RT_NULL) return;	
	if(y_motor == RT_NULL) return;	
	static struct lt_stepper_config_interp config;		/* use static variable to avoid being distroyed */
	char* direction;
	config.x_start = x_start;
	config.y_start = y_start;
	config.x_end = x_end;
	config.y_end = y_end;
	config.dir = dir;
	config.x_stepper = x_motor;
	config.y_stepper = y_motor;
	
	if(dir == DIR_CW)	direction = "CW";
	else				direction = "CCW";
	
	lt_motor_control(x_motor,STEPPER_CTRL_CIRCULAR_INTERPOLATION,&config);
	rt_kprintf("stepper circular interp ==> x_start:%d,  y_start:%d,	x_end:%d,  y_end:%d,	dir:%s \n",x_start,y_start,x_end,y_end,direction);
}
/****************************************************************************************/



