#include "lt_motor_control.h"
#include "protocol.h"
#include "key.h"
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
#define FORWARD_PIN						GET_PIN(F,8)
#define REVERSAL_PIN					GET_PIN(F,7)		/* in stepper case, FORWARD_PIN equals to REVERSAL_PIN, only handle FORWARD_PIN */
#define OUTPUT_CHANNEL					PWM_CHANNEL_4
#define PWM_NUM							PWM_NUM_1
#define ENCODER_NUM						PULSE_ENCODER_NUM_1
/* stepper's specific config param */
#define ENABLE_PIN						GET_PIN(F,7)		
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
static rt_timer_t motor_timer;


#ifdef RT_USING_FINSH
/* test event */ 
#define EVENT_KEY0	(1 << 3)
#define EVENT_KEY_UP (1 << 5)
static rt_thread_t motor_t;
static rt_thread_t key_thread;
static rt_event_t key_event;

static void key_thread_entry(void *parameter)
{
	rt_uint8_t key_val;
	
	while(1)
	{
		key_val = key_scan(0);
		if(key_val == KEY0_PRES)
		{
			rt_kprintf("key thread: KEY0 pres\n");
			rt_event_send(key_event,EVENT_KEY0);
		}
		if(key_val == KEY_UP_PRES)
		{
			rt_kprintf("key thread: KEY_UP pres\n");
			rt_event_send(key_event,EVENT_KEY_UP);
		}
		rt_thread_mdelay(10);	//延时
	}
}


static void motor_thread_entry(void *parameter)
{
	float output = 0;
	rt_uint32_t event;
	while(1)
	{
		rt_event_recv(key_event,(EVENT_KEY0 | EVENT_KEY_UP),
								RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
								RT_WAITING_FOREVER, &event);
		if(event == EVENT_KEY0)
		{
			output -= 20;
		}
		else if(event == EVENT_KEY_UP)
		{
			output += 20;
		}
		rt_kprintf("event: %d, output: %.1f \n",event,output);
#ifdef TEST_MOTOR_OUTPUT_ANGLE
		lt_motor_control(motor,MOTOR_CTRL_OUTPUT_ANGLE,&output);
#else
		lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&output);
#endif
		
	}
}
#ifndef TEST_MOTOR_OUTPUT_ANGLE
static void timeout_func(void *parameter)
{
	float speed = lt_motor_measure_speed(motor,SAMPLE_TIME);	/* measure time: 100ms */
	rt_kprintf("motor speed: %.2f r/s , %.2f rpm \n ",speed,speed* 60);	/* finsh must enable float output */
	if(speed == 0)
	{
		lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&speed);		/* avoid locking rotor */
		return;
	}
}
#endif

#else 
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
	float speed = lt_motor_measure_speed(motor,SAMPLE_TIME) * 60;	/* get rpm */
	float control_u = 0.1f * lt_pid_control(motor_pid,speed);		/* multiply a coefficiency */
	int t_speed = speed, t_control_u = control_u;							
	/* transform control force to duty circle of PWM output */
	lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&control_u);			/* output */
	/* send data to upper computer */
	set_computer_value(SEND_FACT_CMD,CURVES_CH1,&t_speed,1);
	set_computer_value(SEND_FACT_CMD,CURVES_CH2,&t_control_u,1);
}

static void position_loop(void *parameter)
{
	/* eg: KP = 10.1, KI = 23, KD = 0.01 */
	float position = lt_motor_measure_position(motor);			/* get motor angle */
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
		float position = lt_motor_measure_position(motor);
		float desired_speed = lt_pid_control(pos_pid,position);
		lt_pid_set_target(vel_pid,desired_speed);
		int t_position = position;
		set_computer_value(SEND_FACT_CMD,CURVES_CH1,&t_position,1);
	}
	/* velocity loop follow, sample time T */
	float speed = lt_motor_measure_speed(motor,SAMPLE_TIME);
	float control_u = 0.01f * lt_pid_control(vel_pid,speed);
	lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&control_u);		/* output! */
	int t_speed = speed*100, t_control_u = control_u * 100;		/* let control and speed curves smoother */
	
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
	/* create a motor */
	motor = lt_motor_create(MOTOR_NAME,REDUCTION_RATIO,MOTOR_TYPE);
	lt_motor_set_pwm(motor,PWM_NUM,OUTPUT_CHANNEL);						/* set pwm and channel */
	lt_motor_set_encoder(motor,ENCODER_NUM,ENCODER_TOTAL_RESOLUTION);	/* set pulse encoder */
	lt_motor_set_dir_pins(motor,FORWARD_PIN,REVERSAL_PIN);				/* in stepper case, we set FORWARD_PIN equal to REVERSAL_PIN */
	
	if(motor == RT_NULL) return;
#ifdef STEPPER_TEST														/* config a stepper motor */
	struct lt_stepper_config config;
	config.enable_pin = ENABLE_PIN;
	config.period = STEPPER_PERIOD;										/* unit: 10ms */
	config.stepper_angle = STEPPER_ANGLE;								/* unit: degree */
	config.subdivide = STEPPER_SUBDIVIDE;	
	config.timer_num = STEPPER_TIMER_NUM;								/* hardware timer number */
	//config.timer_freq = STEPPER_TIMER_FREQ;		 					/* default: 1Mhz */
	lt_motor_control(motor,STEPPER_CTRL_CONFIG,&config);				/* config stepper */		
#endif	
	

/* create software timer */	
#ifndef TEST_MOTOR_OUTPUT_ANGLE
	motor_timer = rt_timer_create(TIMER_NAME,TIMEOUT_FUNC,RT_NULL,SAMPLE_TIME,RT_TIMER_FLAG_PERIODIC);
	if(motor_timer == RT_NULL) return;
#endif
	
#ifdef RT_USING_FINSH
	key_init();											/* init key! */
	/* create a motor thread */
	motor_t = rt_thread_create("motor",
								motor_thread_entry,
								RT_NULL,
								1024,					/* thread stack size must big enough!!! */
								10,
								20);
		
	if(motor_t != RT_NULL)
	{
		rt_thread_startup(motor_t);	/* start thread */
	}
	/* create key event */
	key_event = rt_event_create("my_event",RT_IPC_FLAG_PRIO);
	if(key_event == RT_NULL)
	{
		rt_kprintf("create event failed!\n");
		return;
	}
	
	key_thread = rt_thread_create("key_thread",
								key_thread_entry,
								RT_NULL,
								512,
								5,
								20);
	if(key_thread != RT_NULL)
	{
		rt_thread_startup(key_thread);			/* start thread */
	}

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

#ifdef STEPPER_TEST
/* stepper for interpolation */
static lt_motor_t x_stepper;
static lt_motor_t y_stepper;
static rt_sem_t sem_test;		/* semaphore for test */
static rt_thread_t stepper_t;	

rt_err_t done_callback(void *parameter)
{
	rt_sem_release(sem_test);
}
/* trapzoid accelerate */
void stepper_trapzoid_accelerate(int step,float acc, float dec, float speed)
{
	struct lt_stepper_config_accel config;
	config.accel = acc;				/* unit: rad/s^2 */
	config.decel = dec;				/* unit: rad/s^2 */
	config.speed = speed;				/* unit: rad/s = 9.55rpm */
	config.step = step;				/* unit: step, >=0 : forward , <0 : reversal */
	/* memory must be enough for large amount of step */
	lt_motor_control(motor,STEPPER_CTRL_TRAPZOID_ACCELERATE,&config);
}

void stepper_s_curve_accelerate(int step,float freq_max, float freq_min,float flexible)
{
	struct lt_stepper_config_accel config;
	config.freq_max = freq_max;				/* unit: rad/s^2 */
	config.freq_min = freq_max;				/* unit: rad/s^2 */
	config.flexible = flexible;				/* unit: rad/s = 9.55rpm */
	config.step = step;				/* unit: step, >=0 : forward , <0 : reversal */
	/* memory must be enough for large amount of step */
	lt_motor_control(motor,STEPPER_CTRL_S_CURVE_ACCELERATE,&config);
}

/* line interpolation test */
void stepper_line_interp(int x_pos, int y_pos)
{
	struct lt_stepper_config_interp config;
	config.x_end = x_pos;
	config.y_end = y_pos;
	
}

void stepper_circular_interp(int x_start, int y_start, int x_end, int y_end)
{
	struct lt_stepper_config_interp config;
	config.x_start = x_start;
	config.y_start = y_start;
	config.x_end = x_end;
	config.y_end = y_end;
	
}

static void stepper_thread_entry(void *parameter)
{
	rt_uint8_t i;
	float output = 360;
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
	
	/* line interpolation path: /\ -->  ---
								\/     |   |
										---	
	*/
	rt_int32_t line_path[8][2] = {{500,800},{500,-800},{-500,-800},{-500,800},
								  {0,1000},{1000,0},{0,-1000},{-1000,0}};
	rt_int32_t circular_path[9][2] = {{707,0},{500,500},{0,707},{-500,500},{-707,0},{-500,-500},{0,-707},{500,-500},{0,707}};
	/* draw a circle with radius = 707 steps */
	struct lt_stepper_config_interp config;
	config.x_stepper = x_stepper;
	config.y_stepper = y_stepper;
	/* line interp test */							  
	for(i = 0; i < 8; i++)
	{
		config.x_end = line_path[i][0];
		config.y_end = line_path[i][1];
		lt_motor_control(x_stepper,STEPPER_CTRL_LINE_INTERPOLATION,&config);
		rt_sem_take(sem_test,RT_WAITING_FOREVER);				/* waiting output finish */
		rt_thread_mdelay(1000);									
	}
	/* circular interp test */
	/* in this case, the circular should lie in one quadrant and interp direction should be reasonable 
	*  eg: start:(50, 50), end:(71,0), direction should be CW since intep circular lies in 1th quadrant
	*/
	config.dir = STEPPER_INTERP_DIR_CCW;						/* counter clock-wise */
	for(i = 0; i < 8; i++)
	{
		config.x_start = circular_path[i][0];
		config.y_start = circular_path[i][1];
		config.x_end = circular_path[i+1][0];
		config.y_end = circular_path[i+1][1];
		lt_motor_control(x_stepper,STEPPER_CTRL_CIRCULAR_INTERPOLATION,&config);
		rt_sem_take(sem_test,RT_WAITING_FOREVER);				/* waiting output finish */
		rt_thread_mdelay(1000);									
	}
	config.dir = STEPPER_INTERP_DIR_CW;						    /* clock-wise */
	for(i = 8; i > 1; i++)
	{
		config.x_start = circular_path[i][0];
		config.y_start = circular_path[i][1];
		config.x_end = circular_path[i-1][0];
		config.y_end = circular_path[i-1][1];
		lt_motor_control(x_stepper,STEPPER_CTRL_CIRCULAR_INTERPOLATION,&config);
		rt_sem_take(sem_test,RT_WAITING_FOREVER);				/* waiting output finish */
		rt_thread_mdelay(1000);									
	}
	/* trapzoid accelerate */
	struct lt_stepper_config_accel config_acc;
	config_acc.accel = 0.002;				/* unit: rad/s^2 */
	config_acc.decel = 0.006;				/* unit: rad/s^2 */
	config_acc.speed = 0.4;					/* unit: rad/s = 9.55 rpm */
	config_acc.step = 1000;					/* run 1000 step! forward rotation */
	lt_motor_control(x_stepper,STEPPER_CTRL_TRAPZOID_ACCELERATE,&config_acc);
	rt_sem_take(sem_test,RT_WAITING_FOREVER);	/* wait accel finish */
	
	config_acc.step = -1000;				/* run 1000 step! reversal speed */
	lt_motor_control(x_stepper,STEPPER_CTRL_TRAPZOID_ACCELERATE,&config_acc);
	rt_sem_take(sem_test,RT_WAITING_FOREVER);	/* wait accel finish */
	/* s curve acclerate */
	config_acc.step = 1000;
	config_acc.freq_max = 300;					/* for stepper, 300Hz is relatively big */
	config_acc.freq_min = 20;
	config_acc.flexible = 2;					/* the smaller, the closer to const accel */
	lt_motor_control(x_stepper,STEPPER_CTRL_S_CURVE_ACCELERATE,&config_acc);
	rt_sem_take(sem_test,RT_WAITING_FOREVER);	/* wait accel finish */
	rt_thread_mdelay(2000);						/* delay 2000ms, then rotates reversally */
	config_acc.step = -1000;
	lt_motor_control(x_stepper,STEPPER_CTRL_S_CURVE_ACCELERATE,&config_acc);
}

static void stepper_config(void)
{
	struct lt_stepper_config config;
	rt_base_t x_pin = GET_PIN(F,8), y_pin = GET_PIN(F,11);
	rt_base_t x_enable = GET_PIN(F,7), y_enable = GET_PIN(F,13);
 	x_stepper = lt_motor_create("x_stepper",1,MOTOR_TYPE_STEPPER);
	y_stepper = lt_motor_create("y_stepper",1,MOTOR_TYPE_STEPPER);
	/* no need to set encoder */
	lt_motor_set_dir_pins(x_stepper,x_pin,x_pin);		/* set direction pin */
	lt_motor_set_dir_pins(y_stepper,y_pin,y_pin);
	/* config stepper params */
	config.enable_pin = x_enable;
	config.period = STEPPER_PERIOD;
	config.stepper_angle = STEPPER_ANGLE;
	config.subdivide = STEPPER_SUBDIVIDE;
	//config.timer_freq = STEPPER_TIMER_FREQ;		default: 1Mhz
	config.timer_num = STEPPER_TIMER_NUM;
	
	lt_motor_control(x_stepper,STEPPER_CTRL_CONFIG,&config);
	config.enable_pin = y_enable;
	lt_motor_control(y_stepper,STEPPER_CTRL_CONFIG,&config);
	/* config two steppers successfully */
	stepper_t = rt_thread_create("stepper")
}


/* use FINSH to help test functions */
#ifdef RT_USING_FINSH
#ifdef LT_USING_MOTOR_TEST
static void _test_info(int argc);
static void _test_basic(int argc,char*argv[]);
static void _test_stepper(int argc, char*argv[]);
//static void _test_bldc(int argc, char*argv[]);
static void motor_test(int argc, char*argv[])
{
	if(argc == 1 || argc == 2)
	{
		_test_info(argc);		/* show motor_test information */
	}
	else if(argc == 3 || argc == 4)
	{
		_test_basic(argc,argv);/* basic part of motor_test */
	}
	else
	{
#ifdef STEPPER_TEST
		if(!rt_strcmp(argv[1],"stepper"))
		{
			_test_stepper(argc,argv);	/* stepper part of motor_test */
		}
#endif
	}

}
MSH_CMD_EXPORT(motor_test, some motor tests examples );


/* show motor_test info */
static void _test_info(int argc)
{
	if(argc == 1)
	{
		rt_kprintf("LtMotorLib --> A motor control library based on RT_Thread RTOS!!!\n ");
		rt_kprintf("Author: LvTou, Date: 2025/4/5, Version: 0.1 \n");
		rt_kprintf("LtMotorLib tests support finsh, by which you would be familar with LtMotorLib quickly!\n ");
		rt_kprintf("Input like this 'motor_test dc/stepper/bldc cmd ' to call correspond functions \n");
		rt_kprintf("Many command can be called, see details by inputting 'motor_test help'\n");
		rt_kprintf("Hope you enjoy it!!! \n\n");
	}
	else if(argc == 2)
	{
		rt_kprintf("motor_test call formats: \n\n");
		rt_kprintf("******************* DC motor ************************\n");
		rt_kprintf("motor_test dc output val \n");
		rt_kprintf("motor_test dc output_angle val \n");
		rt_kprintf("motor_test dc get_angle \n");
		rt_kprintf("motor_test dc get_velocity \n");
		rt_kprintf("motor_test dc get_status \n");
		rt_kprintf("******************* DC motor ************************\n\n");
		rt_kprintf("******************* stepper motor ************************\n");
		rt_kprintf("motor_test stepper output val \n");
		rt_kprintf("motor_test stepper output_angle val \n");
		rt_kprintf("motor_test stepper get_angle \n");
		rt_kprintf("motor_test stepper get_velocity \n");
		rt_kprintf("motor_test stepper get_status \n");
		rt_kprintf("motor_test stepper trapzoid step acc dec speed \n");
		rt_kprintf("motor_test stepper s_curve step freq_max freq_min flexible \n");
		rt_kprintf("motor_test stepper line_interp x_pos, y_pos \n");
		rt_kprintf("motor_test stepper circular_interp x_start y_start  x_end y_end \n");
		rt_kprintf("******************* stepper motor ************************\n\n");
		rt_kprintf("******************* bldc motor ************************\n");
		rt_kprintf("motor_test bldc output val \n");
		rt_kprintf("motor_test bldc output_angle val \n");
		rt_kprintf("motor_test bldc get_angle \n");
		rt_kprintf("motor_test bldc get_velocity \n");
		rt_kprintf("motor_test bldc get_status \n");
		rt_kprintf("******************* bldc motor ************************\n\n");
	}
}
/* basic part of motor_test */
static void _test_basic(int argc,char *argv[])
{
	float res;
	rt_uint8_t status;
	if(!rt_strcmp(argv[1],"dc") || !rt_strcmp(argv[1],"stepper") ||!rt_strcmp(argv[1],"bldc"))
	{
		if(argc == 3)
		{
			if(!rt_strcmp(argv[2],"get_angle"))
			{
				res = lt_motor_measure_position(motor);
				rt_kprintf("motor angle : %.2f degree \n",res);
			}
			else if(!rt_strcmp(argv[2],"get_velocity"))
			{
				/* refresh encorder record first */
				res = lt_motor_measure_speed(motor,SAMPLE_TIME);
				rt_thread_mdelay(SAMPLE_TIME);
				res = lt_motor_measure_speed(motor,SAMPLE_TIME);
				rt_kprintf("motor speed: %.2f rad/s, %.2f rpm \n",res*2*PI,60*res);
			}
			else if(!rt_strcmp(argv[2],"get_status"))
			{
				lt_motor_control(motor,MOTOR_CTRL_GET_STATUS,&status);
				rt_kprintf("motor status: %s \n",_status[status]);
			}
		}
		else if(argc == 4)
		{
			res = (float)atof(argv[3]);
			if(!rt_strcmp(argv[2],"output"))
			{
				lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&res);
				rt_kprintf("motor output : %.2f \n",res);
			}
			else if(!rt_strcmp(argv[2],"output_angle"))
			{
				/* refresh encorder record first */
				lt_motor_control(motor,MOTOR_CTRL_OUTPUT_ANGLE,&res);
				rt_kprintf("motor output_angle: %.2f degree \n",res);
			}
		}
	}
}

static void _test_stepper(int argc, char*argv[])
{
	static struct lt_stepper_config_accel acc_config;
	static struct lt_stepper_config_interp int_config;
	int_config.x_stepper = x_stepper;
	int_config.y_stepper = y_stepper;
	
	if(!rt_strcmp(argv[2],"trapzoid"))
	{
		if(argc == 7)		/* check paramaters */
		{
			int step = atoi(argv[3]);
			float acc = (float)atof(argv[4]);
			float dec = (float)atof(argv[5]);
			float speed = (float)atof(argv[6]);
			acc_config.accel = acc;
			acc_config.decel = dec;
			acc_config.speed = speed;
			acc_config.step = step;
			lt_motor_control(motor,STEPPER_CTRL_TRAPZOID_ACCELERATE,&acc_config);
		}
	}
	else if(!rt_strcmp(argv[2],"s_curve"))
	{
		if(argc == 7)		/* check paramaters */
		{
			int step = atoi(argv[3]);
			float max = (float)atof(argv[4]);
			float min = (float)atof(argv[5]);
			float flex = (float)atof(argv[6]);
			acc_config.step = step;
			acc_config.freq_max = max;
			acc_config.freq_min = min;
			acc_config.flexible = flex;
			lt_motor_control(motor,STEPPER_CTRL_S_CURVE_ACCELERATE,&acc_config);
		}
	}
	else if(!rt_strcmp(argv[2],"line_interp"))
	{
		if(argc == 5)		/* check paramaters */
		{
			int x_pos = atoi(argv[3]);
			int y_pos = atoi(argv[4]);
			int_config.x_end = x_pos;
			int_config.y_end = y_pos;
			lt_motor_control(motor,STEPPER_CTRL_LINE_INTERPOLATION,&int_config);
		}
	}
	else if(!rt_strcmp(argv[2],"circular_interp"))
	{
		if(argc == 7)		/* check paramaters */
		{
			int x_start = atoi(argv[3]);
			int y_start = atoi(argv[4]);
			int x_end 	= atoi(argv[5]);
			int y_end 	= atoi(argv[6]);
			int_config.x_start = x_start;
			int_config.y_start = y_start;
			int_config.x_end = x_end;
			int_config.y_end = y_end;
			lt_motor_control(motor,STEPPER_CTRL_CIRCULAR_INTERPOLATION,&int_config);
		}
	}
}
#endif
#endif
#endif


