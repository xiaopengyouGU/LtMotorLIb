#include "lt_motor_control.h"
#include "protocol.h"
#include "key.h"

/* when communicating with upper computer, FINSH is forbidden */
rt_align(RT_ALIGN_SIZE)	
#ifndef RT_USING_FINSH
static rt_thread_t process_t;
#endif
//test event 
#define EVENT_KEY0	(1 << 3)
#define EVENT_KEY_UP (1 << 5)
#define FORWARD_PIN		GET_PIN(F,6)
#define REVERSAL_PIN	GET_PIN(F,7)
#define OUTPUT_CHANNEL	PWM_CHANNEL_4
#define MEASURE_TIME	100	

#define TARGET_SPEED	120
#define TARGET_POSITION 150
/* ADC --> PA6
*  DAC --> PA4
*  DAC --> PA5
*  PWM --> PA3
*  Encoder --> PB6 ， PB7
*/
static rt_thread_t motor_t;
static rt_thread_t key_thread;
static rt_event_t key_event;
static lt_pid_t motor_pid;
static lt_motor_t motor;
static rt_timer_t motor_timer;

static lt_pid_t pos_pid;		/* position pid */
static lt_pid_t vel_pid;		/* velocity pid */

#ifdef RT_USING_FINSH
static void motor_thread_entry(void *parameter)
{
	float speed,angle;
	float output = 0;
	rt_uint8_t key_val;
	rt_uint32_t event;
	rt_uint32_t voltage;
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
		lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&output);
	
	}
}


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

static void timeout_func(void *parameter)
{
	float speed = lt_motor_measure_speed(motor,MEASURE_TIME);	/* measure time: 100ms */
	rt_kprintf("motor speed: %.2f r/s , %.2f rpm \n ",speed,speed* 60);	/* finsh must enable float output */
	if(speed == 0)
	{
		lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&speed);		/* avoid locking rotor*/
		return;
	}
}
#else 
static void timeout_func(void *parameter)
{
	float read_val = lt_pid_get_control(motor_pid);
	float curr_val = lt_pid_control(motor_pid,read_val);
	int temp = curr_val;									/* transform data */
	set_computer_value(SEND_FACT_CMD,CURVES_CH1,&temp,1);	/* send fact value to upper computer */
}

static void velocity_loop(void *parameter)
{
	float speed = lt_motor_measure_speed(motor,MEASURE_TIME) * 60;	/* get rpm */
	float control_u = 0.1f * lt_pid_control(motor_pid,speed);
	int t_speed = speed, t_control_u = control_u;							
	/* transform control force to duty circle of PWM output */
	lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&control_u);			/* output */
	/* send data to upper computer */
	set_computer_value(SEND_FACT_CMD,CURVES_CH1,&t_speed,1);
	set_computer_value(SEND_FACT_CMD,CURVES_CH2,&t_control_u,1);
}

static void position_loop(void *parameter)
{
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
	static rt_uint32_t count = 0;
	/* process position loop first, sample time 3T */
	if(count % 3 == 0)
	{
		float position = lt_motor_measure_position(motor);
		float desired_speed = 0.03f * lt_pid_control(pos_pid,position);
		lt_pid_set_target(vel_pid,desired_speed);
		int t_position = position;
		set_computer_value(SEND_FACT_CMD,CURVES_CH1,&t_position,1);
	}
	/* velocity loop follow, sample time T */
	float speed = lt_motor_measure_speed(motor,MEASURE_TIME);
	float control_u = 0.1f * lt_pid_control(vel_pid,speed);
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
  float speed = 0;
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
		int actual_temp = COMPOUND_32BIT(&frame_data[13]);    // 得到数据
		lt_pid_set_target(motor_pid,actual_temp);    // 设置目标值
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
        rt_timer_stop(motor_timer);              // 停止电机
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

void lt_motor_test(void)
{
	//rt_base_t forward = rt_pin_get("PA.6"), reversal = rt_pin_get("PA.7");
	key_init();										/* init key! */
	motor_pid = lt_pid_create(0,0,0,MEASURE_TIME);	/* init pid parameter, start from little parameter! */
	//motor_pid = lt_pid_create(0.01,0.0,0.1);		/* init pid parameter */
	lt_pid_set_target(motor_pid,TARGET_SPEED);		/* set target speed */
	if(motor_pid == RT_NULL) return;
	
	/* create a motor */
	motor = lt_motor_create("motor1",MOTOR_REDUCTION_RATIO,MOTOR_TYPE_DC);
	lt_motor_set_pwm(motor,PWM_NUM_1,OUTPUT_CHANNEL);
	lt_motor_set_encoder(motor,PULSE_ENCODER_NUM_1,ENCODER_TOTAL_RESOLUTION);
	lt_motor_set_dir_pins(motor,FORWARD_PIN,REVERSAL_PIN);
	
	if(motor == RT_NULL) return;
#ifdef RT_USING_FINSH
	motor_t = rt_thread_create("motor",
								motor_thread_entry,
								RT_NULL,
								1024,		/* thread stack size must big enough!!! */
								10,
								20);
		
	if(motor_t != RT_NULL)
	{
		rt_thread_startup(motor_t);	//启动线程
	}
	
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
		rt_thread_startup(key_thread);	//启动线程
	}
	motor_timer = rt_timer_create("timer_m",timeout_func,RT_NULL,MEASURE_TIME,RT_TIMER_FLAG_PERIODIC);
	if(motor_timer == RT_NULL) return;
	rt_timer_start(motor_timer);		/* start timer! */
#else
	//motor_timer = rt_timer_create("timer_m",timeout_func,RT_NULL,MEASURE_TIME,RT_TIMER_FLAG_PERIODIC);
	//motor_timer = rt_timer_create("timer_m",velocity_loop,RT_NULL,MEASURE_TIME,RT_TIMER_FLAG_PERIODIC);
	motor_timer = rt_timer_create("timer_m",position_loop,RT_NULL,MEASURE_TIME,RT_TIMER_FLAG_PERIODIC);
	if(motor_timer == RT_NULL) return;
	
	protocol_init();								/* init protocol to communicate with computer */
	process_t = rt_thread_create("process",
								process_thread_entry,
								RT_NULL,
								512,
								9,				
								20);
	if(process_t != RT_NULL)
	{
		rt_thread_startup(process_t);	//启动线程
	}
#endif
}

MSH_CMD_EXPORT(lt_motor_test, motor test);
