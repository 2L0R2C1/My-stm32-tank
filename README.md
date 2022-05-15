# My stm32 tank



#### 介绍
华南理工大学2022年无限机甲杯比赛中碱基互补配队的电控代码工程

功能：实现对一只能跑能射能爬台阶的“战车”控制

编译或安装方式：下载安装stm32cubemx，keil5编译器，及stm32f4xx相应hal库即可

作者：碱基互补配队电控组成员：林瑞创，江浩炯



注：这里有三份project，Mars_Rubber和Mars_Magnam都是底盘主控stm32f407vet6的工程，分别对应普通橡胶轮和麦克纳姆轮，  Turret为炮台主控stm32f411ceu6工程



#### 整体架构

##### 硬件

![frame](D:\Documents\my-stm32-tank\frame.png)

双MCU控制，stm32f407vet6作底盘主控，stm32f411ceu6作炮台主控

底盘是四驱小车，支持麦轮及普通橡胶轮，四个带编码器的直流减速有刷电机，前后两个舵机爬台阶结构

炮台目前一自由度，一个舵机控制俯仰角，一个带编码器的蜗杆直流有刷电机控制弹舱拨盘，两个555电机（或2312无刷电机）驱动摩擦轮。

支持ps2手柄和蓝牙上位机远程遥控


##### 软件

基于stm32 HAL库，stm32cubemx + keil5，面向对象思想模块化编写


#### 具体实现

##### Cubemx配置

![cubemx-f407](D:\Documents\my-stm32-tank\cubemx-f407.png)

![cubemx-f411](D:\Documents\my-stm32-tank\cubemx-f411.png)

这里关于定时器、编码器、pwm、usart怎么配置的可以看这篇[博客：【STM32】HAL库 STM32CubeMX系列学习教程](https://blog.csdn.net/as480133937/article/details/99935090)

这里用到了很多定时器资源，没有用freertos，一块f407板子上资源不够，所以再加f411作辅助补充



##### 面向对象模块化编程

这里把电机类、舵机类等包装成了对象（C中没有C++的类，用结构体模仿），把底层的硬件代码实现都封装成一个个功能函数（接口）供用户调度使用

比如下面的motor.h封装细节

```c
typedef struct
{
	volatile float actual_speed;		//电机实际转速
	volatile float target_speed;		//电机目标转速
	volatile float actual_angle;		//电机实际转过角度
	volatile float target_angle;		//电机目标转过角度
	
	volatile int encoder_overflow;	//各电机encoder溢出次数
	volatile int capture_count;		//当前编码器计数值
	volatile int last_count;			//上一时刻编码器计数值
	volatile int pwm;				//输出给电机pwm值
	
	volatile u8 mode;
	
	PID_TypeDef k_angle;	//带编码器电机对应位置环pid参数
	PID_TypeDef k_speed;	//带编码器电机对应速度环pid参数
	PID_TypeDef k_double;	//带编码器电机对应双环pid参数
	
	TIM_HandleTypeDef timepwm1; //电机in1 pwm对应定时器
	u16 channelpwm1;			//电机in1对应pwm输入通道
	
	
	TIM_HandleTypeDef timepwm2; //电机in2 pwm对应定时器
	u16 channelpwm2;			//电机in2对应pwm输入通道
	
//	GPIO_TypeDef *in1_port,*in2_port;//带编码器电机由IN1，IN2电平关系控制正反转
//	u16 in1_pin,in2_pin;
	
	TIM_HandleTypeDef encoder;	//电机encoder对应定时器
	TIM_HandleTypeDef capturetim;//定时负反馈
	
}MOTOR_TypeDef;	
	
extern MOTOR_TypeDef Forward_L,Forward_R,Back_L,Back_R;

void MOTOR_Init(void);		//初始化所有电机

void MOTOR_Tim_Init(MOTOR_TypeDef *motor);//启用某个电机

void MOTOR_Tim_Stop(MOTOR_TypeDef *motor);//停用某个电机

void MOTOR_reset(void);					  //重置所有电机

float get_speed(float nms,MOTOR_TypeDef *motor);	//测速
float get_angle(MOTOR_TypeDef *motor);	//测角度位置

void set_motor_pwm(MOTOR_TypeDef *motor);	//pwm输出	

void limit_pwm_angle(MOTOR_TypeDef *motor);		//限幅pwm
void limit_pwm_speed(MOTOR_TypeDef *motor);		//限速
void check_pwm_speed(MOTOR_TypeDef *motor);	//检查pid算出的pwm与目标速度关系是否近似对应
void feedback_angle(MOTOR_TypeDef *motor);	//反馈角度信号给电机
void feedback_speed(MOTOR_TypeDef *motor);  //反馈速度信号给电机
void feedback_angle_double(MOTOR_TypeDef *motor, float ms);//双环pid

```

其实stm32hal库底层就是这么编写的，代码是模仿着来的，这样写的好处也是十分明显的，条理清晰，容易debug，易于移植到不同芯片上，而且将底层硬件代码与上层控制代码隔离开，这样用户即使不懂底层也能通过接口函数自行设计功能，操纵机甲。



##### 电机PID控制

关于编码器教程：

[带编码器的直流减速电机_英雄的小白的博客-CSDN博客_mg513编码器减速电机](https://blog.csdn.net/bj318318/article/details/107333918)

 [控制基础\] 定时器TIM的输入捕获——正交编码器模式与PWM输入（基于STM32F103+CubeMX+HAL）_丶漂泊の太阳的博客-CSDN博客_编码器输入捕获](https://blog.csdn.net/csol1607408930/article/details/112793292)

关于pid教程（网上很多，这里举一个我看得比较多的）

[电机控制进阶——PID速度控制_码农爱学习的博客-CSDN博客_电机pid控制算法](https://blog.csdn.net/hbsyaaa/article/details/117003801?ops_request_misc=%7B%22request%5Fid%22%3A%22165249712116780366599244%22%2C%22scm%22%3A%2220140713.130102334.pc%5Fall.%22%7D&request_id=165249712116780366599244&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-5-117003801-null-null.142^v9^pc_search_result_cache,157^v4^new_style&utm_term=pid&spm=1018.2226.3001.4187)

这里对电机有三种pid控制，速度环，角度环，双环，不同作战模式由不同pid控制，其中双环用于弹舱中对拨盘转动的精确控制（确保供弹的稳定性与准确性）

```C
set_pid(&motor->k_speed, 15, 3, 50);
set_pid(&motor->k_angle, 50, 0.5, 100);
set_pid(&motor->k_double, 1.2, 0.005, 25);
```

```c
void feedback_angle(MOTOR_TypeDef *motor){		//角度环
	volatile float angle = get_angle(motor);	//获取实际角度
	volatile float target = motor -> target_angle;
    motor->pwm = (int)PID_position(target,angle, PWM_MAX/1.2, &motor->k_angle);	//pid算出需要输出的pwm

    motor->actual_angle = angle;
    limit_pwm_angle(motor);
    set_motor_pwm(motor);		//反馈电机
}
```
```C
void feedback_speed(MOTOR_TypeDef *motor){			//速度环
	volatile float speed = get_speed(0.01,motor); 	//获取实际速度
	volatile float target = motor ->target_speed;
	volatile int pwm=0;
	
	if( speed - (float)SPEED_MAX/1.5f > 0.5f ) speed = (float)SPEED_MAX/1.5f;	//剔除过大测量值（硬件纹波有噪点）
	if( speed + (float)SPEED_MAX/1.5f < -0.5f) speed = -(float)SPEED_MAX/1.5f;
//	if( speed - motor->actual_speed > SPEED_MAX ) speed = motor->actual_speed + SPEED_MAX;	
//	if( speed - motor->actual_speed < -SPEED_MAX ) speed = motor->actual_speed - SPEED_MAX;
	
	motor -> actual_speed = speed;
	
	pwm = (int)PID_incremental(target*jiansubi, speed*jiansubi,  PWM_MAX/1.2, &motor->k_speed); 	//pid算出需要输出的pwm
	
	if(pwm > PWM_MAX/1.2) pwm = PWM_MAX/1.2;	//限pwm
	if(pwm < -PWM_MAX/1.2)pwm = -PWM_MAX/1.2;
	
	motor->pwm = pwm;
	
	set_motor_pwm(motor);		//反馈电机
}
```
```C
void feedback_angle_double(MOTOR_TypeDef *motor, float ms){ //双环pid，让电机在匀速状态下转到指定角度
	motor->actual_angle = get_angle(motor);	
	
	motor->target_speed = PID_position(motor->target_angle, motor->actual_angle, SPEED_MAX*jiansubi, &motor->k_double)/jiansubi; //位置环pid算出需要的速度
	
	if(motor->target_speed > (float)SPEED_MAX/3.0f ) motor->target_speed = (float)SPEED_MAX/3.0f;
	if(motor->target_speed < -(float)SPEED_MAX/3.0f ) motor->target_speed = -(float)SPEED_MAX/3.0f;
	
	motor->actual_speed = (float) (motor->capture_count - motor->last_count) / (beipin*xianshu*jiansubi*ms); //实际转速 
	motor->last_count = motor->capture_count;
	
	motor->pwm = (int)PID_incremental(motor->target_speed*jiansubi, motor->actual_speed*jiansubi, PWM_MAX, &motor->k_speed); //速度环pid算出应输出的pwm
		
	limit_pwm_angle(motor);
	set_motor_pwm(motor);	//反馈电机
}
```





##### 舵机控制

就是pwm输出，pwm占空比大小对应角度大小，[STM32CubeMX笔记（9）--定时器生成PWM特定波形，控制舵机转动_杰尼君的博客-CSDN博客](https://blog.csdn.net/weixin_44444810/article/details/120675567?ops_request_misc=%7B%22request%5Fid%22%3A%22165249816616781667884925%22%2C%22scm%22%3A%2220140713.130102334.pc%5Fall.%22%7D&request_id=165249816616781667884925&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-5-120675567-null-null.142^v9^pc_search_result_cache,157^v4^new_style&utm_term=stm32cubemx舵机&spm=1018.2226.3001.4187)

下面是ps2手柄控制下的爬台阶代码，其实就是前后两个舵机的升降和前后轮的前进配合而已

```C
	case PSB_PAD_DOWN : {	//前爪降，后爪归位
			while(PS2_DataKey()==PSB_PAD_DOWN&&t<3)t++,delay_ms(50);
			if(Steer_f.angle < 80){
				Steer_f.angle += 10, Steer_b.angle = 0; 
				set_steer_pwm(&Steer_f);
				set_steer_pwm(&Steer_b);
			}
			break;
		}
	
	case PSB_PAD_UP : {		//后爪降，前爪归位
		while(PS2_DataKey()==PSB_PAD_UP&&t<3)t++,delay_ms(50);
		if(Steer_b.angle < 80){
			Steer_f.angle = 0, Steer_b.angle += 10;
			set_steer_pwm(&Steer_f);
			set_steer_pwm(&Steer_b);
		}
		break;
	}
	
	case PSB_PAD_LEFT : {	//前后爪归位
		while(PS2_DataKey()==PSB_PAD_LEFT&&t<3)t++,delay_ms(50);
		Steer_f.angle = 0, Steer_b.angle = 0;
		set_steer_pwm(&Steer_f);
		set_steer_pwm(&Steer_b);
	}
	
	case PSB_PAD_RIGHT : {	//全自动爬梯
		while(PS2_DataKey()==PSB_PAD_RIGHT)delay_ms(50);//等待按键松开，限制按下并松开为一次有效按键
		
		MOTOR_reset(),control_mode=1;	delay_ms(500);
		
		Steer_f.angle = 80;	 
		delay_ms(3000);	
		Back_L.target_angle = 400; 
		Back_R.target_angle = 400;
		delay_ms(3000);	
		Steer_f.angle =0;
		Steer_b.angle = 80;
		delay_ms(3000);
		Forward_L.target_angle = 1080; 
		Forward_R.target_angle = 1080;
		delay_ms(3000);

		MOTOR_reset(),control_mode=0;	delay_ms(500);
		
		break;
	}
```





##### 串口通信

###### 上位机通信

使用usart1，没有dma

电脑端使用串口助手，手机端使用轮趣科技的“minibalance”APP，通信协议包装在communication，datascope文件中

###### 双芯通信

直接通过串口传送指令，这里用的是f407的usart3 和 f411的usart6，接线

usart3->rx  ——  usart6->tx

usart3->tx  ——  usart6->rx

**gnd —— gnd    共地！共地！共地！**

###### 与树莓派通信

使用usart3收发（没开dma空闲接收）

```C
/*三个串口通信

串口一: 核心控制与调试

串口二：与树莓派通信

串口三：与炮台主控芯片（stm32f411ceu6）通信

*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart ->Instance == USART1){		//串口1
		
		bluetooth_control(receiver1);	//非ps2控制下用手机蓝牙控制小车
		
		HAL_UART_Receive_IT(&huart1,(uint8_t *)&receiver1,1);
	}
	
	if(huart ->Instance == USART2){		//串口二，与树莓派通信
		
		vision_control(receiver2);
		
		HAL_UART_Receive_IT(&huart2,(uint8_t *)&receiver2,1);
	}
	
	if(huart ->Instance == USART3){		
//		printf("receiver3! %c",receiver3);
//		HAL_UART_Transmit(&huart1,(u8 *)&receiver3,1,1);
		HAL_UART_Receive_IT(&huart3,(int8_t *)&receiver3,1);
	}
}
```

与视觉通信代码片段

```C
int x=0,y=0; const int m=10;

float xp[50],yp[50];
float xx=0.0f,yy=0.0f;


void getdata(){		//接收并转换树莓派数据 
	HAL_UART_Receive(&huart2,rx2_buffer,10,10);	//接收一组数据帧
	
	x = (rx2_buffer[2]-'0')*10 + (rx2_buffer[3]-'0');//数据转换
	if(rx2_buffer[1]=='-')x=-x;
	y = (rx2_buffer[7]-'0')*10 + (rx2_buffer[8]-'0');
	if(rx2_buffer[6]=='-')y=-y;
	
	for(int t=0;t<10;t++)rx2_buffer[t]=0;	//清空数据缓存
}


void vision_control(u8 order){
	static int t=0;  if(t>m)t=0; t++; int q=0;	
	
	if(order=='G')getdata(), y=-y;
	if(order=='N')x=0,y=0,printf("can not fine the target\r\n");
	
	if(x>-30&&x<30)xp[t] = x; else xp[t]=0;		//过滤过大值
	if(y>-15&&y<15)yp[t] = y;	else yp[t]=0;
	xx=0.0f,yy=0.0f;
//	xx=x; yy=y;
	while(q<m)xx += xp[q]/(float)m, yy += yp[q]/(float)m, q++;	//均值处理，使移动平滑
	xx *= 7; yy *= 0.5;
	
	Forward_L.target_angle += xx;
	Forward_R.target_angle -= xx;
	Back_L.target_angle += xx,	
	Back_R.target_angle -= xx;
	if(0<=Turret.angle+yy&&Turret.angle+yy<=50)Turret.angle += yy;
	
	printf("xx=%f yy=%f\r\n",xx,yy);
}

```





##### PS2手柄控制

软件模拟spi，采用轮询方式读取指令，

ps2原理[PS2手柄通讯协议解析---附资料和源码_冬瓜~的博客-CSDN博客_ps2手柄接线](https://blog.csdn.net/weixin_44793491/article/details/105781595)

代码移植[ 用cube移植PS2手柄--HAL库_Rodeson-James的博客-CSDN博客](https://blog.csdn.net/qq_43206638/article/details/103301762)

```C
while (1)
{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if(ps2_mode){			//ps2手柄控制模式
        if( !PS2_RedLight()){ //手柄为绿灯模式

            delay_ms(50);	 //延时很重要不可去

            //		  if(control_mode)MOTOR_reset(),control_mode=0;

            ps2_control( PS2_DataKey() );

        }else{				//手柄为红灯模式

            delay_ms(50);	 //延时很重要不可去

            if(!control_mode)MOTOR_reset(),control_mode=1;

            //	  ps2_control( PS2_DataKey() );
            ps2_angle();

        }
    }
}
```

ps2摇杆模拟量

![ps2摇杆模拟量](D:\Documents\my-stm32-tank\ps2.jpg)

```C
void ps2_speed(void){		//正常前进模式
	
	i8 x = PS2_AnologData(PSS_RX) - 128; //右摇杆x坐标映射为小车x方向速度
	i8 y = 127 - PS2_AnologData(PSS_RY); //右摇杆y坐标映射为小车y方向速度
	i8 w = PS2_AnologData(PSS_LX) - 128; //左摇杆x坐标映射为小车角速度
	i8 z = PS2_AnologData(PSS_LY) - 127; //左摇杆y坐标映射为炮台俯仰角增量


/*	n = geer*5	; //if(n==5)n=2;		//均值滤波，让小车平稳启动与停下
	
	for(i=n;i>0;i--)V[i]=V[i-1], W[i]=W[i-1];//更新近n次计算值 
		  
	V[0] = geer*(float)y/128.0f; //摇杆y坐标映射为小车前进速度，速度范围-geer ~ geer
											 
	W[0] = (geer/3)*(float)x/128.0f; //摇杆x坐标映射为小车转弯量
	
	v += (V[0]-V[n])/(float)n;   w += (W[0]-W[n])/(float)n;											 
	//以近n次速度与转弯量平均值作为小车新一轮前进速度与转弯量，抑制小车速度突变
	//（这样做一是为了模拟真实开车，二是小车速度突变为0会导致ps2手柄断联）		
//	printf("v = %f\r\n w = %f\r\n",v,w);
*/	

	float vx = 2.5 * (float)x / 128;	//x、y坐标映射为小车x、y方向分速度（最大2m/s）
	float vy = 2.5 * (float)y / 128;
	float ww = 4 * (float)w / 128;		//w坐标映射为小车转速（最大3rad/s）
	z = 5*(float)z/128.0f;				//炮台俯仰角增量
	
	Forward_R.target_speed = vy - vx - ww*(a+b);	//麦轮逆运算
	Forward_L.target_speed = vy + vx + ww*(a+b);
	Back_L.target_speed = vy - vx + ww*(a+b);
	Back_R.target_speed = vy + vx - ww*(a+b);	
	
	if(0<=Turret.angle+z&&Turret.angle+z<=48){
		Turret.angle += z;
	}
}
```

关于麦轮的运动解算可看[麦克纳姆轮及其速度分解计算_一把木剑的博客-CSDN博客_麦克纳姆轮速度分解](https://blog.csdn.net/banzhuan133/article/details/69229922)





------

### 持续更新中......

