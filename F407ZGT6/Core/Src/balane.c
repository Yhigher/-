#include "balance.h"
#include "math.h"
#include "imu.h"
#include "odrive.h"
#include "usart.h"
#include "servo.h"
paramTypeDef param;
show_paramTypeDef show_param;
extern imu_t imu;
float blank = 0;
float PWM_X,PWM_accel,PWM_Final;                  // PWM�м���
int Flag_Stop = 0;
int cnt;//�ǶȻ�����
int cnt1;//�ٶȻ�����
int count = 0;//���ּ���
int cnt2 = 0;
char DataToSend[100];
int Steer_Target=0;//���pid����Ŀ��ֵ
int Steer_Target_Last=0;//���lastֵ
float vx;
void param_init(){
   param.angular_kp = -11.7;//���� -32.05 0 -6.205       12 5 0
    param.angular_ki = 0;
    param.angular_kd = -7.1;
    param.angular_v_kp = -2.3;//-2.3;
    param.angular_v_ki = 0;
    param.angular_v_kd = -0.98;
    param.fly_wheel_speed_kp = 0.75;
	
	
    param.fly_wheel_speed_ki = 0.32;
    param.fly_wheel_speed_kd = 0;
    param.angular_zero = -0.3;
    param.fly_whell_speed_target = 0;
    param.scope_flag = 0;
    param.Steer_Kp = 3;//���kp
    param.Steer_Ki = 0;//���kp
    param.Steer_Kd = 0;//���kd
}


//���ٶȻ�
float Angle_Velocity(float Gyro,float Gyro_Target)
{
    float Angle_Velocity_Bias;
    float PWM_Out;
    static float Angle_Velocity_Last_Bias,Angle_Velocity_Integral;
    Angle_Velocity_Bias = Gyro - Gyro_Target;
    Angle_Velocity_Integral+=Angle_Velocity_Bias;
    if(Angle_Velocity_Integral > 10000) Angle_Velocity_Integral =10000;                    //�����޷�
    if(Angle_Velocity_Integral < -10000) Angle_Velocity_Integral = -10000;                    //�����޷���������
    PWM_Out = param.angular_v_kp * Angle_Velocity_Bias + param.angular_v_ki * Angle_Velocity_Integral + param.angular_v_kd * (Angle_Velocity_Bias - Angle_Velocity_Last_Bias);
    Angle_Velocity_Last_Bias = Angle_Velocity_Bias;                             //�����ϴ����
    return PWM_Out;
}

int Error= 0;
//�ǶȻ�
float X_balance_Control(float Angle,float Angle_Zero,float gyro)
{
     float PWM,Bias;
     static float error,X_Balance_Last_Bias;
     Bias=Angle-Angle_Zero;                                            //��ȡƫ��
     error+=Bias;
	   Error = error-'0'+48;
	  //ƫ���ۻ�
     if(error>+30) error=+30;                                          //�����޷�
     if(error<-30) error=-30;                                          //�����޷�
    //  PWM=param.angular_kp*Bias + param.angular_ki*error + (Bias - X_Balance_Last_Bias)*param.angular_kd;   //��ȡ������ֵ
    PWM=param.angular_kp*Bias + param.angular_ki*error + (gyro)*param.angular_kd;   //��ȡ������ֵ
     X_Balance_Last_Bias = Bias;                                        //��¼�ϴ�ƫ��
     return PWM;
}


//�ٶȻ�
float Velocity_Control(int encoder,int target_encoder)
{
    float encoder_bias,Velocity;
    static float encoder_bias_integral;
    encoder_bias = encoder - target_encoder;
    encoder_bias_integral += encoder_bias;
    if(encoder_bias_integral > +500) encoder_bias_integral = +500;                    //�����޷�
    if(encoder_bias_integral < -500) encoder_bias_integral = -500;                    //�����޷���500
    Velocity = encoder_bias * param.fly_wheel_speed_kp/10 + encoder_bias_integral * param.fly_wheel_speed_ki/1000;
	
	
	//  count+=1;
//	  if(count>=100){
//			char buf[32];
//			sprintf(buf, "%f\n", encoder_bias_integral);
//			for(int i=0; i < strlen(buf); i++)
//				ITM_SendChar(buf[i]);
//		count=0;
//			if(encoder_bias_integral > 300 || encoder_bias_integral < -300)
//				encoder_bias_integral = 0;
//		}
    return Velocity;  
	
}
extern volatile int g_start;
void balance(void)
{
  count++;//���ּ�����
  cnt++;//�Ƕ�
	imu_task();
  cnt1++;//�ٶ�
	param_init();
   vx = low_pass_filter(imu.vx);
	
    /////// �����ֿ���//////////����
     if(cnt1>=50){PWM_accel = Velocity_Control(odrive.now_speed0,0);cnt1=0;}   //�����ֵ���ٶȻ������� �ٶ������Ҹ�
     if(cnt>=5){PWM_X = X_balance_Control(imu.rol,param.angular_zero+PWM_accel,vx);cnt=0;}// �����ֵ�������������  �Ƕ�������
     PWM_Final = Angle_Velocity(vx,PWM_X);        //���ٶȻ�  ���ٶ�������
     odrive.set_speed0 = PWM_Final;//�ٶ����������Ҹ�
		 
		 //odrive.set_speed0 = low_pass_filter2(odrive.set_speed0);

    //���� -32.05 0 -6.205       12 5 0
    // if(cnt>=3)
	// 	{		
	// 			PWM_X = X_balance_Control(imu.rol,param.angular_zero,imu.vx);
	// 			PWM_accel = Velocity_Control(odrive.now_speed0,0);
	// 			cnt = 0;
	// 	}
    // odrive.set_speed0 = PWM_X + PWM_accel;//�ٶ����������Ҹ�
    
    //�������޷�
    if(odrive.set_speed0>50) odrive.set_speed0=50;        // �����ֵ���޷�
    else if(odrive.set_speed0<-50) odrive.set_speed0=-50; // �����ֵ���޷�
		 
		//���ֿ�ʼ�˶� 

    //ˤ��ͣ���ж�
     if((imu.rol-param.angular_zero)>3 || (imu.rol-param.angular_zero)<-3){
         Flag_Stop = 1;
     }
     //else Flag_Stop = 0;

    if(Flag_Stop == 1 || g_start==0){
        odrive.set_speed0 = 0;
			  odrive_speed_ctl(0, 0);
    }

    // ���pid
    Steer_Target =  Steer_Engine_control(caramera.pos_x); //���pid

    Steer_Target = Steer_Speed_Limit(Steer_Target,Steer_Target_Last,3); // ����������
    servo_set_duty(Steer_Target);//�������
    Steer_Target_Last = Steer_Target;//��¼�ϴδ��ֵ

}



void ANODT_Send2(unsigned short _a,unsigned short _b,unsigned short a1,unsigned short b1,unsigned short a2,unsigned short b2)
{
    unsigned char _cnt = 0;
    unsigned char  sc = 0;
    unsigned char  ac = 0;
    unsigned char  i = 0;
    DataToSend[_cnt++] = 0xAA;
    DataToSend[_cnt++] = 0xFF;
    DataToSend[_cnt++] = 0xF1;
    DataToSend[_cnt++] = 12;

    DataToSend[_cnt++] = BYTE0(_a);
    DataToSend[_cnt++] = BYTE1(_a);

    DataToSend[_cnt++] = BYTE0(_b);
    DataToSend[_cnt++] = BYTE1(_b);

    DataToSend[_cnt++] = BYTE0(a1);
    DataToSend[_cnt++] = BYTE1(a1);

    DataToSend[_cnt++] = BYTE0(b1);
    DataToSend[_cnt++] = BYTE1(b1);

    DataToSend[_cnt++] = BYTE0(a2);
    DataToSend[_cnt++] = BYTE1(a2);

    DataToSend[_cnt++] = BYTE0(b2);
    DataToSend[_cnt++] = BYTE1(b2);

    for(i = 0; i< _cnt;i++){
            sc += DataToSend[i];
            ac += sc;
    }
    DataToSend[_cnt++] = sc;
    DataToSend[_cnt++] = ac;

    // UART_PutBuff(UART3, DataToSend, _cnt);
    //todo ����2 ����
    // for(i = 0; i< _cnt;i++){
    // HAL_UART_Transmit_DMA(&huart2,(uint8_t*)msg,strlen(msg));
    // HAL_UART_Transmit_DMA(&huart2,(uint8_t*)DataToSend,_cnt);
    HAL_UART_Transmit(&huart2,(uint8_t*)DataToSend,_cnt,100);
}