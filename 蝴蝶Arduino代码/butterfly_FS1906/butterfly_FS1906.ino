#include <Servo.h>
Servo servo_A0;            //定义舵机A0
Servo servo_A1;               //定义舵机A1
int datachan[10];            //数组用来保存遥控器接收值
int x,y,chx;                //遥控接收程序中用到的变量
int channel1;                //通道1
int channel2;                //通道2
int channel3;                   //通道3
int channel4;                //通道4
int channel5;                 //通道5
int channel6;                   //通道6
int channel7;                      //通道7
int pulsewidth_0;          //舵机控制函数舵机的脉冲宽500-2500
int pulsewidth_1;          //舵机控制函数舵机的脉冲宽500-2500
int servoC_0;                //舵机A0中间值一般在1500左右 该机型初始值为1410
int servoC_1;                //舵机A1中间值一般在1500左右 该机型初始值为1490
int fd,pj,cs,sj,c5,c6;             //幅度,偏角,差速,升降,c5c6用于微调舵机初始位置。
int hx;                         //滑翔角度
float ys;                        //延时,
int fly=0;                      //起飞标志位
void dataget();               //获取ppm函数
void do_sv();                 //舵机控制函数

void do_sv()
{servo_A0.writeMicroseconds(pulsewidth_0);
  servo_A1.writeMicroseconds(pulsewidth_1);}//通过控制脉冲宽度来控制舵机的角度


void setup()                        // 初始化
{
 // Serial.begin(115200);               //开启后可进行串口调试
  servo_A0.attach(A0);                   //舵机端口A0
  servo_A1.attach(A1);                      //舵机端口A1
  servoC_0=1410;                                //舵机A0角度初始值
  servoC_1=1490;                                //舵机A1角度初始值
  fd=400;                                       //幅度初始值400
  ys=10000;                                        //延时初始值10000单位微秒
  pj=0;                                     //偏角初始0
  cs=0;                                     //差速初始0
  sj=0;                                       //升降初始0
         
  pulsewidth_0=servoC_0;                       //舵机A0脉冲宽度设为初始值
  pulsewidth_1=servoC_1;                        //舵机A0脉冲宽度设为初始值
  servo_A0.writeMicroseconds(pulsewidth_0);        //输出相应脉冲宽度控制舵机A0
  servo_A1.writeMicroseconds(pulsewidth_1);         //输出相应脉冲宽度控制舵机A1
   delay(3000);                                  //等待三秒
                
}

void loop()                                         //主程序重复执行
{
 dataget();                                        //采集接收机返回值
channel1=datachan[0];//580 990 1400                  //将1通道返回值保存在变量channel1
channel2=datachan[1];//1400 990  580                  //将2通道返回值保存在变量channel2
channel3=datachan[2];// 1400 990  580                //将3通道返回值保存在变量channel3
channel4=datachan[3];// 580 990  1400                   //将4通道返回值保存在变量channel4
channel5=datachan[4];//1400 990  580                   //将5通道返回值保存在变量channel5
channel6=datachan[5];//580 990  1400                //将6通道返回值保存在变量channel6
channel7=datachan[6];//580 990  1400                    //将71通道返回值保存在变量channel7
//pj=map(channel4,595,1595,-250,250);                       //映射通道四的值用于控制偏角左倾或右倾 此处仿生蝴蝶不用扑翼机可以用
sj=map(channel2,595,1595,-200,200);if(abs(sj)<5)sj=0;      //映射通道2 的值给变量sj 用于控制蝴蝶升降 sj范围是-200到200
ys=map(channel3,595,1595,10000,6000);                      //映射通道3 的值给变量ys 用于控制蝴蝶扑翼频率 ys范围是10000到6000
cs=map(channel1,595,1595,-100,100);if(abs(cs)<5)cs=0;       //映射通道1 的值给变量cs 用于控制蝴蝶两侧幅度差异 可以控制转向 cs范围是-100到100
c5=map(channel5,595,1595,0,200);                          //映射通道5 的值给变量c5 用于控制A0舵机初始位置
c6=map(channel6,595,1595,0,200);                           //映射通道6 的值给变量c6 用于控制A1舵机初始位置
servoC_0=1410-c5;                                          //通过旋钮来微调舵机A0初始位置
servoC_1=1490+c6;                                          //通过旋钮来微调舵机A1初始位置
if(channel7>1200)                                            //通道7为三挡  分别控制三种幅度分别是  400  500  600
  fd=400;                                                    //
  else if(700<channel7&&channel7<1200)                       //
  fd=500;                                                    //  
  else if(channel7<700) fd=600;                              //
if(ys>9500)                                                    //当通道3在最下端时映射ys的值为10000左右  这时候fly为0 当ys小于9500时fly为1
fly=0;                                                            //
else fly=1;                                                     // 
if(!fly)                                                         //fly为0时不启动
     {pulsewidth_0=servoC_0+sj;                                         // 
      pulsewidth_1=servoC_1-sj;                                            //在不启动时只能通过通道2来控制升降
      do_sv();}                                                    //舵机按照上面的值进行执行
if(fly)                                                           //fly为1时启动 
 {                                                                   //
 for (int i=0;i<18;i++)                                               //将扑翼的半个周期分为18份分别是 cos0 cos10 cos20 cos30...cos170
    {                                                                   //
      pulsewidth_0=(servoC_0+sj)+(fd-cs)*cos((10*i)/180.0*3.14);            //y=Acos(wx）+h     其中A控制了余弦函数的幅度  h控制了余弦函数的0点位置
      pulsewidth_1=(servoC_1-sj)-(fd+cs)*cos((10*i)/180.0*3.14);     //通过改变参数来实现控制舵机扑翼的各种动作
      do_sv();                                                         //将计算好的数值即脉冲宽度给舵机 执行相应的角度
      delayMicroseconds(ys);//Serial.println(pulsewidth_0);            //ys越小 半个周期的时间越短 速度就越快实现了通道3控制频率
    }                                                                  // 
 for (int i=18;i>0;i--) 
    {                                
      pulsewidth_0=(servoC_0+sj)+(fd-cs)*cos((10*i)/180.0*3.14);               //
      pulsewidth_1=(servoC_1-sj)-(fd+cs)*cos((10*i)/180.0*3.14);               //
      do_sv();                                                            //
      delayMicroseconds(ys);//Serial.println(pulsewidth_0);                      //
    }   
 }

//Serial.println(pj);//偏角
//Serial.println(ys);//延时
//Serial.println(fd);//幅度
//Serial.println(cs);//差速
//Serial.println(sj);//升降
//Serial.println(hx);//滑翔角度
//Serial.println(pulsewidth_0);
 
}
void dataget()
{while(pulseIn(6,HIGH)<5000){}                //在发送8个脉冲信号以后有一个结束脉冲  结束脉冲较大一般大于5000 接下来依次是一到八通达脉冲数值大概是580到1400
for(x=0;x<8;x++)                                      
datachan[x]=pulseIn(6,HIGH);                  //将这8个脉冲分别保存在数组中
}
