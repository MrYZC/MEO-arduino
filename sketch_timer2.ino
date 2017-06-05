//arduino的中断分为两类：外部中断和定时器中断
//开中断interrupt()和关中断nointerrupt()，用法如下
//可以被中断的函数
//interrupt()
//不可被中断的代码
//nointerrupt()
//可以被中断的代码
//二、外部中断
//1.attachinterrupt(interrupt,function(),mode)
//   1)interupt:中断号，UNO只能使用0或1，即代表D2与d3口。
//   2）function:调用中断函数，中断发生时调用的函数
//   3）mode:中断触发模式。
//   UNO R3支持四种中断触发模式：
//   LOW 当针脚输入为低时触发中断
//   CHANGE 当针脚发生改变时触发中断
//   RISING 当针脚由低变高时触发中断
//   FALLING 当针脚输入由高变低时触发中断
//   注意：
//1：中断服务程序不能够有参数和返回值。即void  FunctionName（void）{……..}
//2：在中断函数中delay()函数将不再起作用
//3：在中断函数中millis()函数的值将不会增加
//4：得到的串行数据将会丢失
//5：需要在中断函数内部更改的值需要声明为volatile类型
#include <MsTimer2.h>   //定时器库的头文件
int pin = 13;
volatile int state = LOW;
void setup() {
  // put your setup code here, to run once:
  pinMode(pin,OUTPUT);
  attachInterrupt(0,blink,CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(pin,state);
}
void blink()
{
  state = !state;
}
void flash()
{
  static boolean output = HIGH;
  digitalWrite(13,output);
  output = !output;
}
void setup()
{
  pinkMode(13,OUTPUT);

  MsTimer2::set(500,flash); //中断设置函数，每500ms进入一次中断
  MsTimer2::start(); //开始计时
}

