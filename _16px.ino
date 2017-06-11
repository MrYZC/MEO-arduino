#include <Arduino.h>
#include <I2Cdev.h>
#include <MemoryFree.h>
#include <MsTimer2.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <GizWits.h>
#include <ringbuffer.h>


#include <Adafruit_NeoPixel.h>


#define PIN            2
#define NUMPIXELS      14



Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


int delayval = 500; // delay for half a second

uint8_t iii = 0;

/*************************** HAL define ***************************/
#define   KEY1              7
#define   KEY2              3

#define   KEY1_SHORT_PRESS  1
#define   KEY1_LONG_PRESS   2
#define   KEY2_SHORT_PRESS  4
#define   KEY2_LONG_PRESS   8
#define   NO_KEY            0
#define   KEY_LONG_TIMER    3   //( 3s )

#define   LIGHT_DELAY    30    //(30ms)


uint8_t gaterSensorFlag ;
uint8_t Set_LedStatus = 0;
uint8_t NetConfigureFlag = 0;
uint32_t Last_KeyTime = 0;
uint8_t scene = 0;
uint8_t scence = 0;
uint32_t previousMillis = 0;
uint8_t  lightI = 1;
bool direct = true;
uint8_t marqueeI = 0;


#if(DEBUG == 1)
SoftwareSerial mySerial(8, 9); // RX, TX
#endif

typedef struct
{ 
  uint8_t       Switch;
  uint8_t       Color_R1;
  uint8_t       Color_G1;
  uint8_t       Color_B1;
  uint8_t       Brightness;
} ReadTypeDef_t;

typedef struct
{ 
  uint8_t       Attr_Flags1;
  uint8_t       Switch;
  uint8_t       Color_R1;
  uint8_t       Color_G1;
  uint8_t       Color_B1;
  uint8_t       Brightness;
} WriteTypeDef_t;
int storeColor[48] ={};
WriteTypeDef_t  WriteTypeDef;
ReadTypeDef_t   ReadTypeDef;
uint8_t showType = 0;

void GizWits_ControlDeviceHandle(void);

/*******************************************************
      function      : gokit_time_ms
     
******************************************************/
unsigned long gokit_time_ms(void)
{
  return millis();
}
/*******************************************************
      function      : gokit_time_m

******************************************************/
unsigned long gokit_time_s(void)
{
  return millis() / 1000;
}

/*******************************************************
      function      : gokit_key1down
******************************************************/
char gokit_key1down(void)
{
  int unsigned long keep_time = 0;
  if (digitalRead(KEY1) == LOW)
  {
    delay(100);
    if (digitalRead(KEY1) == LOW)
    {
      keep_time = gokit_time_s();
      while (digitalRead(KEY1) == LOW)
      {
        if ((gokit_time_s() - keep_time) > KEY_LONG_TIMER)
        {
          Last_KeyTime = gokit_time_s();
          return KEY1_LONG_PRESS;
        }
      } //until open the key

      if ((gokit_time_s() - Last_KeyTime) > KEY_LONG_TIMER)
      {
        return KEY1_SHORT_PRESS;
      }
      return 0;
    }
    return 0;
  }
  return 0;
}

/*******************************************************
      function      : gokit_key2down
******************************************************/
char gokit_key2down(void)
{
  int unsigned long keep_time = 0;
  if (digitalRead(KEY2) == LOW)
  {
    delay(100);
    if (digitalRead(KEY2) == LOW)
    {
      keep_time = gokit_time_s();
      while (digitalRead(KEY2) == LOW) //until open the key
      {

        if ((gokit_time_s() - keep_time) > KEY_LONG_TIMER)
        {
          Last_KeyTime = gokit_time_s();
          return KEY2_LONG_PRESS;
        }
      }

      if ((gokit_time_s() - Last_KeyTime) > KEY_LONG_TIMER)
      {
        return KEY2_SHORT_PRESS;
      }
      return 0;
    }
    return 0;
  }
  return 0;
}

/*******************************************************
      function      : gokit_keydown
******************************************************/
char gokit_keydown(void)
{
  char ret = 0;
  ret |= gokit_key2down();
  ret |= gokit_key1down();
  return ret;
}

void KEY_Handle(void)
{
  /*  长按是指按住按键3s以上   */
  switch (gokit_keydown())
  {
    case KEY1_SHORT_PRESS:

      break;

    case KEY1_LONG_PRESS:
      // GizWits_D2WResetCmd();

      break;

    case KEY2_SHORT_PRESS:
      GizWits_D2WConfigCmd(SoftAp_Mode);
      NetConfigureFlag = 1;
      colorWipe(strip.Color(255, 0, 0), 1);


      break;

    case KEY2_LONG_PRESS:
      GizWits_D2WConfigCmd(AirLink_Mode);
      NetConfigureFlag = 1;
      colorWipe(strip.Color(0, 255, 0), 1);

      break;

    default:

      break;
  }
}

/*******************************************************************************
  Function Name  : GizWits_WiFiStatueHandle

*******************************************************************************/
void GizWits_WiFiStatueHandle(uint16_t wifiStatue)
{
  if (((wifiStatue & Wifi_ConnClouds) == Wifi_ConnClouds) && (NetConfigureFlag == 1) ) //&& (NetConfigureFlag == 1)
  {
    NetConfigureFlag = 0;
  }
}
/*******************************************************************************
  Function Name  : breath

*******************************************************************************/
void breathe(uint8_t wait) {
  uint32_t currentMillis = gokit_time_ms();
  if (currentMillis - previousMillis >= LIGHT_DELAY) {
    previousMillis=currentMillis;
    if (direct == true) {
      lightI++;
    } else {
      lightI--;
    }
    if ((lightI == 255) || (lightI == 0)) {
      direct = !direct;
      delay(wait);
    }
    strip.setBrightness(lightI);
    for( int numPx = 0;numPx < NUMPIXELS; numPx++){
	    strip.setPixelColor(numPx,strip.Color(storeColor[numPx],storeColor[numPx+16],storeColor[numPx+32]));
	    delay(1);
	    strip.show();
  }
     strip.show();
  }
}
/*******************************************************************************
  Function Name :rainbow
********************************************************************************/
void marquee(uint8_t wait){
  uint32_t currentMillis = gokit_time_ms();
  if (currentMillis - previousMillis >= 2*LIGHT_DELAY)
  {
    previousMillis = currentMillis;
    if (direct == true) 
    {
      marqueeI++;
    }
    else{
      marqueeI--;
    }
    if ((marqueeI == NUMPIXELS)||(marqueeI ==0))
    {
      direct = !direct;
      delay(wait);
    }
    for (uint8_t i = 0; i < strip.numPixels();i++)
    {
      if (i == marqueeI)
      {
        strip.setPixelColor(i-1,strip.Color(ReadTypeDef.Color_R1,ReadTypeDef.Color_G1,ReadTypeDef.Color_B1));
        strip.setPixelColor(i-1,strip.Color(ReadTypeDef.Color_R1,ReadTypeDef.Color_G1,ReadTypeDef.Color_B1));
        strip.setPixelColor(i-1,strip.Color(ReadTypeDef.Color_R1,ReadTypeDef.Color_G1,ReadTypeDef.Color_B1));
      }
      else{
        strip.setPixelColor(i,0,0,0);
      }
    }
    strip.show();
  }
}
 
/*******************************************************************************
  Function Name  : maqueen

*******************************************************************************/
void horseRace(uint8_t wait){
  uint32_t currentMillis = gokit_time_s();
  if (currentMillis-previousMillis >=3*LIGHT_DELAY)
  {
    previousMillis = currentMillis;
    colorWipe(strip.Color(0,0,0),100);
    delay(wait);
    strip.setBrightness(ReadTypeDef.Brightness);
    for( int numPx = 0; numPx<NUMPIXELS; numPx++){
      strip.setPixelColor(numPx,strip.Color(storeColor[numPx],storeColor[numPx+16],storeColor[numPx+32]));
      strip.show();
      delay(wait);
      strip.setPixelColor(numPx,strip.Color(0,0,0));
      strip.show();
   }
  strip.show();
  }
}
//  strip.setPixelColor(0,strip.Color(storeColor[0],storeColor[16],storeColor[32],256));
//  delay(wait);
//  strip.setPixelColor(0,strip.Color(storeColor[0],storeColor[16],storeColor[32],0));
//  delay(wait);
//  strip.setPixelColor(1,strip.Color(storeColor[1],storeColor[17],storeColor[33],256));
//  delay(wait);
//  strip.setPixelColor(1,strip.Color(storeColor[1],storeColor[17],storeColor[33],0));
//  delay(wait);
//  strip.setPixelColor(2,strip.Color(storeColor[2],storeColor[18],storeColor[34],256));
//  delay(wait);
//  strip.setPixelColor(2,strip.Color(storeColor[2],storeColor[18],storeColor[34],0));
//  delay(wait);
//  strip.setPixelColor(3,strip.Color(storeColor[3],storeColor[19],storeColor[35],256));
//  delay(wait);
//  strip.setPixelColor(3,strip.Color(storeColor[3],storeColor[19],storeColor[35],0));
//  delay(wait);
//  strip.setPixelColor(4,strip.Color(storeColor[4],storeColor[20],storeColor[36],256));
//  delay(wait);
//  strip.setPixelColor(4,strip.Color(storeColor[4],storeColor[20],storeColor[36],0));
//  delay(wait);
//  strip.setPixelColor(5,strip.Color(storeColor[5],storeColor[21],storeColor[37],0));
//  delay(wait);
//  strip.setPixelColor(5,strip.Color(storeColor[5],storeColor[21],storeColor[37],0));
//  delay(wait);
//  strip.setPixelColor(6,strip.Color(storeColor[6],storeColor[22],storeColor[38],0));
//  delay(wait);
//  strip.setPixelColor(6,strip.Color(storeColor[6],storeColor[22],storeColor[38],0));
//  delay(wait);
//  strip.setPixelColor(7,strip.Color(storeColor[7],storeColor[23],storeColor[39],0));
//  delay(wait);
//  strip.setPixelColor(7,strip.Color(storeColor[7],storeColor[23],storeColor[39],0));
//  delay(wait);
//  strip.setPixelColor(8,strip.Color(storeColor[8],storeColor[24],storeColor[40],0));
//  delay(wait);
//  strip.setPixelColor(8,strip.Color(storeColor[8],storeColor[24],storeColor[40],0));
//  delay(wait);
//  strip.setPixelColor(9,strip.Color(storeColor[9],storeColor[25],storeColor[41],0));
//  delay(wait);
//  strip.setPixelColor(9,strip.Color(storeColor[9],storeColor[25],storeColor[41],0));
//  delay(wait);
//  strip.setPixelColor(10,strip.Color(storeColor[10],storeColor[26],storeColor[42],0));
//  delay(wait);
//  strip.setPixelColor(10,strip.Color(storeColor[10],storeColor[26],storeColor[42],0));
//  delay(wait);
//  strip.setPixelColor(11,strip.Color(storeColor[11],storeColor[27],storeColor[43],0));
//  delay(wait);
//  strip.setPixelColor(11,strip.Color(storeColor[11],storeColor[27],storeColor[43],0));
//  delay(wait);
//  strip.setPixelColor(12,strip.Color(storeColor[12],storeColor[28],storeColor[44],0));
//  delay(wait);
//  strip.setPixelColor(12,strip.Color(storeColor[12],storeColor[28],storeColor[44],0));
//  delay(wait);
//  strip.setPixelColor(13,strip.Color(storeColor[13],storeColor[29],storeColor[45],0));
//  delay(wait);
//  strip.setPixelColor(13,strip.Color(storeColor[13],storeColor[29],storeColor[45],0));
//  delay(wait);
//  strip.setPixelColor(14,strip.Color(storeColor[14],storeColor[30],storeColor[46],0));
//  delay(wait);
//  strip.setPixelColor(14,strip.Color(storeColor[14],storeColor[30],storeColor[46],0));
//  delay(wait);
//  strip.setPixelColor(15,strip.Color(storeColor[15],storeColor[31],storeColor[47],0));
//  delay(wait);
//  strip.setPixelColor(15,strip.Color(storeColor[15],storeColor[31],storeColor[47],0));
//  delay(wait);
//  strip.show();
/*******************************************************************************
  Function Name  : colorWipe

*******************************************************************************/
void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}
void GoKit_Init()
{
#if(DEBUG==1)
  //自定义引脚通信SoftwareSerial初始
  mySerial.begin(9600);

#endif

  //按键初始
  pinMode(KEY1, INPUT_PULLUP); //KEY1 上拉输入
  pinMode(KEY2, INPUT_PULLUP); //KEY2 上拉输入


  memset(&ReadTypeDef, 0, sizeof(ReadTypeDef));
  memset(&WriteTypeDef, 0, sizeof(WriteTypeDef));
  GizWits_init(sizeof(ReadTypeDef_t));
}



void setup()
{
  GoKit_Init();
  strip.begin();
  strip.show();

}

void loop()
{
  uint8_t ret = 0;
  uint8_t buf[256];
  KEY_Handle();
  ret = GizWits_MessageHandle(buf, sizeof(WriteTypeDef_t));
  if (ret == 0)
  {
    memcpy((uint8_t *)&WriteTypeDef, buf, sizeof(WriteTypeDef_t));
    GizWits_ControlDeviceHandle();
    GizWits_DevStatusUpgrade((uint8_t *)&ReadTypeDef, 10 * 60 * 1000, 1, NetConfigureFlag);
  }
  
  /***************************场景模式****************************************/
  switch (scene)
  {
    case 0:
 
  strip.show();
      break;
    case 1:

  strip.show();
      break;
    case 2:

  strip.show();
      break;
    case 3:

  strip.show();
      break;
    case 4:
  strip.show();
      break;
    case 5:
  strip.show();
    break;
     case 6:
  strip.show();
    break;
     case 7:
  strip.show();
    break;
     case 8:
  strip.show();
    break;
     case 9:
  strip.show();
    break;
     case 10:
  strip.show();
    break;
     case 11:
  strip.show();
    break;
     case 12:
  strip.show();
    break;
    case 13:
  marquee(100);
    break;
    case 14:
   breathe(100);
    break;
    case 15:
   horseRace(100);
    break;
    default:
      break;
  }
/***************************两个场景****************************************/
//switch(scence){
//    case 1:
//    breathe(100);
//      break;
//    case 2:
//    maqueen(500);
//      break;
//      default:
//      break;
// }
}
void GizWits_ControlDeviceHandle(void)
{
  /***********照明是否打开*****************************************/
  if ( (WriteTypeDef.Attr_Flags1 & (1 << 0)) == (1 << 0))
  {
    //ReadTypeDef.Switch &= 0xF1;//将场景模式设为默认
    if ((WriteTypeDef.Switch & (1 << 0)) == (1 << 0))
    {
      colorWipe(strip.Color(255, 255, 255), 1);
      ReadTypeDef.Switch |= 0x01;
      ReadTypeDef.Color_R1 = 255;
      ReadTypeDef.Color_G1 = 255;
      ReadTypeDef.Color_B1 = 255;
      ReadTypeDef.Brightness = 255;
    }
    else
    {
      colorWipe(strip.Color(0, 0, 0), 1);
      ReadTypeDef.Switch &= ~(1 << 0);
      ReadTypeDef.Color_R1 = 0;
      ReadTypeDef.Color_B1 = 0;
      ReadTypeDef.Color_G1 = 0;
      ReadTypeDef.Brightness = 0;
      lightI = 0;
    }
  }
  /**************像素及场景模式*************************/
  if ( (WriteTypeDef.Attr_Flags1 & (1 << 1)) == (1 << 1))
  {

    /********************像素一*********************/
    if ( ((WriteTypeDef.Switch >> 1) &  0x0F) == 0x00)
    {
      ReadTypeDef.Switch &= 0xF1;
      scene = 0;
    }

    /********************像素二*********************/
    if ( ((WriteTypeDef.Switch >> 1) & 0x0F) == 0x01)
    {
      ReadTypeDef.Switch &= 0xF1;
      ReadTypeDef.Switch |= 0xF1;
      scene = 1;
    }
    /********************像素三*********************/
    if ( ((WriteTypeDef.Switch >> 1) &  0x0F) == 0x02)
    {
      ReadTypeDef.Switch &= 0xF1;
      ReadTypeDef.Switch |= 0xF2;
      scene = 2;
    }
    /********************像素四*********************/
    if ( ((WriteTypeDef.Switch >> 1) & 0x0F) == 0x03)
    {
      ReadTypeDef.Switch &= 0xF1;
      ReadTypeDef.Switch |= 0xF3;
      scene = 3;
    }
    /********************像素五*********************/
    if ( ((WriteTypeDef.Switch >> 1) &  0x0F) == 0x04)
    {
      ReadTypeDef.Switch &= 0xF1;
      ReadTypeDef.Switch |= 0xF4;
      scene = 4;
    }
    /********************像素六*********************/
    if ( ((WriteTypeDef.Switch >> 1) &  0x0F) == 0x05)
    {
      ReadTypeDef.Switch &= 0xF1;
      ReadTypeDef.Switch |= 0xF5;
      scene = 5;
    }
    /********************像素七*********************/
     if ( ((WriteTypeDef.Switch >> 1) & 0x0F) == 0x06)
    {
      ReadTypeDef.Switch &= 0xF1;
      ReadTypeDef.Switch |= 0xF6;
      scene = 6;
    }
    /********************像素八*********************/
     if ( ((WriteTypeDef.Switch >> 1) &  0x0F) == 0x07)
    {
      ReadTypeDef.Switch &= 0xF1;
      ReadTypeDef.Switch |= 0xF7;
      scene = 7;
    }
    /********************像素九*********************/
    if ( ((WriteTypeDef.Switch >> 1) &  0x0F) == 0x08)
    {
      ReadTypeDef.Switch &= 0xF1;
      ReadTypeDef.Switch |= 0xF8;
      scene =8;
    }
    /********************像素十*********************/
    if ( ((WriteTypeDef.Switch >> 1) &  0x0F) == 0x09)
    {
      ReadTypeDef.Switch &= 0xF1;
      ReadTypeDef.Switch |= 0xF9;
      scene =9;
    }
    /********************像素十一*********************/
    if ( ((WriteTypeDef.Switch >> 1) & 0x0F) == 0x0A)
    {
      ReadTypeDef.Switch &= 0xF1;
      ReadTypeDef.Switch |= 0xFA;
      scene = 10;
    }
    /********************像素十二*********************/
     if ( ((WriteTypeDef.Switch >> 1) & 0x0F) == 0x0B)
    {
      ReadTypeDef.Switch &= 0xF1;
      ReadTypeDef.Switch |= 0xFB;
      scene = 11;
    }
    /********************像素十三*********************/
     if ( ((WriteTypeDef.Switch >> 1) & 0x0F) == 0x0C)
    {
      ReadTypeDef.Switch &= 0xF1;
      ReadTypeDef.Switch |= 0xFC;
      scene = 12;
    }
    /********************场景一*********************/
     if ( ((WriteTypeDef.Switch >> 1) & 0x0F) == 0x0D)
    {
      ReadTypeDef.Switch &= 0xF1;
      ReadTypeDef.Switch |= 0xFD;
      scene = 13;
    }
    //  if ( ((WriteTypeDef.Switch >> 1) & 0x0F) == 0x0E)
    // {
    //   ReadTypeDef.Switch &= 0xF1;
    //   ReadTypeDef.Switch |= 0xFE;
    //   scene = 14;
    // }
    //  if ( ((WriteTypeDef.Switch >> 1) & 0x0F) == 0x0F)
    // {
    //   ReadTypeDef.Switch &= 0xF1;
    //   ReadTypeDef.Switch |= 0xFF;
    //   scene = 15;
    // }
    /********************场景二*********************/
     if ( ((WriteTypeDef.Switch >> 1) & 0x0F) == 0x0E)
    {
      ReadTypeDef.Switch &= 0xF1;
      ReadTypeDef.Switch |= 0xFE;
      scene = 14;
    }
    /********************场景三*********************/
    if ( ((WriteTypeDef.Switch >> 1) &  0x0F) == 0x0F)
    {
      ReadTypeDef.Switch &= 0xF1;
      ReadTypeDef.Switch |= 0xFF;
      scene = 15;
    }
  }

    /***************两个场景*************************/
//    if ( (WriteTypeDef.Attr_Flags1 & (1 << 2)) == (1 << 2))
//   {
//    /********************场景一*********************/
//    if ( ((WriteTypeDef.Switch >> 1) & 0x0F) == 0x0E)
//    {
//      ReadTypeDef.Switch &= 0xF1;
//      ReadTypeDef.Switch |= 0xFE;
//      scene = 14;
//    }
//    /********************场景二*********************/
//    if ( ((WriteTypeDef.Switch >> 1) &  0x0F) == 0x0F)
//    {
//      ReadTypeDef.Switch &= 0xF1;
//      ReadTypeDef.Switch |= 0xFF;
//      scene = 15;
//    }
//   }
  /***************红色****************************/
  if ( (WriteTypeDef.Attr_Flags1 & (1 << 2)) == (1 << 2))
  {
    strip.setPixelColor(scene,WriteTypeDef.Color_R1,ReadTypeDef.Color_G1,ReadTypeDef.Color_B1);
    storeColor[scene] = WriteTypeDef.Color_R1;
    ReadTypeDef.Color_R1 = WriteTypeDef.Color_R1;
  }

  /***************绿色****************************/
  if ( (WriteTypeDef.Attr_Flags1 & (1 << 3)) == (1 << 3))
  {
    strip.setPixelColor(scene,ReadTypeDef.Color_R1,WriteTypeDef.Color_G1,ReadTypeDef.Color_B1);
    storeColor[scene+16] = WriteTypeDef.Color_G1;
    ReadTypeDef.Color_G1 = WriteTypeDef.Color_G1;
  }

  /***************蓝色****************************/
  if ( (WriteTypeDef.Attr_Flags1 & (1 << 4)) == (1 << 4))
  {
    strip.setPixelColor(scene,ReadTypeDef.Color_R1,ReadTypeDef.Color_G1,WriteTypeDef.Color_B1);
    storeColor[scene+32] = WriteTypeDef.Color_B1;
    ReadTypeDef.Color_B1 = WriteTypeDef.Color_B1;
  }
}
/******************色温*************************/
//  if ( (WriteTypeDef.Attr_Flags1 & (1 << 5)) == (1 << 6))
//  {
//    strip.setBrightness(WriteTypeDef.Brightness);
//   strip.setPixelColor(scene,ReadTypeDef.Color_R1,ReadTypeDef.Color_G1,ReadTypeDef.Color_B1);
//    strip.show();
//    ReadTypeDef.Brightness = WriteTypeDef.Brightness;
//  }
//}
  
