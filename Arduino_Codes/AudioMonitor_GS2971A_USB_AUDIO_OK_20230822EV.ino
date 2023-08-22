#include "Arduino.h"
#include "PCF8575.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <EEPROM.h>


#include <TFT_eSPI.h>




//顶部标题信息显示
String str_TopHeader = "AUDIO   GEARBOX   PRO";



//GS2971A SPI数据读取
char SPI_READ_VALUE[48] = {0};


//实例化TFT_eSPI
TFT_eSPI tft  = TFT_eSPI();

//LCD时间显示最大计数复位
#define MAX_TIME_COUNT    86400



//最大音量限制
#define SSM3582A_MAX_OUTPUT_VOLUME   245


//信号数量定义
#define INPUT_SIGNAL_COUNT   5

//信号接口及信号类型定义
String str_AudioSource[INPUT_SIGNAL_COUNT][2] = {
  {"AES", "BNC-1"},
  {"AES", "SFP-1"},
  {"SDI", "BNC-2"},
  {"SDI", "SFP-2"},
  {"USB", "FRONT"}
};








//CS8416复位控制引脚:GPIO3(ESP8285_UART_RX)
//CS8416复位控制引脚:GPIO1(ESP8285_UART_TX)
//CS8416复位控制引脚:GPIO2(ESP8285_SPI_CS)


//*********GS2971与CS8416采用同一复位引脚*********//
#define CS8416_GS2971_RESET_PIN    1










//CS8416锁定状态ESP8285检测GPIO端口(GPIO16)
//设置cs8416寄存器6第4位(UNLOCKM)掩码为1，设置GPIO0输出为RERR，通过CS8416 GPIO0端口输出信号锁定状态
#define CS8416_LOCKED_DETECT_PIN  16
//CS8416上次检测锁定状态
int   last_CS8416_lock_status = 0;
//CS8416锁定状态
int   CS8416_lock_status = 0;


//CS8416 I2C地址
#define  CS8416_I2C_ADDRESS   0x14


//信号输入端口定义
int RX_PORT[INPUT_SIGNAL_COUNT] = {0x00, 0x08, 0x10, 0x10, 0x18};

//信号输出端口定义
int TX_PORT[INPUT_SIGNAL_COUNT] = {0x00, 0x01, 0x02, 0x02, 0x03};


//BNC接口及FC接口AES信号输出源索引
//  0:  BNC接口输入
//  1:  FC光纤输入
//  2:  BNC接口SDI输入音频解嵌输出
//  3:  FC光纤接口SDI输入音频解嵌输出
int   n_AES_TX_Index = 0;

//监听AES信号输入源索引
//  0:  BNC接口输入
//  1:  FC光纤输入
//  2:  BNC接口SDI输入音频解嵌输出
//  3:  FC光纤接口SDI输入音频解嵌输出
int   n_AES_monitor_Index = 2;






#include <PolledTimeout.h>

#define ESP8285_SDA_PIN 4
#define ESP8285_SCL_PIN 5

//ESP8285 I2C地址(可选)
const int16_t ESP8285_I2C_MASTER_ADDRESS = 0x42;





//SSM3582A初始音量设置
int n_SSM3582A_vol = 0x80;

//GS12070 SPI Control SS PIN
int GPIO_SPI_CS = 10;














//LED闪烁间隔
unsigned long LedFlash_Time = {0};
int   LedFlashControl = 0;
int   LedFlashInterval = 500;
int   FlashInterval = 500;










//信息刷新显示间隔
int  infoUpdate_Interval = 1000;
unsigned long infoUpdate_Time = 0;





















//**********串口数据读取**********
char str_ReadCommand[200];
int nIndex = 0;




/*************************************************************/
//TLC5615 SPI控制引脚
const int CS_PIN = 2;   // 片选引脚
const int CLK_PIN = 14;  // 时钟引脚
const int DATA_PIN = 13; // 数据引脚
/*************************************************************/





/*************************************************************/
//旋转编码器变量定义
int encoderPos = 0;
//前一个计数值（-----当计数值未变化时不刷新显示或执行操作-----）
int last_encoderPos = 0;

//编码器状态
int encoderState = 0;
int lastEncoded = 0;

//编码器各引脚状态值
int encoderPinAState;
int encoderPinBState;
int encoderPinSWState;

//编码器SW按钮按下计数
int   n_EC11_Index = 0;



//数值是否改变标志
bool  b_value_changed = false;
/*************************************************************/






/*
  0x18ce  //午夜蓝
  0x22f   //水手蓝
  0x18a   //普鲁士蓝
  0x194
  0x4004    //勃艮第红
  0x5345    //暗橄榄绿
  0x6199    //紫水晶色
  0x49c0    //咖啡色
  0x62ca    //铁灰色
  0x48e0    //椰褐色
  0xcfe0    //亮柠檬色
  0x9806    //紫红
  0xa7f9    //苍色
  0xc618    //银色
  0xf7be    //白烟色
  0xcaa0    //燃橙
  0xff59    //杏仁白
*/

//顶部标题显示背景色
int top_BG = 0x9806; //紫红
//int top_BG = 0xcfe0; //亮柠檬色
int top_FG = TFT_WHITE;
//int top_FG = 0xa7f9; //苍色
//int top_FG = 0xcfe0; //亮柠檬色
uint8_t top_Font = 4;
int top_MC_OFFSET = 3; //文字显示垂直偏移





//频率显示背景色
int freq_BG = 0x48e0; //椰褐色
int freq_FG = 0xc618;  //银色
uint8_t freq_Font = 4;
int freq_MC_OFFSET = 5; //文字显示垂直偏移

//电压显示背景色
int voltage_BG = 0x18a; //普鲁士蓝
//int voltage_FG = TFT_WHITE;
int voltage_FG = 0xc618;  //银色
uint8_t voltage_Font = 4;
int voltage_MC_OFFSET = 3; //文字显示垂直偏移




//信号源选择显示背景色
int audio_BG = 0x18a; //普鲁士蓝
int audio_FG = 0xc618;  //银色
uint8_t audio_Font = 4;
int audio_MC_OFFSET = 3; //文字显示垂直偏移




//电流显示背景色
int amps_BG = 0x4004; //勃艮第红
//int amps_FG = TFT_WHITE;
int amps_FG = 0xc618;  //银色
uint8_t amps_Font = 4;
int amps_MC_OFFSET = 3; //文字显示垂直偏移




//输出功率显示背景色
int fwd_BG = 0x5345;  //暗橄榄绿
//int fwd_FG = 0xcaa0;  //燃橙
int fwd_FG = 0xff59;  //杏仁白
//int fwd_FG = 0xc618;  //银色
//int fwd_FG = 0xf7be;  //白烟色
uint8_t fwd_Font = 4;
int fwd_MC_OFFSET = 8; //文字显示垂直偏移



//运行时间显示背景色
int time_BG = 0x2a69; //暗岩灰
int time_BG2 = 0x11;  //暗蓝
int time_FG = 0xcfe0; //亮柠檬色
uint8_t time_Font = 7;

//时钟刷新定时间隔
unsigned long clock_Time = 0;
int   ClockInterval = 1000;


//系统启动计时
unsigned long n_StartUpTime = 0;



//LCD屏背光设置
int LCD_BLK_VALUE = 140;

//LCD屏幕保护等待时间
unsigned long ScreenSave_Time = 0;
int   ScreenSave_Interval = 30000;



//面板键盘按钮数量定义
#define   KEY_COUNT   6


//按钮消除抖动时间设置
const int DEBOUNCE_DELAY = 50;

//按钮长按时间定义
const int LONG_PRESS_TIME = 2000;




//按钮上一次处于稳定状态时引脚电平
int lastSteadyState[KEY_COUNT + 1] = {HIGH};
//按钮上一次抖动状态时引脚电平
int lastFlickerableState[KEY_COUNT + 1] = {HIGH};
//按钮当前状态引脚电平
int currentState[KEY_COUNT + 1];

//按钮上一次处于抖动状态时间
unsigned long lastDebounceTime[KEY_COUNT + 1] = {0};
//按钮上一次按下时间
unsigned long prevPressTime[KEY_COUNT + 1] = {0};
//当前时间
unsigned long curTime[KEY_COUNT + 1] = {0};









/**********GS2971音频解嵌SPI控制参数**********/
//HD MODE AES OUT CONFIG
//char HD_AES_OUT_CONFIG[4] = {0x02, 0x00, 0x0f, 0x04};
char HD_AES_OUT_CONFIG[4] = {0x02, 0x00, 0x00, 0x00};
//SD MODE AES OUT CONFIG
//char SD_AES_OUT_CONFIG[4] = {0x04, 0x08, 0x00, 0x00};
char SD_AES_OUT_CONFIG[4] = {0x04, 0x08, 0xff, 0x00};

//SATA0-SATA2 PORT MODE SET
//0_XXXXX_XXXXX_XXXXX
//4-0   SATA0_CONFIG
//9-5   SATA1_CONFIG
//14-10 SATA2_CONFIG

//00011:  LOCKED
char SATA_0_1_2_CONFIG[4] = {0x00, 0x08, 0x0c, 0x63};

//SATA3-SATA5 PORT MODE SET
//0_XXXXX_XXXXX_XXXXX
//4-0   SATA3_CONFIG
//9-5   SATA4_CONFIG
//14-10 SATA5_CONFIG

//00011:  LOCKED
char SATA_3_4_5_CONFIG[4] = {0x00, 0x09, 0x0c, 0x63};



//AES OUTPUT CHANNEL CONFIG
//CH1+CH2:  0x208
//CH3+CH4:  0x692
//CH5+CH6:  0xB2C
//CH7+CH8:  0xFBE
//CH1+CH1:  0x000
//CH2+CH2:  0x249
//CH3+CH3:  0x492
//CH4+CH4:  0x6DB
//CH5+CH5:  0x924
//CH6+CH6:  0xB6D
//CH7+CH7:  0xDB6
//CH8+CH8:  0xFFF
char SD_AES_OUT_SELECT_CH1234[4] = {0x04, 0x09, 0x02, 0x08};
char SD_AES_OUT_SELECT_CH5678[4] = {0x04, 0x0A, 0x02, 0x08};

char HD_AES_OUT_SELECT_CH1234[4] = {0x02, 0x0A, 0x02, 0x08};
char HD_AES_OUT_SELECT_CH5678[4] = {0x02, 0x0B, 0x02, 0x08};



char  SPI_CMD[48] = {
  0x80, 0x1F, 0xff, 0xff,
  0x80, 0x20, 0x00, 0x00,
  0x80, 0x21, 0xff, 0xff,
  0x80, 0x22, 0x00, 0x00,
  //0x02, 0x00, 0x0f, 0x04,   //HD AES OUT
  0x02, 0x00, 0x00, 0x00,   //HD AES OUT
  //0x04, 0x08, 0x00, 0x00,   //SD AES OUT
  0x04, 0x08, 0xff, 0x00,   //SD AES OUT(auto 20bit or 24bit)
  0x00, 0x08, 0x0c, 0x63,   //SATA_0_1_2 CONFIG
  0x00, 0x09, 0x0c, 0x63,   //SATA_3_4_5 CONFIG
  0x04, 0x09, 0x02, 0x08,   //SD_AES_OUT_SELECT_CH1234
  0x04, 0x0A, 0x02, 0x08,   //SD_AES_OUT_SELECT_CH5678
  0x02, 0x0A, 0x02, 0x08,   //HD_AES_OUT_SELECT_CH1234
  0x02, 0x0B, 0x02, 0x08,   //HD_AES_OUT_SELECT_CH5678
};



int i_CH = 0;
char CH[12] = {
  0x0000, 0x0249, 0x0492, 0x06db,
  0x0924, 0x0b6d, 0x0db6, 0x0fff,
  0x0208, 0x0692, 0x0b2c, 0x0fbe
};













/***************************2023-08-04******************************/

//SDI子卡GPIO输入输出控制PCF8575 I2C地址
int SDI_CONTROL_PCF8575_I2C_ADDRESS = 0x23;

//SDI子卡GPIO输入输出控制PCF8575实例化
PCF8575 SDI_control_pcf8575(SDI_CONTROL_PCF8575_I2C_ADDRESS);

//GS2971_STAT5状态监测GPIO端口编号
int GS2971_STAT5_STATUS_PORT = 0;
//STAT5端口状态
int GS2971_STAT5_STATUS = 0;

//SFP接收信号中断状态监测GPIO端口编号
int SFP_RX_LOSS_STATUS_PORT = 1;
//RX Loss端口状态
int SFP_RX_LOSS_STATUS = 0;

//LMH0302 SD/HD SLEW控制GPIO端口编号
int LMH0302_SLEW_CONTROL_PORT = 8;

//SFP发送信号允许控制GPIO端口编号
int SFP_TX_ENABLE_PORT = 9;

//GS2971芯片复位控制GPIO端口编号
int GS2971_RESET_CONTROL_PORT = 10;

//RS22228XN SDI输入选择控制GPIO端口编号
int RS2228XN_INPUT_SELECT_PORT = 11;

//GS2971A SPI 控制片选引脚ESP8285 GPIO端口编号
int ESP8285_GS2971_GPIO_SPI_CS = 2;


//SDI子卡PCF8575状态一次性读取
PCF8575::DigitalInput SDI_pcf8575_di;






//AudioMonitor音频功放键盘检测PCF8575 I2C地址
int PCF8575_I2C_ADDRESS = 0x21;

//面板旋转编码器状态监测PCF8575实例化
PCF8575 pcf8575(PCF8575_I2C_ADDRESS);


//按钮数组定义，其中第0位代表旋转编码器按钮，其余6位代表6个键盘按钮
int PCF8575_GPIO_CH[KEY_COUNT + 1] = {P15, P0, P1, P2, P3, P4, P5};

//PCF8575一次性读取
PCF8575::DigitalInput pcf8575_di;







//旋转编码器与PCF8575引脚定义
#define   ENCODER_ROT_A   P13
#define   ENCODER_ROT_B   P14
//#define   ENCODER_SW    P15

//选择编码器状态读取
int ROT_A_State;
int ROT_A_LastState;
int ROT_B_State;
int ROT_SW_State;





//提示信息显示
String  strInfo = "";

//文本内容屏幕显示宽度
int string_W;
//文本内容屏幕显示高度
int string_H;




//LCD屏幕长宽参数
int   LCD_W = 320;
int   LCD_H = 170;

//屏幕显示区域水平间隔
int   H_SPACE = 8;

//屏幕显示区域垂直间隔
int   V_SPACE = 7;


//顶部标题栏高度
int   H_Top = 36;
//顶部标题栏宽度
int   W_Top = LCD_W;

//左侧三列显示高度
int   H_Left = (LCD_H - H_Top - V_SPACE * 3) / 3;

//左侧三列显示宽度
int   W_Left = (LCD_W - H_SPACE) * 38 / 100;

//右侧三列显示高度
int   H_Right = (LCD_H - H_Top - V_SPACE * 2) / 2;

//右侧三列显示宽度
int   W_Right = LCD_W - H_SPACE - W_Left;

















//*********************SPI控制命令发送*********************
void SPI_SEND_CMD(char CMD[], int SPI_delay = 5)
{
  SPI.begin();

  digitalWrite(GPIO_SPI_CS, LOW);
  //delay(5);
  for (int i = 0; i < 6; i++)
  {
    SPI.transfer(CMD[i]);
  }
  digitalWrite(GPIO_SPI_CS, HIGH);
  //delay(SPI_delay);
  delayMicroseconds(500);

  SPI.end();
}



//*********************I2C写数据操作*********************
void I2C_EXECUTE_WRITE_CMD(int i2c_addr, char reg_addr, char command)
{
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  Wire.write(command);
  Wire.endTransmission();
  delayMicroseconds(500);
}



//*********************I2C写数据操作*********************
void I2C_EXECUTE_WRITE_CMD(int i2c_addr, char reg_addr)
{
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  //Wire.read();
  Wire.endTransmission();
  delayMicroseconds(500);
}



//*********************I2C读数据操作*********************
void I2C_EXECUTE_READ_CMD(int i2c_addr)
{
  //Wire.beginTransmission(i2c_addr + 1);
  //Wire.requestFrom(i2c_addr + 1, 1);
  Wire.beginTransmission(i2c_addr);
  Wire.requestFrom(i2c_addr, 1);

  ///2023-02-08 ADD
  Wire.read();
  ///

  Wire.endTransmission();
  delayMicroseconds(500);
}




//*********************向指定地址的I2C器件指定寄存器写入数据*********************
void I2C_SEND_COMMAND(int i2c_addr, char reg_addr, char command)
{
  //STEP:1
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  //Wire.read();
  Wire.endTransmission();
  delayMicroseconds(500);

  //STEP:2
  //Wire.beginTransmission(i2c_addr + 1);
  //Wire.requestFrom(i2c_addr + 1, 1);

  Wire.beginTransmission(i2c_addr);
  Wire.requestFrom(i2c_addr, 1);

  /////////////
  Wire.read();


  Wire.endTransmission();
  delayMicroseconds(500);

  //STEP:3
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  Wire.write(command);
  Wire.endTransmission();
  delayMicroseconds(500);
}





//*********************向指定地址的I2C器件指定寄存器写入数据(数据含掩码)*********************
void I2C_SEND_COMMAND_V2(int i2c_addr, char reg_addr, char command, char mask)
{
  //STEP:1
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  //Wire.read();
  Wire.endTransmission();
  delayMicroseconds(500);

  //STEP:2
  //Wire.beginTransmission(i2c_addr + 1);
  //Wire.requestFrom(i2c_addr + 1, 1);

  Wire.beginTransmission(i2c_addr);
  Wire.requestFrom(i2c_addr, 1);
  Wire.read();

  Wire.endTransmission();
  delayMicroseconds(500);

  //STEP:3
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg_addr);
  Wire.write(command);
  Wire.write(mask);
  Wire.endTransmission();
  delayMicroseconds(500);
}



































/*************************************************************/
//获取系统启动时间并转换为时码格式显示
String GET_RUNING_TIME()
{
  String ss;
  int nSec = ((millis() - n_StartUpTime) / 1000) % MAX_TIME_COUNT;
  int t_s = nSec % 60;

  if (t_s < 10)
  {
    ss = "0" + String(t_s);
  }
  else
  {
    ss = String(t_s);
  }

  return String(nSec / 60) + ":" + ss;
}




/*************************************************************/
void I2C_SCAN()
{
  //I2C DEVICE SCAN
  int nDevices = 0;

  Serial.println("Scanning...");

  for (byte address = 1; address < 127; ++address)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
      {
        Serial.print("0");
      }

      Serial.print(address, HEX);
      Serial.println("  !");

      ++nDevices;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0)
  {
    Serial.println("No I2C devices found\n");
  }
  else
  {
    Serial.println("done\n");
  }
}















/*更新音量输出设置*/
void update_volume(int nVol)
{
  //背景色
  //tft.fillRect(W_Left + V_SPACE, H_Top + V_SPACE, W_Right, H_Right, fwd_BG);
  //tft.fillRect(W_Left + V_SPACE, H_Top + V_SPACE, W_Right, H_Right, fwd_BG);
  //tft.fillRect(W_Left + V_SPACE, H_Top + V_SPACE, W_Right, H_Right, fwd_BG);

  //音量柱前景色
  tft.fillRect(W_Left + V_SPACE, H_Top + V_SPACE, W_Right * nVol / 100, H_Right, TFT_GREEN);

  //背景色
  tft.fillRect(W_Left + V_SPACE + W_Right * nVol / 100, H_Top + V_SPACE, W_Right - W_Right * nVol / 100, H_Right, fwd_BG);

  /*
    tft.setFreeFont(&FreeSansBold18pt7b);
    tft.setTextColor(fwd_FG);
    strInfo = String(nVol) ;
    string_W = tft.textWidth(strInfo);
    string_H = tft.fontHeight();
    tft.drawString(strInfo, LCD_W - string_W - (W_Right - string_W) / 2, H_Top + V_SPACE * 1 + H_Right * 0 + (H_Right - string_H) / 2 + fwd_MC_OFFSET);
  */
}








//左第二行
/*更新选择信号类型显示*/
void update_AudioInput_type(int n_AudioSource_Index)
{
  tft.fillRect(0, H_Top + V_SPACE, W_Left, H_Left, freq_BG);
  tft.fillRect(0, H_Top + V_SPACE, W_Left, H_Left, freq_BG);
  tft.fillRect(0, H_Top + V_SPACE, W_Left, H_Left, freq_BG);

  tft.setTextFont(4);

  strInfo = str_AudioSource[n_AudioSource_Index][0];

  tft.setTextColor(freq_FG);
  string_W = tft.textWidth(strInfo);
  string_H = tft.fontHeight();
  tft.drawString(strInfo, (W_Left - string_W) / 2, H_Top + V_SPACE + (H_Left - string_H) / 2 + freq_MC_OFFSET, 4);
}



//左第三行
/*更新信号输入接口显示*/
void update_AudioInput_connector(int n_AudioSource_Index)
{
  tft.fillRect(0, H_Top + V_SPACE * 2 + H_Left * 1, W_Left, H_Left, audio_BG);
  tft.fillRect(0, H_Top + V_SPACE * 2 + H_Left * 1, W_Left, H_Left, audio_BG);
  tft.fillRect(0, H_Top + V_SPACE * 2 + H_Left * 1, W_Left, H_Left, audio_BG);
  tft.setTextFont(4);
  //tft.setTextColor(audio_FG);
  tft.setTextColor(TFT_GOLD);

  strInfo = str_AudioSource[n_AudioSource_Index][1];

  string_W = tft.textWidth(strInfo);
  string_H = tft.fontHeight();
  tft.drawString(strInfo, (W_Left - string_W) / 2, H_Top + V_SPACE * 2 + H_Left * 1 + (H_Left - string_H) / 2 + audio_MC_OFFSET, 4);
}




//左第四行
/*更新信号锁定状态显示*/
void update_AES_locked_status(int locked)
{
  tft.fillRect(0, H_Top + V_SPACE * 3 + H_Left * 2, W_Left, H_Left, amps_BG);
  tft.setTextFont(4);

  if (locked == 0)
  {
    //信号锁定
    strInfo = "LOCKED";
    tft.setTextColor(TFT_ORANGE);
    //tft.setTextColor(TFT_GREENYELLOW);
  }
  else
  {
    //信号未锁定
    strInfo = "UNLOCK";
    tft.setTextColor(TFT_RED);
  }
  string_W = tft.textWidth(strInfo);
  string_H = tft.fontHeight();
  tft.drawString(strInfo, (W_Left - string_W) / 2, H_Top + V_SPACE * 3 + H_Left * 2 + (H_Left - string_H) / 2 + amps_MC_OFFSET, 4);
}




/*更新顶部标题栏显示*/
void update_TopHeader()
{
  //设置字体
  tft.setTextFont(4);
  //tft.setFreeFont(&FreeSansBold12pt7b);
  //tft.setFreeFont(&FreeMonoBold12pt7b);
  //tft.setFreeFont(&FreeSerifBold12pt7b);

  //顶部局部填充（标题信息显示）
  tft.fillRect(0, 0, W_Top, H_Top, top_BG);

  //顶部标题信息阴影显示
  tft.setTextColor(TFT_BLACK);
  tft.drawString(str_TopHeader, (LCD_W - tft.textWidth(str_TopHeader)) / 2 + 2, (H_Top - tft.fontHeight()) + 2 - top_MC_OFFSET, 4);
  //tft.drawString(str_TopHeader, (LCD_W - tft.textWidth(str_TopHeader)) / 2 + 2, (H_Top - tft.fontHeight()) + 2+top_MC_OFFSET);

  //顶部标题信息显示
  tft.setTextColor(top_FG);
  tft.drawString(str_TopHeader, (LCD_W - tft.textWidth(str_TopHeader)) / 2, (H_Top - tft.fontHeight()) - top_MC_OFFSET, 4);
  //tft.drawString(str_TopHeader, (LCD_W - tft.textWidth(str_TopHeader)) / 2, (H_Top - tft.fontHeight())+top_MC_OFFSET);
}







bool b_chg = true;

/*更新运行时间显示*/
void update_TimeDisplay()
{
  //右第三行局部清屏（运行时间显示）
  //if (b_chg)
  //  {
  tft.fillRect(W_Left + V_SPACE, H_Top + V_SPACE * 2 + H_Right * 1, W_Right, H_Right, time_BG);
  //tft.fillRect(W_Left + V_SPACE, H_Top + V_SPACE * 2 + H_Right * 1, W_Right, H_Right, time_BG);
  //tft.fillRect(W_Left + V_SPACE, H_Top + V_SPACE * 2 + H_Right * 1, W_Right, H_Right, time_BG);
  //  }
  //  else
  //  {
  //    tft.fillRect(W_Left + V_SPACE, H_Top + V_SPACE * 2 + H_Right * 1, W_Right, H_Right, TFT_DARKGREEN);
  //    tft.fillRect(W_Left + V_SPACE, H_Top + V_SPACE * 2 + H_Right * 1, W_Right, H_Right, TFT_DARKGREEN);
  //    tft.fillRect(W_Left + V_SPACE, H_Top + V_SPACE * 2 + H_Right * 1, W_Right, H_Right, TFT_DARKGREEN);
  //  }
  //  b_chg = !b_chg;

  //printf("W_Right=%d   H_Right=%d\r\n", W_Right, H_Right);

  //刷新时间显示
  //时间显示
  tft.setTextFont(7);
  tft.setTextColor(time_FG);

  strInfo = GET_RUNING_TIME();

  //系统启动计时
  string_W = tft.textWidth(strInfo);
  string_H = tft.fontHeight();
  //文字居中显示
  tft.drawString(strInfo, LCD_W - string_W - (W_Right - string_W) / 2, H_Top + V_SPACE * 2 + H_Right * 1 + (H_Right - string_H) / 2, 7);
}




/*************************************************************/
//CS8416初始化
void CS8416_INIT()
{
  //pinMode(CS8416_GS2971_RESET_PIN, OUTPUT);
  digitalWrite(CS8416_GS2971_RESET_PIN, LOW);
  delay(200);
  digitalWrite(CS8416_GS2971_RESET_PIN, HIGH);




  //mute error,RMCK=128Fs
  I2C_SEND_COMMAND(CS8416_I2C_ADDRESS, 0x01, 0x06);



  //De-emphasis设置：  100   100-50us/15us auto
  //GPIO0输出选择：    GPIO0=RERR(0101)
  I2C_SEND_COMMAND(CS8416_I2C_ADDRESS, 0x02, 0x45);
  //I2C_SEND_COMMAND(CS8416_I2C_ADDRESS, 0x02, 0x35);


  //Receiver Error Mask(输入信号未锁定触发RERR信号)
  //bit4:UNLOCKM
  I2C_SEND_COMMAND(CS8416_I2C_ADDRESS, 0x06, 0x10);



  //GPIO1输出设置:  1011   TX
  //GPIO2输出设置:  0000(GND)   1100(VDD)
  I2C_SEND_COMMAND(CS8416_I2C_ADDRESS, 0x03, 0xbc);


  //输出格式设置
  //Master模式
  //64Fs 24bit SODEL=1   SOLRPOL=1
  I2C_SEND_COMMAND(CS8416_I2C_ADDRESS, 0x05, 0x85);
}








//SDI信号输入子卡GPIO输入输入控制PCF8575芯片引脚初始化
void  SDI_CONTROL_GPIO_INIT()
{
  //GS2971_STAT5状态监测GPIO端口编号
  SDI_control_pcf8575.pinMode(GS2971_STAT5_STATUS_PORT, INPUT);

  //SFP接收信号中断状态监测GPIO端口编号
  SDI_control_pcf8575.pinMode(SFP_RX_LOSS_STATUS_PORT, INPUT);

  //LMH0302 SD/HD SLEW控制GPIO端口编号
  SDI_control_pcf8575.pinMode(LMH0302_SLEW_CONTROL_PORT, OUTPUT);

  //SFP发送信号允许控制GPIO端口编号
  SDI_control_pcf8575.pinMode(SFP_TX_ENABLE_PORT, OUTPUT);

  //GS2971芯片复位控制GPIO端口编号
  SDI_control_pcf8575.pinMode(GS2971_RESET_CONTROL_PORT, OUTPUT);

  //RS22228XN SDI输入选择控制GPIO端口编号
  SDI_control_pcf8575.pinMode(RS2228XN_INPUT_SELECT_PORT, OUTPUT);


  //PCF8575端口模式设置延迟
  delay(100);


  //读取PCF8575输入端口状态
  SDI_pcf8575_di = SDI_control_pcf8575.digitalReadAll();

  //GS2971信号锁定状态
  GS2971_STAT5_STATUS = SDI_pcf8575_di.p0;

  //SFP接收信号中断RX Loss端口状态
  SFP_RX_LOSS_STATUS = SDI_pcf8575_di.p1;



  //控制端口状态设置
  //LMH0302 SD/HD SLEW控制(low:HD/3G  high:SD)
  SDI_control_pcf8575.digitalWrite(LMH0302_SLEW_CONTROL_PORT, LOW);

  //SFP发送信号允许控制GPIO端口编号
  SDI_control_pcf8575.digitalWrite(SFP_TX_ENABLE_PORT, LOW);

  //GS2971芯片复位
  SDI_control_pcf8575.digitalWrite(GS2971_RESET_CONTROL_PORT, LOW);
  delay(300);
  SDI_control_pcf8575.digitalWrite(GS2971_RESET_CONTROL_PORT, HIGH);



  //RS22228XN SDI输入选择控制GPIO端口
  //  HIGH:   选择SFP输入
  //  LOW:    选择BNC输入
  if (n_AES_monitor_Index == 2)
  {
    SDI_control_pcf8575.digitalWrite(RS2228XN_INPUT_SELECT_PORT, LOW);
  }
  else
  {
    SDI_control_pcf8575.digitalWrite(RS2228XN_INPUT_SELECT_PORT, HIGH);
  }
}













//GS2971A芯片SDI音频解嵌初始化设置
void GS2971A_INIT()
{
  digitalWrite(0, HIGH);
  digitalWrite(ESP8285_GS2971_GPIO_SPI_CS, LOW);
  delayMicroseconds(20);

  //HD MODE AES OUT CONFIG
  for (int i = 0; i < 4; i++)
  {
    SPI.transfer(HD_AES_OUT_CONFIG[i]);
  }

  //SD MODE AES OUT CONFIG
  for (int i = 0; i < 4; i++)
  {
    SPI.transfer(SD_AES_OUT_CONFIG[i]);
  }


  for (int i = 0; i < 4; i++)
  {
    SPI.transfer(SD_AES_OUT_SELECT_CH1234[i]);
  }

  for (int i = 0; i < 4; i++)
  {
    SPI.transfer(SD_AES_OUT_SELECT_CH5678[i]);
  }

  for (int i = 0; i < 4; i++)
  {
    SPI.transfer(HD_AES_OUT_SELECT_CH1234[i]);
  }

  for (int i = 0; i < 4; i++)
  {
    SPI.transfer(HD_AES_OUT_SELECT_CH5678[i]);
  }


  //SATA0-SATA2 PORT MODE SET
  for (int i = 0; i < 4; i++)
  {
    SPI.transfer(SATA_0_1_2_CONFIG[i]);
  }

  //SATA3-SATA5 PORT MODE SET
  for (int i = 0; i < 4; i++)
  {
    SPI.transfer(SATA_3_4_5_CONFIG[i]);
  }




  //SPI.end();
  delayMicroseconds(5);
  digitalWrite(ESP8285_GS2971_GPIO_SPI_CS, HIGH);

  digitalWrite(0, LOW);
}













/*************************************************************/
void setup()
{
  //Serial.begin(9600, SERIAL_8N1);

  EEPROM.begin(10);
  //读取EEPROM通道配置信息
  n_AES_monitor_Index = (byte)(EEPROM.read(0));
  if ((n_AES_monitor_Index > (INPUT_SIGNAL_COUNT - 1)) || (n_AES_monitor_Index < 0))
  {
    //配置信息错误，设置为默认值
    n_AES_monitor_Index = 0;
  }



  //CS8416及GS2971复位控制引脚初始化
  pinMode(CS8416_GS2971_RESET_PIN, OUTPUT);


  //GS2971A SPI 控制片选引脚ESP8285 GPIO端口
  pinMode(ESP8285_GS2971_GPIO_SPI_CS, OUTPUT);


  //CS8416锁定状态ESP8285检测GPIO端口(GPIO16)
  //设置cs8416寄存器6第4位(UNLOCKM)掩码为1，设置GPIO0输出为RERR，通过CS8416 GPIO0端口输出信号锁定状态
  pinMode(CS8416_LOCKED_DETECT_PIN, INPUT);




  //ESP8285 SPI初始化
  //SPI总线参数设置
  //  SPI.setClockDivider(SPI_CLOCK_DIV128);
  //  SPI.setDataMode(SPI_MODE0);
  //  SPI.setBitOrder(MSBFIRST);
  SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));
  SPI.begin();


  /********************GS2971A芯片SDI音频解嵌初始化设置********************/
  //GS2971A_INIT();









  //ESP8285 I2C初始化
  Wire.begin(ESP8285_SDA_PIN, ESP8285_SCL_PIN, ESP8285_I2C_MASTER_ADDRESS);
  delay(50);


  //CS8416上次检测锁定状态
  last_CS8416_lock_status = digitalRead(CS8416_LOCKED_DETECT_PIN);


  //SCAN I2C DEVICE
  //I2C_SCAN();




  //SDI信号输入子卡GPIO输入输入控制PCF8575芯片引脚初始化
  SDI_CONTROL_GPIO_INIT();













  /**************************SSM3582A启动**************************/
  I2C_SEND_COMMAND(0x10, 0x04, 0x20);

  //SSM3582A初始音量设置
  //旋转编码器变量定义
  encoderPos = 40;

  //最大音量限制
  n_SSM3582A_vol = 255 - encoderPos * SSM3582A_MAX_OUTPUT_VOLUME / 100;
  I2C_SEND_COMMAND(0x10, 0x07, n_SSM3582A_vol);
  I2C_SEND_COMMAND(0x10, 0x08, n_SSM3582A_vol);














  /***********************************************/
  //TFT_eSPI初始化
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.fillScreen(TFT_BLACK);

  //关闭TFT文本自动换行
  tft.setTextWrap(false, false);
  tft.setTextDatum(TL_DATUM);

  //调整LCD背光亮度
  analogWrite(TFT_BL, LCD_BLK_VALUE);


  int LED_FLASH_DELAY = 50;

  tft.fillScreen(TFT_RED);
  delay(LED_FLASH_DELAY * 10);

  tft.fillScreen(TFT_GREEN);
  delay(LED_FLASH_DELAY * 10);

  tft.fillScreen(TFT_BLUE);
  delay(LED_FLASH_DELAY * 10);



  //整体清屏
  tft.fillScreen(TFT_BLACK);
  tft.fillScreen(TFT_BLACK);
  tft.fillScreen(TFT_BLACK);


  /*更新顶部标题栏显示*/
  update_TopHeader();



  //左四行
  /*更新信号锁定状态显示*/
  update_AES_locked_status(last_CS8416_lock_status);


  //右第二行 输出音量设置
  update_volume(encoderPos);

  //右第三行（运行时间显示）
  tft.fillRect(W_Left + V_SPACE, H_Top + V_SPACE * 2 + H_Right * 1, W_Right, H_Right, time_BG);






  /***********************************************/
  //PCF8575按键读取初始化
  pcf8575.begin();

  //设置PCF8575键盘按钮端口工作模式
  for (int i = 0; i <= KEY_COUNT; i++)
  {
    pcf8575.pinMode(PCF8575_GPIO_CH[i], INPUT);
  }

  //设置PCF8575旋转编码器端口工作模式
  pcf8575.pinMode(ENCODER_ROT_A, INPUT);
  pcf8575.pinMode(ENCODER_ROT_B, INPUT);

  //PCF8575端口模式设置延迟
  delay(100);

  //读取PCF8575输入端口状态
  pcf8575_di = pcf8575.digitalReadAll();
  currentState[0] = pcf8575_di.p15;
  currentState[1] = pcf8575_di.p0;
  currentState[2] = pcf8575_di.p1;
  currentState[3] = pcf8575_di.p2;
  currentState[4] = pcf8575_di.p3;
  currentState[5] = pcf8575_di.p4;
  currentState[6] = pcf8575_di.p5;

  for (int i = 0; i <= KEY_COUNT; i++)
  {
    //currentState[i] = pcf8575.digitalRead(PCF8575_GPIO_CH[i]);
    lastFlickerableState[i] = currentState[i];
    lastSteadyState[i] = currentState[i];
    //printf("CH[%d]  init status=%d\r\n", i, currentState[i]);
  }







  //时钟刷新定时间隔
  clock_Time = millis();

  //系统启动计时
  n_StartUpTime = millis();

  //LED闪烁间隔
  LedFlash_Time = millis();

  //信息刷新显示间隔
  infoUpdate_Time = millis();

  //屏幕保护等待时间
  ScreenSave_Time = millis();

  //CS8416初始化
  CS8416_INIT();

  //CS8416输入输出信号源选择设置更新
  CS8416_TX_RX_UPDATE();

}




















/***********串口通信，字符转数值*********************************/
int ch2num(char x)
{
  if (x > 'A')
    return int(x - 55);
  else
    return int(x - 48);
}




/*************************************************************/
//LCD液晶屏屏幕保护
void LCD_ScreenSave(bool b_screenSave)
{
  if (b_screenSave)
  {
    analogWrite(TFT_BL, 5);
  }
  else
  {
    analogWrite(TFT_BL, LCD_BLK_VALUE);

    //重新设置屏幕保护计时
    ScreenSave_Time = millis();
  }
}





/*************************************************************/
//CS8416输入输出信号源选择设置更新
void  CS8416_TX_RX_UPDATE()
{
  //  n_AES_monitor_Index:  监听AES信号输入源索引
  //  0:  BNC接口输入
  //  1:  FC光纤输入
  //  2:  BNC接口SDI输入音频解嵌输出
  //  3:  FC光纤接口SDI输入音频解嵌输出
  //  4:  CM108AH USB AUDIO

  //  n_AES_TX_Index:  环串输出信号源索引
  //  0:  BNC接口输入
  //  1:  FC光纤输入
  //  2:  BNC接口SDI输入音频解嵌输出
  //  3:  FC光纤接口SDI输入音频解嵌输出
  //  4:  CM108AH USB AUDIO

  if (n_AES_monitor_Index >= 2)
  {
    //设置SDI音频解嵌源（BNC SDI输入或SFP模块输入）
    //设置音频解嵌通道

    if (n_AES_monitor_Index == 2)
    {
      //RS22228XN SDI输入选择控制GPIO端口
      //  HIGH:   选择SFP输入
      //  LOW:    选择BNC输入
      SDI_control_pcf8575.digitalWrite(RS2228XN_INPUT_SELECT_PORT, LOW);
    }


    if (n_AES_monitor_Index == 3)
    {
      //RS22228XN SDI输入选择控制GPIO端口
      //  HIGH:   选择SFP输入
      //  LOW:    选择BNC输入
      SDI_control_pcf8575.digitalWrite(RS2228XN_INPUT_SELECT_PORT, HIGH);
    }




  }


  //左第二行
  /*更新选择信号类型显示*/
  update_AudioInput_type(n_AES_monitor_Index);

  //左第三行（音频输入信号接口显示）
  /*更新音频输入信号接口显示*/
  update_AudioInput_connector(n_AES_monitor_Index);


  //printf("monitor=%2d  TX=%d  0x%X\r\n", n_AES_monitor_Index, n_AES_TX_Index, 0x80 + RX_PORT[n_AES_monitor_Index] + TX_PORT[n_AES_TX_Index]);

  I2C_SEND_COMMAND(CS8416_I2C_ADDRESS, 0x04, 0x80 + RX_PORT[n_AES_monitor_Index] + TX_PORT[n_AES_TX_Index]);
}














/*************************************************************/
void loop()
{
  //背光变暗等待时间
  if ((millis() - ScreenSave_Time) > ScreenSave_Interval)
  {
    //屏幕保护时间达到
    //进入屏幕保护模式
    LCD_ScreenSave(true);
  }




  //时钟刷新定时间隔
  if ((millis() - clock_Time) > ClockInterval)
  {
    clock_Time = millis();

    /*更新运行时间显示*/
    update_TimeDisplay();


    //GS2971A芯片SDI音频解嵌设置
    GS2971A_INIT();
  }






  //CS8416锁定状态ESP8285检测GPIO端口(GPIO16)
  //设置cs8416寄存器6第4位(UNLOCKM)掩码为1，设置GPIO0输出为RERR，通过CS8416 GPIO0端口输出信号锁定状态

  //CS8416锁定状态
  CS8416_lock_status = digitalRead(CS8416_LOCKED_DETECT_PIN);
  if (CS8416_lock_status != last_CS8416_lock_status)
  {
    last_CS8416_lock_status = CS8416_lock_status;

    /*更新信号锁定状态显示*/
    update_AES_locked_status(CS8416_lock_status);
  }

 







  pcf8575_di = pcf8575.digitalReadAll();
  encoderPinAState = pcf8575_di.p13;
  encoderPinBState = pcf8575_di.p14;

  //更新状态机
  if (encoderPinAState != lastEncoded)
  {
    //旋转编码器启动
    //取消屏幕保护
    LCD_ScreenSave(false);


    if (encoderPinBState != encoderPinAState)
    {
      encoderState++;
    }
    else
    {
      encoderState--;
    }

    lastEncoded = encoderPinAState;

    //根据状态机的状态变化更新旋转编码器位置
    if (encoderState == 2 || encoderState == -2)
    {
      if (encoderPinBState == HIGH)
      {
        encoderPos -= 1;
        if (encoderPos <= 0)
        {
          encoderPos = 0;
        }
      }
      else
      {
        encoderPos += 1;
        if (encoderPos >= 100)
        {
          encoderPos = 100;
        }
      }

      encoderState = 0;


      //输出当前位置值
      Serial.println(encoderPos);

      if (encoderPos != last_encoderPos)
      {
        //------值未更改的情况下不刷新显示，以减少画面刷新抖动-------
        update_volume(encoderPos);
      }







      //更改SSM3582A音量设置
      //最大音量限制
      n_SSM3582A_vol = 255 - encoderPos * SSM3582A_MAX_OUTPUT_VOLUME / 100;
      I2C_SEND_COMMAND(0x10, 0x07, n_SSM3582A_vol);
      I2C_SEND_COMMAND(0x10, 0x08, n_SSM3582A_vol);


      //保存当前计数值
      last_encoderPos = encoderPos;
    }
  }




  currentState[0] = pcf8575_di.p15;
  currentState[1] = pcf8575_di.p0;
  currentState[2] = pcf8575_di.p1;
  currentState[3] = pcf8575_di.p2;
  currentState[4] = pcf8575_di.p3;
  currentState[5] = pcf8575_di.p4;
  currentState[6] = pcf8575_di.p5;

  for (int p = 0; p <= KEY_COUNT; p++)
  {
    curTime[p] = millis();
    //currentState[p] = pcf8575.digitalRead(PCF8575_GPIO_CH[p]);
    if (currentState[p] != lastFlickerableState[p])
    {
      //printf("CH[%d]  state changed!  CURRENT=%d\r\n", p, currentState[p]);
      lastDebounceTime[p] = curTime[p];
      lastFlickerableState[p] = currentState[p];
    }

    //去抖动判断
    if ((curTime[p] - lastDebounceTime[p]) > DEBOUNCE_DELAY)
    {
      //超过去抖动时间设定
      if (lastSteadyState[p] == LOW && currentState[p] == HIGH)
      {
        //按钮释放

        if ((curTime[p] - prevPressTime[p]) >= LONG_PRESS_TIME)
        {
          //按钮长按时间超过5秒，保存配置
          EEPROM.begin(10);

          //保存当前选择通道信息到EEPROM
          EEPROM.write(0, byte(n_AES_monitor_Index));
          EEPROM.commit();

          //重新启动
          ESP.restart();
        }


        //检测到按钮按下
        //取消屏幕保护
        LCD_ScreenSave(false);

        //current short press released:
        //execute key click command
        //printf("KEY=%02d\r\n", p);
        switch (p)
        {
          case 0:
            {
              //旋转编码器按钮按下
              //编码器SW按钮按下计数

              //  n_AES_monitor_Index:  监听AES信号输入源索引
              //  0:  BNC接口输入
              //  1:  FC光纤输入
              //  2:  BNC接口SDI输入音频解嵌输出
              //  3:  FC光纤接口SDI输入音频解嵌输出
              n_AES_monitor_Index = (n_AES_monitor_Index + 1) % INPUT_SIGNAL_COUNT;

              //  n_AES_TX_Index:  环串输出信号源索引
              //  0:  BNC接口输入
              //  1:  FC光纤输入
              //  2:  BNC接口SDI输入音频解嵌输出
              //  3:  FC光纤接口SDI输入音频解嵌输出

              //监听联动输出
              n_AES_TX_Index = n_AES_monitor_Index;

              //执行CS8416输入输出信号源选择设置更新
              CS8416_TX_RX_UPDATE();

            }
            break;


          default:
            break;
        }
      }
      else
      {
        if (lastSteadyState[p] == HIGH && currentState[p] == LOW)
        {
          //按钮按下
          //保存按下时间
          prevPressTime[p] = curTime[p];
        }
      }
      lastSteadyState[p] = currentState[p];
    }
  }




  //delay for i2c operation
  delayMicroseconds(500);
}
