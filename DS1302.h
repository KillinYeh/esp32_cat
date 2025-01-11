 
// CONNECTIONS:
// DS1302 CLK/SCLK --> 5
// DS1302 DAT/IO --> 4
// DS1302 RST/CE --> 2
// DS1302 VCC --> 3.3v - 5v
// DS1302 GND --> GND
 
#include <ThreeWire.h>  
#include <RtcDS1302.h>
 
ThreeWire myWire(4,5,2); // IO, SCLK, CE  //对应接到ESP32的IO号
RtcDS1302<ThreeWire> Rtc(myWire);
char a[20];

void printDateTime(const RtcDateTime& dt); 

void initDS1302 () 
{
    Serial.begin(57600);
    
    Serial.print("compiled: ");
    Serial.print(__DATE__); //打印系统日期
    Serial.println(__TIME__); //打印系统时间
    
    Rtc.Begin();  //用管脚声明来初始化RTC
 
    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__); //将系统时间声明一个时间结构
    printDateTime(compiled);  //打印时间
    Serial.println();
 
    if (!Rtc.IsDateTimeValid())   //读RTC时间，如果没读到，就是电池没电了
    {
        // Common Causes:
        //    1) first time you ran and the device wasn't running yet
        //    2) the battery on the device is low or even missing
        Serial.println("RTC lost confidence in the DateTime!");
        Rtc.SetDateTime(compiled);    //设置系统时间为当前时间
    }
    if (Rtc.GetIsWriteProtected())    //检查是不是写保护了
    {
        Serial.println("RTC was write protected, enabling writing now");
        Rtc.SetIsWriteProtected(false);
    }
    if (!Rtc.GetIsRunning())  //检查是不是可读
    {
        Serial.println("RTC was not actively running, starting now");
        Rtc.SetIsRunning(true);
    }
    RtcDateTime now = Rtc.GetDateTime();  //得到RTC的时间
    if (now < compiled) //如果读出的时间早于系统时间，则重新设置RTC时间
    {
        Serial.println("RTC is older than compile time!  (Updating DateTime)");
        Rtc.SetDateTime(compiled);
    }
    else if (now > compiled) //如果读出的时间晚于系统时间，则重新设置RTC时间
    {
        Serial.println("RTC is newer than compile time. (this is expected)");
    }
    else if (now == compiled) 
    {
        Serial.println("RTC is the same as compile time! (not expected but all is fine)");
    }
}
const char* getDataTime() 
{
    RtcDateTime now = Rtc.GetDateTime();    //核心函数，读当前RTC时间
    printDateTime(now);
    Serial.println();
    if (!now.IsValid())
    {
        // Common Causes:
        //    1) the battery on the device is low or even missing and the power line was disconnected
        Serial.println("RTC lost confidence in the DateTime!");
    }
    //delay(60000); // ten seconds    //延时10s
    return a;
}
#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime(const RtcDateTime& dt)   //一个通过串口格式化打印日期和时间的函数
{
  char datestring[20];
    snprintf_P(datestring,                //avr的一个打印格式化函数
            countof(datestring),
            PSTR("%04u/%02u/%02u %02u:%02u:%02u"),    //和print类似，指定长度
            dt.Year(),
            dt.Month(),
            dt.Day(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    //Serial.print(datestring);
    strcpy(a, datestring);
}
