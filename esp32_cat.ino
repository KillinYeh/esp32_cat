#include "initPins.h"     // 腳位與系統模組
#include "LineLib.h"      // Line Notify 自訂模組
#include "DS1302.h"       // DS1302 時鐘模組
#include <Wire.h>
#include <ESP32Servo.h>
#include <Arduino.h>

#define S3_SERIAL        Serial1
#define S3_RX_PIN        37   
#define S3_TX_PIN        36   
#define BAUD_RATE        115200

Servo myservo_cat;
Servo myservo_dog;
int servoPin_cat = 16;
int servoPin_dog = 9 ;
int pos = 0; // 定義舵機轉動位置

bool flag = true;    // 原先用來控制是否已送出 Line 訊息
bool food = false; 
bool hasRotated_cat = false; // 新增旗標，控制舵機只轉動一次
bool hasRotated_dog = false;

int ledPin = 15; 
int trigPin = 11;    // 超音波感測器 Trig 腳接 Arduino pin 11
int echoPin = 12;    // 超音波感測器 Echo 腳接 Arduino pin 12
int speakerpin = 7;  // 蜂鳴器 + 腳接 Arduino pin 7

long duration, cm;   // 宣告計算距離時，需要用到的兩個實數

void initALL();
void setup();
void loop();

void setup()
{
    initALL();       // 系統硬體/軟體初始化
    initWiFi();      // 網路連線
    ShowInternet();  // 顯示網路連線資訊
    initLine();      // 初始化 Line Notify
    S3_SERIAL.begin(BAUD_RATE, SERIAL_8N1, S3_RX_PIN, S3_TX_PIN);
    
    myservo_cat.attach(servoPin_cat);  // 設置舵機控制腳位
    myservo_dog.attach(servoPin_dog);
    
    pinMode(ledPin, OUTPUT);
    delay(1000);
    Serial.begin(115200);        // 設定序列埠監控視窗傳輸速率
    Serial.println("set up");
    pinMode(trigPin, OUTPUT);   // Arduino 對外啟動距離感測器 (Trig)
    pinMode(echoPin, INPUT);    // 讀取超音波回傳 (Echo)
    pinMode(speakerpin, OUTPUT);// 蜂鳴器腳位
}



void loop()
{
    // 1) 取得 RTC 回傳的完整時間字串，例如 "2025-01-03 20:51"
    const char* dataTime = getDataTime();
    
    // 2) 分割出 日期(Date) 與 時間(Time)
    char Date[11], Time[6];
    for(int i = 0; i < 17; i++){
      if (i < 11){
        Date[i] = dataTime[i];
      } else {
        Time[i - 11] = dataTime[i];
      }
    }
    Date[10] = '\0'; 
    Time[5] = '\0';


    // 4) 檢查是否到達指定時間 (20:51)
    if(strcmp(Time, "18:06") == 0) 
    {
      

      // (b) 量測距離
      digitalWrite(trigPin, LOW);
      delayMicroseconds(5);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      duration = pulseIn(echoPin, HIGH);
      cm = (duration / 2.0) / 29.1;  

      Serial.print(cm);
      Serial.println(" cm");

      // (c) 根據距離做對應動作
      if(cm > 20) 
      {
        // 距離大於 20 cm：蜂鳴器斷斷續續響 (0.1s ON / 0.1s OFF)
        digitalWrite(speakerpin, HIGH);
        delay(100);
        digitalWrite(speakerpin, LOW);
        delay(100);

        // 可視需要決定是否要「在大於20時就重置 hasRotated」
        // 如果希望再次從 >20cm -> <=20cm 時舵機能再動，就在這裡重置:
        // hasRotated = false; 
      } else 
      {
        // 距離小於等於 20 cm：先關蜂鳴器
        digitalWrite(speakerpin, LOW);
        if(digitalRead(speakerpin)==LOW)
        {
          Serial.println("[S3] Sending CAPTURE command to CAM...");
          S3_SERIAL.println("CAPTURE");
          unsigned long t0 = millis();
          String result = "1";
          while (millis() - t0 < 5000) 
          { // 最多等3秒
            if (S3_SERIAL.available()) {
                result = S3_SERIAL.readStringUntil('\n');
                result.trim();
                break;
            }
          } 
          Serial.print("[S3] Received result from CAM: ");
          Serial.println(result);
          if(result=="cat")
          {
            // 接著判斷馬達是否已經轉過
            if(!hasRotated_cat)
            {
            // 第一次發生距離 <= 20 cm：執行舵機動作 (轉90度再回0度)
              for(pos = 0; pos <= 90; pos += 1) {
                myservo_cat.write(pos);
                delay(15); 
              } 
              for(pos = 90; pos >= 0; pos -= 1) {
                myservo_cat.write(pos);
                delay(15);
              }
              myservo_cat.detach();
              String msg = "目前date:" + String(Date) + 
                     ", 目前time：" + String(Time) + 
                     " 已投餵貓";
              SendMsgtoLine(msg);
              hasRotated_cat = true; // 記錄舵機已執行過一次
            }
          }
          else if(result=="dog") //dog 
          {
            // 接著判斷馬達是否已經轉過
            if(!hasRotated_dog)
            {
            // 第一次發生距離 <= 20 cm：執行舵機動作 (轉90度再回0度)
              for(pos = 0; pos <= 90; pos += 1) {
                myservo_dog.write(pos);
                delay(15); 
              } 
              for(pos = 90; pos >= 0; pos -= 1) {
                myservo_dog.write(pos);
                delay(15);
              }
              myservo_dog.detach();
              String msg = "目前date:" + String(Date) + 
                     ", 目前time：" + String(Time) + 
                     " 已投餵狗";
              SendMsgtoLine(msg);
              hasRotated_dog = true; // 記錄舵機已執行過一次
            }    
          }
          else
          {
            Serial.println("UNKNOWN");
          }
          }
        }

        
      } 
 /* else {
        // 5) 如果不在 20:51 (代表其他時間或下一天)，就重置旗標
        //    以便明天(或下一次) 20:51 時再次餵食/再次傳訊息
        flag = true;
        hasRotated = false;
  
        // 確保蜂鳴器不會在其他時間繼續響
        digitalWrite(speakerpin, LOW);
      }*/
    }



//--------------------------------------------------------
//  initALL()：系統硬體/軟體初始化
//--------------------------------------------------------
void initALL()
{
    Serial.println("System Start");

    // 時鐘初始化 (DS1302)；你的 DS1302 庫已在 #include "DS1302.h"
    initDS1302 (); 

    // 伺服馬達歸零
    myservo_cat.write(0);
    myservo_dog.write(0);
}
