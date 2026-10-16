#include <SoftwareSerial.h>
// 软串口：RX不用(255)，TX=D9
SoftwareSerial SoftSerial(255, 11);

//4个按键引脚
const int keyPin[4] = {2, 3, 4, 6};
bool isPress[4] = {false, false, false, false};
unsigned long pressTime[4] = {0, 0, 0, 0};
const unsigned long longPressThresh = 600;  //大于600ms判定长按
const unsigned long holdTimeout = 1500;     //按住超过1500ms，强制重置状态，防卡死

void setup() {
  Serial.begin(9600);
  SoftSerial.begin(9600);
  for(int i = 0; i < 4; i++){
    pinMode(keyPin[i], INPUT_PULLUP);
  }
}

void loop() {
  unsigned long now = millis();

  for(int i = 0; i < 4; i++){
    int val = digitalRead(keyPin[i]);

    // ====== 超时强制复位，解决卡死BUG ======
    if(isPress[i]){
      unsigned long holdDur = now - pressTime[i];
      if(holdDur > holdTimeout){
        isPress[i] = false;
      }
    }

    // 检测：按键按下（下降沿）
    if(val == LOW && !isPress[i]){
      isPress[i] = true;
      pressTime[i] = now;
    }

    // 检测：按键松开（上升沿）
    if(val == HIGH && isPress[i]){
      unsigned long dur = now - pressTime[i];
      byte cmd;
      if(dur >= longPressThresh){
        cmd = i + 5; //长按 5,6,7,8
      }else{
        cmd = i + 1; //短按 1,2,3,4
      }
      //发送二进制指令给主控
      SoftSerial.write(cmd);
      //调试打印（仅电脑串口看，不会发给主控）
      Serial.print("发送指令：");
      Serial.println(cmd);
      //重置状态
      isPress[i] = false;
    }
  }
}