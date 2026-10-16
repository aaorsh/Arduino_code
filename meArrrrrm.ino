#include <Servo.h>
#include<math.h>
#include<SoftwareSerial.h>

SoftwareSerial SoftSerial(0,1);


#define joy1_X A2
#define joy1_Y A1
#define joy2_X A0
#define joy2_Y A3

#define clawPin 6
#define rArmPin 7
#define fArmPin 8
#define basePin 9

#define neichun 32



#define PI_F 3.141592653589793f//弧度转角度

//任务三的按钮引脚设置
// #define KEY1_PIN 12
// #define KEY2_PIN 13
// #define KEY3_PIN A4
// #define KEY4_PIN A5

//按钮1功能：单击-录制动作/长按-ABC循环播放
//按钮2功能：单击-播放动作/长按-回中
//按钮3功能：单击-示教点存储/长按-直线曲线切换
//按钮4功能：单击-自动绘图/长按取消自动绘图


#define STEP_MS 20
#define DEAD_THRESHOLD 1.5f


int playIdx =0;
int interStepTotal =0;
int interpNowStep =0;
int pFrom_b,pFrom_f,pFrom_r,pFrom_c;
int pTo_b,pTo_f,pTo_r,pTo_c;

unsigned long interpLastTime;

unsigned long recordLastTime;
unsigned long playLastTime;

Servo base,fArm,rArm,claw;


const int angleopen = 40;
const int angleclose = 130;
const int basepos = 90;
const int fArmpos = 90;
const int rArmpos = 90;
const int xiaodoushichang = 20;
const float DRAW_STEP_MS = 30.0f;
unsigned long drawLastTime = 0;


const int MIN_speedstep = 1;
const int MAX_speedstep = 5;
const int baseMax = 180;
const int baseMin = 0;
const int fArmMax = 160;
const int fArmMin = 0;
const int rArmMax = 180;
const int rArmMin = 10;
const int clawMax = 130;
const int clawMin = 30;


const int MID = 512;
const int deadzone = 35;
const int speedMin = 1;
const int speedmax = 2;

const int zhenZhenshu = 200;

const float L1 = 80.0f;
const float L2 = 80.0f;
const float L3 = 60.0f;
const float L4 = 70.0f;

//舵机零点偏移：推导得到的理论角度 → 实际舵机物理零点补偿，实物调试修改
const float OFFSET_ALPHA = 0.0f;
const float OFFSET_BETA  = 0.0f;
const float OFFSET_GAMMA = 0.0f;

//舵机行程限制，防止撞机
const float SERVO_MIN_ANGLE = 0.0f;
const float SERVO_MAX_ANGLE = 180.0f;

struct ArmAngles
{
    float alpha;    //base舵机理论角度 
    float beta;     //fArm舵机理论角度 
    float gamma;    //rArm舵机理论角度 
    bool  valid;    //true解有效；false无解、奇异点、超出工作空间
};

struct ArmPoint
{
    float x;    //水平面X mm
    float y;    //水平面Y mm
    float H;    //A点离地高度 mm
    bool valid;
}fpt;

float deg2rad(float deg)
{
    return deg * PI / 180.0f;
}

float rad2deg(float rad)
{
    return rad * 180.0f / PI;
}


const int YaoGanTingDun = 8;

int step;

int basetarget;
int fArmtarget;
int rArmtarget;
int clawtarget;

int baseoff;
int fArmoff;
int rArmoff;
int clawoff;
int * basetargetp = &basetarget;
int * fArmtargetp = &fArmtarget;
int * rArmtargetp = &rArmtarget;
int * clawtargetp = &clawtarget;


char Scmd[neichun];
char shuzi[3][4]{};
int Index =0;
int xyz[3];

bool last[4];
bool now[4];
bool jianche[8];

unsigned long StartTime;
unsigned long EndTime;
bool sd;

unsigned long b;
unsigned long f;
unsigned long r;
unsigned long c;

int real_b,real_f,real_r,real_c;
float acc_b,acc_f,acc_r,acc_c;



const int seqA[][4] = {
  {120,20,110,40},
  {120,20,110,130},
  {120,90,90,130},
  {60,90,90,130},
  {60,20,120,130},
  {60,20,120,40}
};

int lena = sizeof(seqA)/sizeof(seqA[0]);

const int seqB[][4] = {
  {90,20,110,40},
  {90,20,110,130},
  {90,90,90,130},
  {100,90,90,130},
  {100,20,120,130},
  {100,20,120,40}
};

int lenb = sizeof(seqB)/sizeof(seqB[0]);

const int seqC[][4] ={
  {30,60,100,40},
  {30,60,100,130},
  {30,90,90,130},
  {90,90,90,130},
  {90,60,100,130},
  {90,60,100,40}
};

int lenc = sizeof(seqC)/sizeof(seqC[0]);

unsigned long LastTime =0;
int taskCount =0;

const int MAX_FRAME = 100;//最大存储空间
int zhenshu2;//录制帧数
int ipzhenshu;//播放的最大帧数

int * ipzhenshup = &ipzhenshu;
int * interpNowStepp = &interpNowStep;
int * playIdxp = &playIdx;


bool zhuangtai2;//录制状态
bool zhuangtai3;//播放状态
int luzhi[MAX_FRAME][4];//存储数组

bool * zhuangtai3p = &zhuangtai3;

//注意是让nowTime = millis
unsigned long nowTime;

bool zhixiantai;
int TeachpointCount;

bool jisuanC;
bool qidong;

bool recordStopFlag = false;
unsigned long playTime;
int waitFrame;

int drawIndex =0;
float currentT =0;
float stepT=0.05;
int16_t x[5];
int16_t y[5];

float Xnow;
float Ynow;

// float x_out;
// float y_out;

float base_out;
float fArm_out;
float rArm_out;

float alpha_in;
float beta_in;
float gamma_in;

float normalizeDeg(float deg)
{
    deg = fmod(deg, 360.0f);
    if(deg > 180.0f)  deg -= 360.0f;
    if(deg < -180.0f) deg += 360.0f;
    return deg;
}

float mapServoAngle(float theoryDeg, float offset)
{
    float real = theoryDeg + offset;
    real = constrain(real, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    return real;
}


void MoveTo(Servo &s,int target,int &step)
{
  int now = s.read();
  if(now >= target){
    for(int i =now;i>target;i-=step){
      s.write(i);
      // unsigned long Time = millis();
      // while(millis() - Time < 10){
      //   continue;
      // }

      // Serial.println(i);
      delay(10);
    }
  }
  else{
    for(int i = now;i<target;i+=step){
      s.write(i);
      unsigned long Time = millis();
      while(millis() - Time < 10){
        continue;
      }
      // Serial.println(i);
      delay(10);
    }
  }
}

void cmdI(char cmd)
{
  switch(cmd){
      case 'O':
        Serial.print(F("command:"));
        Serial.println(F("O"));
        MoveTo(claw,angleopen,step);
        break;
      case 'S':
        Serial.print(F("command:"));
        Serial.println(F("S"));
        MoveTo(claw,angleclose,step);
        break;
      case 'H':
        if(step >=MAX_speedstep){
        Serial.println(F("Speed above maximum!"));
        }
        else{
        step++;
        Serial.print(F("Speed level:"));
        Serial.println(step);
        }
        break;
      case 'L':
        if(step <=MIN_speedstep){
        Serial.println(F("Speed below minimum!"));
        }
        else{
        step--;
        Serial.print(F("Speed level:"));
        Serial.println(step);
        }
        break;
      default :
        Serial.println(F("Unrecognized command!"));
        break;
    }
}

void cmdtwo(char cmd,int data,int &step)
{
  switch(cmd){
    case 'b':
    MoveTo(base,data,step);
    break;

    case 'f':
    MoveTo(fArm,data,step);
    break;

    case 'r':
    MoveTo(rArm,data,step);
    break;

    case 'c':
    MoveTo(claw,data,step);
    break;
    default:
    Serial.println(F("Unrecognized command!"));
    break;
  }
}

void yaogan(int xxxoff,int  *xxxtarget,Servo &s,int min,int max)
{
  if(abs(xxxoff) > deadzone){
    int xxxCZ;
    xxxCZ = map(abs(xxxoff),deadzone,512,speedMin,speedmax);

    xxxCZ = (xxxoff > 0) ? -xxxCZ : xxxCZ;
  
    *xxxtarget += xxxCZ;

    *xxxtarget = constrain(*xxxtarget,min,max);

    s.write(*xxxtarget);

  }
}

void xyzv(char * Scmd)
{
  int pos =0;

    for(int i =0;i<3;i++){
        while(Scmd[pos]==','){
            pos++;
        }

        pos++;

        int j =0;

        while(Scmd[pos]>='0' && Scmd[pos]<='9'){
            shuzi[i][j] = Scmd[pos];
            j++;
            pos++;
        }
        shuzi[i][j] = '\0';

        if(Scmd[pos]>='0' && Scmd[pos]<='9'){
            pos++;
        }

        xyz[i] = atoi(shuzi[i]);
        xyz[i] = constrain(xyz[i],0,180);

    }

    
    base.write(xyz[0]);
    rArm.write(xyz[1]);
    fArm.write(xyz[2]);

    //待优化

    
    
}

void moveto3(int b,int f,int r,int c)
{
  MoveTo(base,b,step);
  MoveTo(fArm,f,step);
  MoveTo(rArm,r,step);
  MoveTo(claw,c,step);
}

void run(int seq[][4],int len)
{
  for(int i=0;i<len;i++)
  {
    int b = seq[i][0];
    int f = seq[i][1];
    int r = seq[i][2];
    int c = seq[i][3];

    moveto3(b,f,r,c);
    delay(800);
  }
}

void tasktwo(char first){
  switch(first){
    case 'A':
    run(seqA,lena);
    break;
    case 'B':
    run(seqB,lenb);
    break;
    case 'C':
    run(seqC,lenc);
    break;
    default:
    Serial.println(F("Unrecognized command!"));
    break;
  }
}

void cmdall(char * str)
{
  int len = strlen(str);
  if(len == 0){
    return;
  }

  char first = str[0];

  

  if(first == 'O' || first == 'S' || first == 'H' || first == 'L'){
    cmdI(first);
  }
  else if(first == 'b' || first == 'f' || first == 'r' || first == 'c'){
    int data = atoi(&str[1]);
    cmdtwo(first,data,step);
  }
  else if(first == 'x'){
    xyzv(str);
  }
  else if(first == 'A' || first == 'B' ||first == 'C'){
    tasktwo(first);
  }
  else if(first == '1' || first == '2' || first == '3' || first == '4'){
    shan(first);

  }
  else{
    Serial.println(F("Unrecognized command!"));
   }

}

void shan(char first){

  //按钮0/4功能：单击-录制动作/长按-ABC循环播放
  //按钮1/5功能：单击-播放动作/长按-回中
  //按钮2/6功能：单击-示教点存储/长按-直线曲线切换
  //按钮3/7功能：单击-自动绘图/长按取消自动绘图

  switch(first){
    case '1':
    Serial.println(F("单击按钮按下"));
    jianche[2] = true;
    EndTime = StartTime =0;

    break;
    case '2':
    Serial.println(F("单击按钮按下"));
    jianche[3] = true;
    EndTime = StartTime =0;

    break;
    case '3':
    Serial.println(F("长按按钮按下"));
    jianche[6] = true;
    EndTime = StartTime =0;


    break;

    case '4':
    Serial.println(F("长按按钮按下"));
    jianche[7] = true;
    EndTime = StartTime =0;

    break;
    
  }
}

void anjian4(){

  MoveTo(base,90,step);
  MoveTo(fArm,90,step);
  MoveTo(rArm,90,step);

}

void anjian1(){
  switch(taskCount){
    case 0:
    run(seqA,lena);
    break;
    case 1:
    run(seqB,lenb);
    break;
    case 2:
    run(seqC,lenc);
    break;
  }
}

void anjian2(int b,int f,int r,int c,int * ipzhenshup){
        //读入数据并传进数组中
        //需要写帧数这样的时间设置+播放逻辑
        *ipzhenshup = zhenshu2;
       
        if(zhenshu2 < MAX_FRAME){
            luzhi[zhenshu2][0] = b;
            luzhi[zhenshu2][1] = f;
            luzhi[zhenshu2][2] = r;
            luzhi[zhenshu2][3] = c;
            zhenshu2++;
        }
        else{
          
          // Serial.print(F("ipzhenshu:"));Serial.println(ipzhenshu);

          Serial.println(F("<--已达到最长录制时间!-->"));
        }
}

void anjian3(int * interpNowStepp,int * playIdxp,bool * zhuangtai3p){
  // Serial.println(F("进入按键3"));
    if(*playIdxp >=ipzhenshu -1){
        //Serial.println(F("<--已结束播放-->")); 
        // zhuangtai3 = false;
       
        *playIdxp=0;
        *interpNowStepp=0;
        *zhuangtai3p = false;
        return;
    }else{
      if(*interpNowStepp ==0){

        pFrom_b = luzhi[*playIdxp][0];
      pFrom_f = luzhi[*playIdxp][1];
      pFrom_r = luzhi[*playIdxp][2];
      pFrom_c = luzhi[*playIdxp][3];

      pTo_b = luzhi[*playIdxp+1][0];
      pTo_f = luzhi[*playIdxp+1][1];
      pTo_r = luzhi[*playIdxp+1][2];
      pTo_c = luzhi[*playIdxp+1][3];

        interStepTotal = zhenZhenshu / STEP_MS;
        if(interStepTotal < 1)interStepTotal =1;

        real_b = pFrom_b;
        real_f = pFrom_f;
        real_r = pFrom_r;
        real_c = pFrom_c;

        acc_b =0;
        acc_f =0;
        acc_r =0;
        acc_c =0;
        waitFrame =0;

         
        
      }

       

    float t = (float)*interpNowStepp / (float)interStepTotal;
    float ratio = t * t * (3 - 2 * t);
    float cur_b = pFrom_b + (pTo_b - pFrom_b) * ratio;
    float cur_f = pFrom_f + (pTo_f - pFrom_f) * ratio;
    float cur_r = pFrom_r + (pTo_r - pFrom_r) * ratio;
    float cur_c = pFrom_c + (pTo_c - pFrom_c) * ratio;

    acc_b += cur_b - real_b;
    acc_f += cur_f - real_f;
    acc_r += cur_r - real_r;
    acc_c += cur_c - real_c;

    waitFrame++;

    bool needsend = false;
    if(fabs(acc_b) >= DEAD_THRESHOLD || 
       fabs(acc_f) >= DEAD_THRESHOLD ||
       fabs(acc_r) >= DEAD_THRESHOLD ||
       fabs(acc_c) >= DEAD_THRESHOLD ||
       waitFrame >= 2 ){ needsend = true;}

       if(needsend)
       {
        int send_b = real_b + (int)acc_b;
        int send_f = real_f + (int)acc_f;
        int send_r = real_r + (int)acc_r;
        int send_c = real_c + (int)acc_c;

        real_b = send_b;
        acc_b =0;
        real_f = send_f;
        acc_f =0;
        real_r = send_r;
        acc_r =0;
        real_c = send_c;
        acc_c =0;
        waitFrame = 0;

        moveto3(send_b,send_f,send_r,send_c);
 

       }

      (*interpNowStepp)++;
      if(*interpNowStepp > interStepTotal){
        real_b = pTo_b;
        real_f = pTo_f;
        real_r = pTo_r;
        real_c = pTo_c;
        acc_b = 0;
        acc_f = 0;
        acc_r = 0;
        acc_c = 0;
         moveto3(real_b,real_f,real_r,real_c);

      *interpNowStepp = 0;
      (*playIdxp)++;
    }

    }


      
     

    
    
    
    
}



void anjian5(){
  if(TeachpointCount < 5){

    alpha_in = base.read();
    beta_in = fArm.read();
    gamma_in = rArm.read();

    ArmAngles tmpAng;
    tmpAng.alpha = alpha_in;
    tmpAng.beta = beta_in;
    tmpAng.gamma = gamma_in;

    ArmPoint pt = zhengjie(tmpAng);

    if(pt.valid)
    {
      
      
         x[TeachpointCount] = (int16_t)pt.x;
        y[TeachpointCount] = (int16_t)pt.y;
      
      Serial.print(F("保存示教点: x= "));
      Serial.print(x[TeachpointCount]);Serial.print(F(" y="));Serial.println(y[TeachpointCount]);
      TeachpointCount++;
    }
    else
    {
      Serial.println(F("该位置无效,无法保存示教点"));
    }
    }else{
      Serial.println(F("Reached the quantity limit!"));
    }
}

void anjian6(){
  if(qidong)
  {
    // Serial.println(F("正在绘图,禁止切换模式!先取消绘图"));
    return;
  }
  zhixiantai = !zhixiantai;
  Serial.print(F("是否直线:"));Serial.println(zhixiantai);
}

void anjian7(){
  bool oldState = qidong;
  qidong = !qidong;

  
  if(!oldState && qidong && drawIndex == 0 && currentT == 0){
     
     basetarget = 90;
     fArmtarget = 90;
     rArmtarget = 90;
     anjian4();

     

     

  }

  
 
}

void anjian8(){
  qidong = false;
  drawIndex =0;
  currentT =0;
  TeachpointCount =0;
  //按键7开什么就关什么,按初始化标准解决
  moveto3(90,90,90,120);
  //moveto3需要优化掉delay
  basetarget = 90;
  fArmtarget = 90;
  rArmtarget = 90;
  clawtarget = 120;
}

ArmAngles nijie(ArmPoint target)
{
    ArmAngles res;
    res.valid = false;

    float x = target.x;
    float y = target.y;
    float H = target.H;

    //1、计算水平总投影L
    float L = sqrt(x*x + y*y);
    float L_minus_L4 = L - L4;

    //奇异点检测：L-L4接近0，AC杆接近竖直，分母趋近0，方程病态无解
    if(fabs(L_minus_L4) < 1.0f)
    {
        return res;
    }

    //2、计算theta6：K杆与水平面的锐角
    float deltaH = H - L3;
    float theta6_rad = atan(fabs(deltaH) / L_minus_L4);
    float theta6_deg = rad2deg(theta6_rad);

    //3、求θ5 = ∠ACD，严格和推导文档公式保持一致
    float theta5_deg;
    if(deltaH >= 0.0f)
    {
        theta5_deg = 90.0f + theta6_deg;
    }
    else
    {
        theta5_deg = 90.0f - theta6_deg;
    }

    //4、求AC杆长度K
    float K = L_minus_L4 / cos(theta6_rad);

    //三角形构成条件：K必须在 |L1‑L2| ~ L1+L2之间，否则目标点在工作空间之外
    if( K > (L1 + L2) || K < fabs(L1 - L2) )
    {
        return res;
    }

    //5、余弦定理计算cosθ2 cosθ3
    //⚠浮点误差会让数值轻微超出[-1,1]，acos会返回NaN，必须constrain钳位！
    float cosTheta2 = (L1*L1 + L2*L2 - K*K) / (2.0f * L1 * L2);
    cosTheta2 = constrain(cosTheta2, -1.0f, 1.0f);

    float cosTheta3 = (K*K + L2*L2 - L1*L1) / (2.0f * K * L2);
    cosTheta3 = constrain(cosTheta3, -1.0f, 1.0f);

    //acos返回值域 [0 , π]，默认得到【肘向上】解
    float theta2_rad = acos(cosTheta2);
    float theta3_rad = acos(cosTheta3);

    float theta2_deg = rad2deg(theta2_rad);
    float theta3_deg = rad2deg(theta3_rad);

    //6、装配几何关系求舵机β γ，与推导论文公式完全一致
    float beta_deg  = 360.0f - theta5_deg - theta3_deg - theta2_deg;
    float gamma_deg = theta2_deg + fabs(beta_deg - 90.0f);

    //7、基座旋转角α；arduino atan2(y , x) 参数顺序 y在前 x在后，四象限求解
    float alpha_rad = atan2(y , x);
    float alpha_deg = rad2deg(alpha_rad);

    //角度归一化
    res.alpha  = normalizeDeg(alpha_deg);
    res.beta   = normalizeDeg(beta_deg);
    res.gamma  = normalizeDeg(gamma_deg);
    res.valid  = true;

    return res;
}

ArmPoint zhengjie(ArmAngles ang)
{
    ArmPoint pt;
    pt.valid = false;

    float alpha_deg = ang.alpha;
    float beta_deg  = ang.beta;
    float gamma_deg = ang.gamma;

    //----------1.由beta、gamma求解theta2，解开绝对值----------
    float theta2_deg;
    if(beta_deg >= 90.0f)
    {
        //case1 β >=90°： |β‑90| = β‑90
        theta2_deg = gamma_deg - beta_deg + 90.0f;
    }
    else
    {
        //case2 β <90°： |β‑90| = 90‑β
        theta2_deg = gamma_deg + beta_deg - 90.0f;
    }

    float theta2_rad = deg2rad(theta2_deg);

    //----------2.余弦定理求K(AC长度)----------
    float K = sqrt(L1*L1 + L2*L2 - 2*L1*L2*cos(theta2_rad));

    //----------3.正弦定理求θ3----------
    // L1 / sinθ3 = K / sinθ2

  

    // ====== 用atan2求theta3，彻底消除asin象限歧义 ======
  float cosTheta3 = (L2*L2 + K*K - L1*L1) / (2.0f * L2 * K);
   cosTheta3 = constrain(cosTheta3, -1.0f, 1.0f);
   float theta3_rad = acos(cosTheta3);


    float theta3_deg = rad2deg(theta3_rad);

    //----------4.装配关系求θ5： β = 360 −θ5 −θ3 −θ2 →变形解θ5----------
    //float theta5_deg = 360.0f - beta_deg - theta3_deg - theta2_deg;
   float theta5_deg = 360.0f - beta_deg  - theta3_deg - theta2_deg;
    float theta5_rad = deg2rad(theta5_deg);

    //----------5.求水平总投影L  L = L4 + K*cos(|θ5−90°|) 等价 L = L4 + K*sinθ5----------
    float L = L4 + K * sin(theta5_rad);

    //----------6.极坐标转x,y----------
    float alpha_rad = deg2rad(alpha_deg);
    pt.x = L * cos(alpha_rad);
    pt.y = L * sin(alpha_rad);

    //----------7.求A点离地高度H（正解也算出H，用于和逆解互校验）----------
    // float theta6_deg;
    // if(theta5_deg >= 90.0f)
    // {
    //     theta6_deg = theta5_deg - 90.0f;
    // }
    // else
    // {
    //     theta6_deg = 90.0f - theta5_deg;
    // }
    // float deltaH = K * sin(deg2rad(theta6_deg)) * ((theta5_deg>=90)? 1.0f:-1.0f);
    // pt.H = L3 + deltaH;

    pt.valid = true;
    return pt;
}



// void changduanjian(){

//     if(!sd){
//         StartTime = EndTime = millis();
        
//     }else{
//         EndTime = millis();
//     }
//     sd = !sd;

// }


void setup() {
  // put your setup code here, to run once:


  claw.attach(clawPin);
  rArm.attach(rArmPin);
  fArm.attach(fArmPin);
  base.attach(basePin);


  rArm.write(rArmpos);
  fArm.write(fArmpos);
  base.write(basepos);
  delay(100);


  claw.write(angleclose);
  Serial.begin(9600);
  SoftSerial.begin(9600);
  Serial.println(F("Please input serial data"));
  step = MIN_speedstep;
  basetarget = base.read();
  fArmtarget = fArm.read();
  rArmtarget = rArm.read();
  clawtarget = claw.read();


  jianche[0] = 0;
  jianche[1] = 0;
  jianche[2] = 0;
  jianche[3] = 0;
  jianche[4] = 0;
  jianche[5] = 0;
  jianche[6] = 0;
  jianche[7] = 0;

  
  
  sd = 0;


  zhuangtai2 =false;
  zhuangtai3 =false; 
  
  zhenshu2 =0;



  Index =0;


  nowTime=0;
  LastTime =0;
  LastTime = nowTime;
  ipzhenshu = 0;

  zhixiantai = true;
  TeachpointCount = 0;
  // jisuanC = false;
  qidong = false;

  
   Xnow =0;
   Ynow =0;

   alpha_in =0;
   beta_in =0;
   gamma_in =0;

   //StartTime = EndTime =playTime =0;
   b = f =c= r =0;

   interpLastTime =0;

 recordLastTime =0;
 playLastTime =0;

 playIdx =0;
    interpNowStep =0;

    // Serial.println("====机械臂逆运动学测试启动====");

    // //=======测试目标点，修改这里更换坐标========
    // ArmPoint target;
    // target.x  = 120.0f;   //X mm
    // target.y  = 40.0f;    //Y mm
    // target.H  = 90.0f;    //A点离地高度 mm

    // //求解【肘部向上】构型
    // ArmAngles solUp = nijie(target, true);
    
    // Serial.println(F("\n--------肘部向上解--------"));
    // if(solUp.valid)
    // {
    //     Serial.print(F("alpha(base理论角度):"));Serial.print(solUp.alpha);Serial.print(F(" ° | "));
    //     Serial.print(F("beta(fArm理论角度):")); Serial.print(solUp.beta); Serial.print(F(" ° | "));
    //     Serial.print(F("gamma(rArm理论角度):"));Serial.print(solUp.gamma);Serial.println(F(" °"));

    //     //转换为舵机实际角度
    //     float realA = mapServoAngle(solUp.alpha, OFFSET_ALPHA);
    //     float realB = mapServoAngle(180.0f - solUp.beta,  OFFSET_BETA);
    //     float realG = mapServoAngle(solUp.gamma, OFFSET_GAMMA);
    //     Serial.print(F("舵机输出角度 A:"));Serial.print(realA);Serial.print(" B:");Serial.print(realB);Serial.print(F(" G:"));Serial.println(realG);

    //     //驱动舵机运动；确认参数没问题再取消下面注释！防止撞机
        
    //     base.write(realA);
    //     fArm.write(realB);
    //     rArm.write(realG);
        
//     }
//     else
//     {
//         Serial.println(F("肘部向上：无解/超出工作空间/奇异点"));
//     }

//     ArmAngles testAng;
// testAng.alpha = 45;
// testAng.beta  =120;
// testAng.gamma = 90;//90-150,70-100,80-120;

// ArmPoint fpt = zhengjie(testAng);
// if(fpt.valid)
// {
//   Serial.print(F("【正解】 x="));Serial.print(fpt.x);
//   Serial.print(F("  y="));Serial.print(fpt.y);
//   Serial.print(F("  H="));Serial.println(fpt.H);

//   ArmAngles invSol = nijie(fpt,true);
//   if(invSol.valid)
//   {
//       Serial.print(F("【逆解回算】 alpha="));Serial.print(invSol.alpha);
//       Serial.print(F(" beta="));Serial.print(invSol.beta);
//       Serial.print(F(" gamma="));Serial.println(invSol.gamma);
//   }
// }

}


  



void loop() {
  // put your main code here, to run repeatedly:

  
  

// 如果收到串口数据就读取
if(SoftSerial.available()){
  // Serial.println(F("链接"));
  uint8_t cmd = SoftSerial.read();
  if(cmd >= 1 && cmd <= 8){
    jianche[cmd -1] = true;
  }
}


  

  baseoff = analogRead(joy1_X) - MID;
  rArmoff = analogRead(joy1_Y) - MID;
  clawoff = analogRead(joy2_X) - MID;
  fArmoff = analogRead(joy2_Y) - MID;
  
  

  nowTime = millis();



  // for(int i =0;i<4;i++)
  // {

  //   if(last[i] != now[i]){
  //     //Serial.print(i);//检测状态，可注释
  //     changduanjian();
  //     if(EndTime - StartTime > 200 && EndTime - StartTime < 800){
  //       //20
  //   //单击
  //   //Serial.println(F("单击按钮按下"));
  //   jianche[i] = true;
  //   EndTime = StartTime =0;

  // }else if(EndTime - StartTime >= 800 && EndTime - StartTime < 10000) {
  //   //长按
  //  // Serial.println(F("长按按钮按下"));
  //   jianche[i+4] = true;
  //   EndTime = StartTime =0;

  // }
  //   }

  //   last[i] = now[i];
  // }

  


  if(jianche[0]){
   
    zhuangtai2 = !zhuangtai2;
    LastTime = nowTime;
    jianche[0] = false;
  }
  if(jianche[1]){
    

    LastTime = nowTime ;
    zhuangtai3 = true;
    jianche[1] = false;
  }
  if(jianche[2]){
  
    

    anjian5();
    jianche[2] = false;
  }
  if(jianche[3]){
   

    jisuanC = true;
    anjian7();
    jianche[3] = false;
  }


  if(jianche[4]){
    

    taskCount = (taskCount +1)%3;
    Serial.print(F("ABC模式:"));Serial.println(taskCount);
    anjian1();
    jianche[4] = false;
  }
  if(jianche[5]){
   

    anjian4();
    jianche[5] = false;
  }

  if(jianche[6]){
    
    anjian6();
    jianche[6] = false;
  }
  if(jianche[7]){
    anjian8();
    jianche[7] = false;
  }

  //后期要防呆

  
  if(zhuangtai2 == false){//只要结束->帧数归0

            recordStopFlag = true;

            }else{
              
              if(millis() - playTime >= zhenZhenshu){
                //当时间来到了指定的时候
                //Serial.println(F("一帧"));
                playTime = millis();//时间差归零
 
                anjian2(base.read(),fArm.read(),rArm.read(),claw.read(),ipzhenshup);
            }
    }
             

  if(zhuangtai2 == false && recordStopFlag == true){
    
    zhenshu2 =0;
    recordStopFlag = false;
  }

  

  if(zhuangtai3 == true){
     
    
   
            
    if(nowTime - playLastTime >= STEP_MS){
      anjian3(interpNowStepp,playIdxp,zhuangtai3p);
      
    
      playLastTime = nowTime ;
      
    }
            
  }


  if(millis() - b >= 10){
    yaogan(baseoff,basetargetp,base,baseMin,baseMax);

    b = millis();
  }
  if(millis() - f >= 10){
    yaogan(fArmoff,fArmtargetp,fArm,fArmMin,fArmMax);

    f = millis();
  }
  if(millis() - r >= 10){
    yaogan(rArmoff,rArmtargetp,rArm,rArmMin,rArmMax);

    r = millis();
  }
  if(millis() - c >= 10){
    yaogan(clawoff,clawtargetp,claw,clawMin,clawMax);

    c = millis();
  }


 
  
  
  

if(qidong){
  
  if(TeachpointCount < 2){
   Serial.println(F("示教点数量不足,停止绘图"));
    qidong = false;
    drawIndex = 0;
    currentT =0;
  }

  if((nowTime - drawLastTime) < DRAW_STEP_MS )
  {
    return;
  }
  drawLastTime = nowTime;

  if(zhixiantai == false){
    if(TeachpointCount != 5){
     Serial.println(F("曲线模式必须存储5个示教点!切换直线模式"));
      zhixiantai = true;
    }
  }



  if(zhixiantai==false && TeachpointCount ==5){

      float x0,y0,x1,y1,x2,y2,x3,y3;
      if(drawIndex == 0){
        x0 = x[0];y0 = y[0];
        x1 = x[0];y1 = y[0];
        x2 = x[1];y2 = y[1];
        x3 = x[2];y3 = y[2];
      }else if(drawIndex == 1){
        x0 = x[0];y0 = y[0];
        x1 = x[1];y1 = y[1];
        x2 = x[2];y2 = y[2];
        x3 = x[3];y3 = y[3];
      }else if(drawIndex == 2){
        x0 = x[1];y0 = y[1];
        x1 = x[2];y1 = y[2];
        x2 = x[3];y2 = y[3];
        x3 = x[4];y3 = y[4];
      }else if(drawIndex == 3){
        x0 = x[2];y0 = y[2];
        x1 = x[3];y1 = y[3];
        x2 = x[4];y2 = y[4];
        x3 = x[4];y3 = y[4];
      }float t = currentT;
      float t2 = t*t;
      float t3 = t2 * t;

      float w0 = 0.5f * (-t3 + 2*t2 -t);
      float w1 = 0.5f * (3*t3 - 5*t2 + 2);
      float w2 = 0.5f * (t + 4*t2 - 3*t3);
      float w3 = 0.5f * (-t2 + t3);

      Xnow = x0*w0 + x1*w1 + x2*w2 + x3*w3;
      Ynow = y0*w0 + y1*w1 + y2*w2 + y3*w3;

      fpt.x = Xnow;
      fpt.y = Ynow;
      fpt.H = 80.0f;

      ArmAngles sol = nijie(fpt);

    


if(sol.valid == true)
{
    // 理论角度叠加零点偏移，得到舵机输出角度

    base_out = sol.alpha + OFFSET_ALPHA;
    fArm_out = 180.0f - sol.beta + OFFSET_BETA;
    rArm_out = sol.gamma + OFFSET_GAMMA;

    base_out = constrain(base_out,0.0f,180.0f);
    fArm_out = constrain(fArm_out,0.0f,180.0f);
    rArm_out = constrain(rArm_out,0.0f,180.0f);

 
// MoveTo(base,b,step);
//   MoveTo(fArm,f,step);
//   MoveTo(rArm,r,step);
    base.write(base_out);
    fArm.write(fArm_out);
    rArm.write(rArm_out);

    
    //同步摇杆,防止跳变
    basetarget = (int)base_out;
    fArmtarget = (int)fArm_out;
    rArmtarget = (int)rArm_out;
}
}
    else{
      if(drawIndex >= TeachpointCount-1)
      {
        qidong = false;
        drawIndex =0;
        currentT =0;
        return;
      }
      Xnow = x[drawIndex] + currentT*(x[drawIndex+1] - x[drawIndex]);
      Ynow = y[drawIndex] + currentT*(y[drawIndex+1] - y[drawIndex]);

      fpt.x = Xnow;
      fpt.y = Ynow;
      fpt.H = 55.0f;

      ArmAngles sol = nijie(fpt);

if(sol.valid == true)
{
    // 理论角度叠加零点偏移，得到舵机输出角度
    base_out = sol.alpha + OFFSET_ALPHA;

    fArm_out = 180 - sol.beta + OFFSET_BETA;
    
    rArm_out = sol.gamma + OFFSET_GAMMA;

    base_out = constrain(base_out,0.0f,180.0f);
    fArm_out = constrain(fArm_out,0.0f,180.0f);
    rArm_out = constrain(rArm_out,0.0f,180.0f);

   
// MoveTo(base,b,step);
//   MoveTo(fArm,f,step);
//   MoveTo(rArm,r,step);

    base.write(base_out);
    fArm.write(fArm_out);
    rArm.write(rArm_out);

    


    basetarget = (int)base_out;
    fArmtarget = (int)fArm_out;
    rArmtarget = (int)rArm_out;
}

    }
    currentT+=stepT;
    if(currentT>= 1.0f){
      drawIndex++;
      currentT = 0; 
    }
    if(drawIndex >= TeachpointCount -1){
      currentT = 0;
      drawIndex = 0;
      qidong = false;

      Serial.println(F("绘图执行完成"));
      
    }

  



}
  
 if(Serial.available() > 0)
  {
   

    char cmd = Serial.read();

     if(cmd == '\n'){
      Scmd[Index] = '\0';
      cmdall(Scmd);
      
      Index =0;
      memset(Scmd,0,sizeof(Scmd));
    }
    else if(cmd != '\r'){
      if(Index < neichun -1){
        Scmd[Index] = cmd;
        Index++;
      }
      else{
        Index = 0;
      }
    }
    
  }

}


