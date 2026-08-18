//硬件版本： USB电压电流表
//软件版本： 2.0
//创建时间： 熵 2022-9-28
//STM32 USB VA电压电流表
//量程5~30V/1mA-10A

#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display(128, 64, &Wire, 4); //(宽度, 高度, &Wire, 4) 


byte a1 = PA1;        //大电流采样
byte a2 = PA2;        //小电流采样
byte dn = PA3;        //D-电压采样 
byte dp = PA0;        //D+电压采样
byte wd = PA4;        //NTC采样
byte dy = PA5;        //电压采样

byte k1 = PA15;       //按键检测
byte k2 = PB4;        //按键检测
byte k3 = PB10;       //按键检测
byte k4 = PB1;        //按键检测

byte M1 = PB9;        //大电流控制
byte M2 = PB8;        //小电流控制

float V ;             //电源电压
float VN ;            //D-电压  
float VP ;            //D+电压 
     
float A ;             //电流      
float W ;             //功率    
float R ;             //负载    
float C ;             //温度    
int Wh ;              //电量
long int T;           //时间
long int WT ;         //电功

byte K1,K2,K3,K4;     //按键

byte ms = 0;



//测电压 电流 功率
void VA(){
  float vn = map(analogRead(dn),0,4095,0,3300);
  VN = vn*2/1000;
  float vp = map(analogRead(dp),0,4095,0,3300);
  VP = vp*2/1000;

  float LB = 0;
  for(byte i = 0;i < 50;i++){
  float v = map(analogRead(dy),0,4095,0,3300);
  LB = v * 12.01 / 1000 + LB;
}
  V = LB/50;
  
  float LA = 0;
  for(byte i = 0;i < 50;i++){
    float a = map(analogRead(a1),0,4095,0,3300);
    A = a/101/5;
    LA = LA+A;
  }
  A = LA/50;

  W = V*A;
  if( A > 0.1){ R = V/A; }else{ R = 0; }
}


//测电量 每过一秒记录一次电功
void UIT(){ 
  if(millis() - T > 1000){ 
    T = millis();
    WT = WT+W;
    Wh = WT/3.6;
  }   
}

//测温度
void NTC(){
  pinMode( wd , INPUT_ANALOG);
  float v = map(analogRead(wd),0,4095,0,3300);
  float ntc = (10000 * v)/(3300 - v);
  //K =(3950 * 298.15)/(3950 +(298.15 * log( ntc/ 10000)));
  C = (3950 * 298.15)/(3950 +(298.15 * log( ntc/10000))) - 273.15 - 2;  

}

//测按键
void KEY(){
  K1 = digitalRead( k1 );
  K2 = digitalRead( k2 );
  K3 = digitalRead( k3 );
  K4 = digitalRead( k4 );   
}

void SSD1306(){
  display.clearDisplay();                   //清理1306屏幕，准备显示：
  display.setTextSize(2);                   //设置字体大小，正比
  display.setTextColor(WHITE);              //设置字体颜色
  display.clearDisplay();                   //清屏   
  
  display.setCursor( 0 , 0 );   
  display.print(V); 
  display.setCursor(62, 0 );   
  display.print("V");
  
  display.setCursor( 0 , 16 );   
  display.print(A,3); 
  display.setCursor( 62, 17 );   
  display.print("A");     

  display.setCursor( 0 , 33);   
  display.print(W); 
  display.setCursor( 62, 33);   
  display.print("W");

  display.setCursor( 0 , 49);   
  display.print(R); 
  display.setCursor( 62, 49);   
  display.print("R");

  display.setTextSize(1);

  display.setCursor( 81, 0 );   
  display.print(C, 1); 
  display.setCursor(116, 0 );   
  display.print("C*");
  
  display.setCursor( 81, 8 );   
  display.print(VP); 
  display.setCursor(116, 8 );   
  display.print("D+");            
 
  display.setCursor( 81, 17);   
  display.print(VN); 
  display.setCursor(116, 17);   
  display.print("D-");

  display.setCursor( 81, 25);   
  display.print(Wh); 
  display.setCursor(110, 25);   
  display.print("mWh");
 
  display.display();                     //把缓存都显示 
}

//串口打印
void ckdy(){
 Serial.print(V ); Serial.print(" V   ");
 Serial.print(VN); Serial.print(" VN  ");
 Serial.print(VP); Serial.print(" VP  ");
 Serial.print(A ); Serial.print(" A   ");
 Serial.print(W ); Serial.print(" W   ");
 Serial.print(R ); Serial.print(" R   ");
 Serial.print(C ); Serial.print(" C   ");

 Serial.print(K1); Serial.print(" k1  ");
 Serial.print(K2); Serial.print(" k2  ");
 Serial.print(K3); Serial.print(" k3  ");
 Serial.print(K4); Serial.println(" k4"); 
}


void setup(){
  analogReadResolution(12);   // ADC设置为12位：0~4095
  //Serial.begin(115200);
  pinMode( a1 , INPUT_ANALOG);
  pinMode( a2 , INPUT_ANALOG);
  pinMode( dn , INPUT_ANALOG);
  pinMode( dp , INPUT_ANALOG);
  pinMode( wd , INPUT_ANALOG);
  pinMode( dy , INPUT_ANALOG);
  
  pinMode( k1 , INPUT_PULLUP);     
  pinMode( k2 , INPUT_PULLUP);     
  pinMode( k3 , INPUT_PULLUP);     
  pinMode( k4 , INPUT_PULLUP); 
      
  pinMode( M1, OUTPUT);
  pinMode( M2, OUTPUT);
  digitalWrite( M1 ,HIGH);
  digitalWrite( M2 ,LOW);
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); //初始化I2C地址0X3C
  //display.setRotation(2);                  //设置屏幕方向 0/0°, 1/90° ,2/180°, 3/270°
}

void loop(){
  VA();
  UIT();
  NTC();
  KEY();
  //ckdy();
  SSD1306();
}
