#include <Adeept_Distance.h>

double distance  ;
int movinspeed = 100;
int mode = 1 ;
Adeept_Distance Dist;
/***********************************************************
File   : TDTP2_Stand_Up
Subject: Stand Up ADEEPT Hexapode 
Author : Doc. Ing. Med Radhouan HACHICHA
E-mail : radhouan.mail@gmail.com*/
#include <Adeept_PWMPCA9685.h>
Adeept_PWMPCA9685 pwm0 = Adeept_PWMPCA9685(0x40);                //1+A5 A4 A3 A2 A1 A0+RW, RW is Read and Write
Adeept_PWMPCA9685 pwm1 = Adeept_PWMPCA9685(0x41);                //1+A5 A4 A3 A2 A1 A0+RW, RW is Read and Write

float alpha, beta, gamma; // Control Angles to calculate
float x=0,y=50.5,z=-22.5; // Default leg position Coordinates

//Default leg dimentions
float c = 34.5;  // z axis Offset
float d = 12.5;  //Coxa length
float e = 38;    //Tibia length
float f = 57;    //Femur length

//Default Legs angles
float alpha1=90, beta1=90, gamma1=90;   //Leg1
float alpha2=90, beta2=90, gamma2=90;   //Leg2
float alpha3=90, beta3=90, gamma3=90;   //Leg3
float alpha4=90, beta4=90, gamma4=90;   //Leg4
float alpha5=90, beta5=90, gamma5=90;   //Leg5
float alpha6=90, beta6=90, gamma6=90;   //Leg6

//Angle offset adjustment
int a1=5,b1=5,g1=0;   //Leg1: (a1:PWM13 offset);(b1:PWM14 offset);  (g1:PWM15 offset);
int a2=5,b2=0,g2=-10;    //Leg2: (a2:PWM18 offset); (b2:PWM17 offset); (g2:PWM16 offset);
int a3=7,b3=12,g3=5;    //Leg3: (a3:PWM9 offset);  (b3:PWM10 offset); (g3:PWM11 offset);
int a4=-5,b4=-5,g4=-3;  //Leg4: (a4:PWM22 offset); (b4:PWM21 offset); (g4:PWM20 offset);
int a5=-10,b5=10,g5=0;    //Leg5: (a5:PWM5 offset);  (b5:PWM6 offset);  (g5:PWM7 offset);
int a6=0,b6=0,g6=-10;    //Leg6: (a6:PWM26 offset); (b6:PWM25 offset); (g6:PWM24 offset);

void CalculateAngle(float x, float y, float z, float &alpha, float &beta, float &gamma){
  // calculate u-v angle
  float u, v;
  u = sqrt(pow(x, 2) + pow(y, 2));
  v = z;
  beta = PI / 2 - acos((pow(e, 2) + (pow(u - d, 2) + pow(v - c, 2)) - pow(f, 2)) / (2 * e * sqrt(pow(u - d, 2) + pow(v - c, 2)))) - atan2(v - c, u - d);
  gamma = acos((pow(e, 2) + pow(f, 2) - (pow(u - d, 2) + pow(v - c, 2))) / (2 * e * f));
  // calculate x-y-z angle
  alpha = atan2(y, x);
  // transform radian to angle
  alpha = alpha * 180 / PI;
  beta = beta * 180 / PI;
  gamma = gamma * 180 / PI;
}

void Leg1Position(float x, float y, float z){
  CalculateAngle(x,y,z,alpha, beta, gamma); 
  alpha1=alpha+a1; beta1=beta+b1; gamma1=gamma+g1;
}
void Leg2Position(float x, float y, float z){
  CalculateAngle(x,y,z,alpha, beta, gamma); 
  alpha2=alpha+a2; beta2=beta+b2; gamma2=gamma+g2;
}
void Leg3Position(float x, float y, float z){
  CalculateAngle(x,y,z,alpha, beta, gamma); 
  alpha3=alpha+a3; beta3=beta+b3; gamma3=gamma+g3;
}
void Leg4Position(float x, float y, float z){
  CalculateAngle(x,y,z,alpha, beta, gamma); 
  alpha4=alpha+a4; beta4=beta+b4; gamma4=gamma+g4;
}
void Leg5Position(float x, float y, float z){
  CalculateAngle(x,y,z,alpha, beta, gamma); 
  alpha5=alpha+a5; beta5=beta+b5; gamma5=gamma+g5;
}
void Leg6Position(float x, float y, float z){
  CalculateAngle(x,y,z,alpha, beta, gamma); 
  alpha6=alpha+a6; beta6=beta+b6; gamma6=gamma+g6;
}

//Angle Convertion
int angle(int angle){//Angle conversion
  if(angle>=180){angle=180;}
  if(angle<=0){angle=0;}
  return map(angle,0,180,190,570);//570-190 
}
int oppAngle(int oppAngle){// The opposite direction angle conversion
  if(oppAngle>=180){oppAngle=180;}                  
  if(oppAngle<=0){oppAngle=0;}
 return map(oppAngle,0,180,570,190);//190-570
}

void MoveAnglePosition(){
  pwm0.setPWM(13,0,oppAngle(alpha1)); pwm0.setPWM(14,0,oppAngle(beta1)); pwm0.setPWM(15,0,oppAngle(gamma1)); //Leg1
  pwm1.setPWM(2,0,angle(alpha2)); pwm1.setPWM(1,0,angle(beta2)); pwm1.setPWM(0,0,angle(gamma2));             //Leg2 
  pwm0.setPWM(9,0,oppAngle(alpha3)); pwm0.setPWM(10,0,oppAngle(beta3)); pwm0.setPWM(11,0,oppAngle(gamma3));  //Leg3
  pwm1.setPWM(6,0,angle(alpha4)); pwm1.setPWM(5,0,angle(beta4)); pwm1.setPWM(4,0,angle(gamma4));             //Leg4
  pwm0.setPWM(5,0,oppAngle(alpha5)); pwm0.setPWM(6,0,oppAngle(beta5)); pwm0.setPWM(7,0,oppAngle(gamma5));    //Leg5
  pwm1.setPWM(10,0,angle(alpha6)); pwm1.setPWM(9,0,angle(beta6)); pwm1.setPWM(8,0,angle(gamma6));            //Leg6
}

void setup(){
  pwm0.begin();
  pwm0.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pwm1.begin();
  pwm1.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  Serial.begin(9600);
  
  //Leg1Position(5, 60, -18); Leg2Position(0, 45, -19); Leg3Position(-5, 40, -22.5);
  //Leg4Position(0, 45, -26); Leg5Position(3, 35, -25); Leg6Position(0, 45, -20);

  Leg2Position(0, 50.5, -22.5); Leg3Position(0, 50.5, -22.5); Leg6Position(0, 50.5, -22.5); 
  Leg1Position(0, 50.5, -22.5); Leg4Position(0, 50.5, -22.5); Leg5Position(0, 50.5, -22.5);

  MoveAnglePosition();
  pwm0.setPWM(3, 0, angle(90)); //Sound Radar initial Position
}

void loop()
{

UltrasoundDetectionMode() ;

}

void UltrasoundDetectionMode(){
  pwm0.setPWM(3,0,angle(90));  
  distance = Dist.getDistanceCentimeter() ;
  if (distance <20){
     ForwardStop() ;
      ForwardStart();
     pwm0.setPWM(3,0,angle(170)); 
     double distance_left = Dist.getDistanceCentimeter();
     delay(500) ;
      pwm0.setPWM(3,0,angle(0)); 
     double distance_right = Dist.getDistanceCentimeter();
     if (distance_left > 30 && distance_left > distance_right){
      ForwardStart();
      TurnLeft() ;}
     else if (distance_right >= 30 &&  distance_left <= distance_right){
        ForwardStart();
        TurnRight() ;
      }
      
     else {
      ForwardStart();
      TurnRight() ;TurnRight() ;
     }
  }
  else {
    ForwardStart();Forward();
  }
}


void ForwardStart() {
Leg3Position(0,50.5, 0) ;
Leg4Position(0,50.5,0) ;
delay(100);MoveAnglePosition();
Leg3Position(17.27,47.45,0);
Leg4Position(-17.27,47.45,0);
delay(100);MoveAnglePosition();
Leg3Position(17.27,47.45,-22.5);
Leg4Position(-17.27,47.45,-22.5);
delay(100);MoveAnglePosition();
Leg2Position(0,50.5,0);
Leg5Position(0,50.5,0);
delay(100);MoveAnglePosition();
Leg5Position(-32.46,38.69,0);
Leg5Position(-32.46,38.69,-22.5);
delay(100);MoveAnglePosition();
}

void Forward(){
  Leg1Position(0,50.5,-22.5);
  Leg2Position(32.46,38.69,0);
  Leg3Position(17.27,47.45,0);
  Leg4Position(-17.27,47.45,-22.5);
  Leg5Position(-32.46,38.69,-22.5);
  Leg6Position(0,50.5,0);
  delay(100);MoveAnglePosition();
  Leg2Position(32.46,38.69,-22.5);
  Leg3Position(17.27,47.45,-22.5);
  Leg6Position(0,50.5,-22.5);
  delay(100);MoveAnglePosition();
  Leg1Position(0,50.5,0);
  Leg4Position(-17.27,47.45,0);
  Leg5Position(-32.46,38.69,0);
  delay(100);MoveAnglePosition();
  Leg1Position(32.46,38.69,0);
  Leg2Position(0,50.5,-22.5);
  Leg3Position(-17.27,47.45,-22.5);
  Leg4Position(17.27,47.45,0);
  Leg5Position(0,50.5,0);
  Leg6Position(-32.46,38.69,-22.5);
  delay(100);MoveAnglePosition();
  Leg1Position(32.46,38.69,-22.5);
  Leg4Position(17.27,47.45,-22.5);
  Leg3Position(0,50.5,-22.5);
  delay(100);MoveAnglePosition();
  Leg2Position(0,50.5,0);
  Leg3Position(-17.27,47.45,0);
  Leg6Position(-32.46,38.69,0);
  delay(100);MoveAnglePosition();  
}


void ForwardStop(){
  Leg3Position(0,50.5,0);
  Leg6Position(0,50.5,0);
  delay(100);MoveAnglePosition(); 
  Leg2Position(0,50.5,-22.5);
  Leg3Position(0,50.5,-22.5);
  Leg6Position(0,50.5,-22.5);
  delay(100);MoveAnglePosition();
  Leg1Position(0,50.5,0);
  Leg4Position(0,50.5,0);
  delay(100);MoveAnglePosition();  
  Leg1Position(0,50.5,-22.5);
  Leg4Position(0,50.5,-22.5);
  delay(100);MoveAnglePosition(); 
}

void TurnLeft(){
  Leg2Position(0,50.5,0);
  Leg3Position(0,50.5,0);  
  delay(100);MoveAnglePosition();
  Leg2Position(32.46,38.69,0);
  Leg3Position(-17.27,47.45,0);  
  delay(100);MoveAnglePosition();
  Leg2Position(32.46,38.69,-22.5);
  Leg3Position(-17.27,47.45,-22.5);  
  delay(100);MoveAnglePosition();
  Leg1Position(0,50.5,0);
  Leg4Position(0,50.5,0);
  Leg5Position(0,50.5,0);
  delay(100);MoveAnglePosition(); 
  Leg2Position(0,50.5,-22.5);
  Leg3Position(17.27,47.45,-22.5);
  Leg4Position(17.27,47.45,0);
  Leg5Position(-32.46,38.69,0);
  Leg6Position(-32.46,38.69,-22.5);
  delay(100);MoveAnglePosition();
  Leg1Position(0,50.5,-22.5);
  Leg4Position(17.27,47.45,-22.5);
  Leg5Position(-32.46,38.69,-22.5);
  delay(100);MoveAnglePosition();
  Leg2Position(0,50.5,0);
  Leg3Position(17.27,47.45,0);
  Leg6Position(-32.46,38.69,0);
  delay(100);MoveAnglePosition();
  Leg1Position(32.46,38.69,-22.5);
  Leg2Position(0,50.5,0);
  Leg3Position(0,50.5,0);
  Leg4Position(-17.27,47.45,-22.5);
  Leg5Position(0,50.5,-22.5);
  Leg6Position(0,50.5,0);
  delay(100);MoveAnglePosition(); 
  Leg2Position(0,50.5,-22.5);
  Leg3Position(0,50.5,-22.5);
  Leg6Position(0,50.5,0);
  delay(100);MoveAnglePosition();
  Leg1Position(32.46,38.69,0);
  Leg4Position(-17.27,47.45,0);
  delay(100);MoveAnglePosition();
  Leg1Position(0,50.5,0);
  Leg4Position(0,50.5,0);
  delay(100);MoveAnglePosition();
  Leg1Position(0,50.5,-22.5);
  Leg4Position(0,50.5,-22.5);
  delay(100);MoveAnglePosition();
}

void TurnRight(){
  Leg1Position(0,50.5,0);
  Leg4Position(0,50.5,0);  
  delay(100);MoveAnglePosition();
  Leg1Position(32.46,38.69,0);
  Leg4Position(-17.27,47.45,0);  
  delay(100);MoveAnglePosition();
  Leg1Position(32.46,38.69,-22.5);
  Leg4Position(-17.27,47.45,-22.5);  
  delay(100);MoveAnglePosition();
  Leg2Position(0,50.5,0);
  Leg3Position(0,50.5,0);
  Leg6Position(0,50.5,0);
  delay(100);MoveAnglePosition(); 
  Leg1Position(0,50.5,-22.5);
  Leg3Position(17.27,47.45,-22.5);
  Leg4Position(17.27,47.45,0);
  Leg5Position(-32.46,38.69,0);
  Leg6Position(-32.46,38.69,-22.5);
  delay(100);MoveAnglePosition();
  Leg2Position(0,50.5,-22.5);
  Leg3Position(17.27,47.45,-22.5);
  Leg6Position(-32.46,38.69,-22.5);
  delay(100);MoveAnglePosition();
  Leg1Position(0,50.5,0);
  Leg4Position(17.27,47.45,0);
  Leg5Position(-32.46,38.69,0);
  delay(100);MoveAnglePosition();
  Leg1Position(32.46,38.69,-22.5);
  Leg2Position(0,50.5,0);
  Leg3Position(0,50.5,0);
  Leg4Position(-17.27,47.45,-22.5);
  Leg5Position(0,50.5,-22.5);
  Leg6Position(0,50.5,0);
  delay(100);MoveAnglePosition(); 
  Leg1Position(0,50.5,-22.5);
  Leg4Position(0,50.5,-22.5);
  Leg5Position(0,50.5,0);
  delay(100);MoveAnglePosition();
  Leg2Position(32.46,38.69,0);
  Leg3Position(-17.27,47.45,0);
  delay(100);MoveAnglePosition();
  Leg2Position(0,50.5,0);
  Leg3Position(0,50.5,0);
  delay(100);MoveAnglePosition();
  Leg2Position(0,50.5,-22.5);
  Leg3Position(0,50.5,-22.5);
  delay(100);MoveAnglePosition();
}
