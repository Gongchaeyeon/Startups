// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
int a, b; // unit: mm
#include <Servo.h>
#define PIN_SERVO 10
float dist_ema;
Servo myservo;

void setup() {
  
  b = 345;
  a = 86;
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(1580);//중간
  Serial.begin(57600);
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {  
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
//  dist_ema = raw_dist*0.9 +(1-0.9)*dist_ema;

  Serial.print("min:0,max:500,dist:");
  Serial.println(raw_dist);
//  Serial.print(",dist_ema:");
//  Serial.println(dist_ema);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);

  if (raw_dist >= 171){
    myservo.writeMicroseconds(1380);//최대
  }else{
    myservo.writeMicroseconds(1750);//최소     
  }
  delay(20);  
  }
