#include <Servo.h> // 서보를 쓰기 위해 넣음.

// Arduino pin assignment------------------------------------------------------------
#define PIN_LED 9     // [1234] LED를 GPIO 9번 핀에 연결
#define PIN_SERVO 10  //[3030]SERVO -> 10번 핀에 연결
#define PIN_IR A0     //[3037]적외선 센서 -> A0에 연결

// Framework setting------------------------------------------------------------
#define _DIST_TARGET 255 // [3040] 레일플레이트의 중간지점(목표지점=25.5cm)
#define _DIST_MIN 100     //[3035] 측정 최소 거리
#define _DIST_MAX 410    //[3036] 측정 가능한 최대 거리

// Servo range ------------------------------------------------------------
#define _DUTY_MIN 1345 //[3028] 서보 각도 최소값
#define _DUTY_NEU 1575 //[3038] 레일 수평 서보 펄스폭
#define _DUTY_MAX 1745 //[3031] 서보 최대값

// Servo speed control------------------------------------------------------------
#define _SERVO_ANGLE 60
#define _SERVO_SPEED 2000

// Event periods------------------------------------------------------------
#define _INTERVAL_DIST 20   //[3039]적외선 센서 측정 간격
#define _INTERVAL_SERVO 20         //[3046]서보갱신간격
#define _INTERVAL_SERIAL 100       //[3030]시리얼 플로터 갱신간격

// PID parameters------------------------------------------------------------
#define _KP 1.1
#define _KD 68
#define _KI 0.005

//EMA -------------------------------------------------------------
#define EMA_ALPHA 0.16     // EMA 필터 값을 결정하는 ALPHA 값. 작성자가 생각하는 최적값임.

Servo myservo;
float dist_min,dist_max,duty_neutral; // 최소, 최대, 중간 거리

// Distance sensor------------------------------------------------------------
float dist_target; // 공을 보낼 위치
float dist_raw, dist_ema; //[3034] 적외선센서로 측정한 거리값과 ema필터를 적용한 거리값

// Event periods------------------------------------------------------------
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;   //[3039] interval간격으로 동작 시행을 위한 정수 값
bool event_dist, event_servo, event_serial; //[3023] 적외선센서의 거리측정, 서보모터, 시리얼의 이벤트 발생 여부

// Servo speed control------------------------------------------------------------
int duty_chg_per_interval;  //[3039] interval 당 servo의 돌아가는 최대 정도
int duty_target, duty_curr; //[3030]servo 목표 위치, servo 현재 위치

// PID variables------------------------------------------------------------

float error_curr, error_prev, control, pterm, iterm, dterm; // [3042] 현재 오차값, 이전 오차값, ? , p,i,d 값

float ema_dist=0;            // EMA 필터에 사용할 변수
float filtered_dist;       // 최종 측정된 거리값을 넣을 변수. loop()안에 filtered_dist = filtered_ir_distance(); 형태로 사용하면 됨.
float samples_num = 3;     // 스파이크 제거를 위한 부분필터에 샘플을 몇개 측정할 것인지. 3개로 충분함! 가능하면 수정하지 말 것.
const float coE[] = {0.0000002, -0.0016359, 1.7118393, 7.8449750};
float _ITERM_MAX; // 적분기 와인드업 웅앵웅

void setup() {//====================================================================================================
  
    pinMode(PIN_LED,OUTPUT);   //[3030]LED를 연결[3027]
    myservo.attach(PIN_SERVO); //[3039]servo를 연결
    
    // 전역 변수 초기화.
    dist_min = _DIST_MIN;       //[3030] 측정값의 최소값(측정가능 거리의 최소값)
    dist_max= _DIST_MAX;        //[3032] 측정값의 최대값(측정가능 거리의 최대값)
    dist_target = _DIST_TARGET; //[3023] 목표 거리 위치
    duty_neutral=_DUTY_NEU;
    duty_target = _DUTY_NEU;
    error_curr = 200;
    error_prev = 100;
    _ITERM_MAX = 50;
    
    myservo.writeMicroseconds(_DUTY_NEU);
    Serial.begin(57600);
    
    duty_chg_per_interval =(float)(_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE) * (_INTERVAL_SERVO / 1000.0); //[3039] 
   
}  
//====================================================================================================
void loop() {
   unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
     last_sampling_time_dist += _INTERVAL_DIST;
     event_dist = true; 
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
     last_sampling_time_servo += _INTERVAL_SERVO;
     event_servo = true;
   }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
   last_sampling_time_serial += _INTERVAL_SERIAL;
   event_serial = true; 
  }
  
 if(event_dist) { 
  
    event_dist = false;
    float x = ir_distance_filtered();
    dist_ema = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
  
    // PID control logic
    error_curr = dist_target - dist_ema; //[3034] 목표위치와 실제위치의 오차값 
    pterm =  _KP*error_curr;
    dterm = _KD *(error_curr - error_prev);
    iterm += _KI * error_curr;
//    if(abs(iterm) > _ITERM_MAX) iterm = 0; 
    if(iterm > _ITERM_MAX) iterm = _ITERM_MAX;
    if(iterm < - _ITERM_MAX) iterm = - _ITERM_MAX; 
    control = pterm + dterm + iterm;
    duty_target =  _DUTY_NEU + control;
 }

 if(event_servo){
   event_servo = false;
   if(duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; // lower limit
   if(duty_target > _DUTY_MAX) duty_target = _DUTY_MAX; // upper limit
    error_prev = error_curr;
    duty_curr = duty_target;

   myservo.writeMicroseconds(duty_curr);//[3034] 
 }
 
  if(event_serial){
    event_serial = false;
     Serial.print("IR:");
     Serial.print(dist_ema);
     Serial.print(",T:");
     Serial.print(dist_target);
     Serial.print(",P:");
     Serial.print(map(pterm,-1000,1000,510,610));
     Serial.print(",D:");
     Serial.print(map(dterm,-1000,1000,510,610));
     Serial.print(",I:");
     Serial.print(map(iterm,-1000,1000,510,610));
     Serial.print(",DTT:");
     Serial.print(map(duty_target,1000,2000,410,510));
     Serial.print(",DTC:");
     Serial.print(map(duty_curr,1000,2000,410,510));
     Serial.println(",-G:245,+G:265,m:0,M:800");
      
  }
}

float ir_distance(void){ // return value unit: mm
  float val; //[3031] 
  float volt = float(analogRead(PIN_IR)); //[3031]
  val = ((6762.0/(volt-9.0))-4.0) * 10.0; //[3031]
  return val; //[3031]
}

float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    delayMicroseconds(1500);
    
  }
  return largestReading;
}

float ir_distance_filtered(void){ // return value unit: mm
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
}
