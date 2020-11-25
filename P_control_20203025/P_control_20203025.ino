#include <Servo.h> // 서보를 쓰기 위해 넣음.

// Arduino pin assignment------------------------------------------------------------
#define PIN_LED 9     // [1234] LED를 GPIO 9번 핀에 연결
#define PIN_SERVO 10  //[3030]SERVO -> 10번 핀에 연결
#define PIN_IR A0     //[3037]적외선 센서 -> A0에 연결

// Framework setting------------------------------------------------------------
#define _DIST_TARGET 250 // [3040] 레일플레이트의 중간지점(목표지점=25.5cm)
#define _DIST_MIN 100     //[3035] 측정 최소 거리
#define _DIST_MAX 410    //[3036] 측정 가능한 최대 거리

// Servo range ------------------------------------------------------------
#define _DUTY_MIN 1000 //[3028] 서보 각도 최소값
#define _DUTY_NEU 1450 //[3038] 레일 수평 서보 펄스폭
#define _DUTY_MAX 2000 //[3031] 서보 최대값

// Servo speed control------------------------------------------------------------
#define _SERVO_ANGLE 30 //[3030] servo angle limit 실제 서보의 동작크기
                        //[3023] 서보모터의 작동 범위(단위 : degree)
#define _SERVO_SPEED 30 //[3040] 서보의 각속도(초당 각도 변화량)

// Event periods------------------------------------------------------------
#define _INTERVAL_DIST 20   //[3039]적외선 센서 측정 간격
#define _INTERVAL_SERVO 20         //[3046]서보갱신간격
#define _INTERVAL_SERIAL 100       //[3030]시리얼 플로터 갱신간격

// PID parameters------------------------------------------------------------
#define _KP 0.06    //[3039] 비례 제어의 상수 값

//EMA -------------------------------------------------------------
#define DELAY_MICROS  1500 // 필터에 넣을 샘플값을 측정하는 딜레이(고정값!)
#define EMA_ALPHA 0.35     // EMA 필터 값을 결정하는 ALPHA 값. 작성자가 생각하는 최적값임.

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

void setup() {//====================================================================================================
//  LED의 GPIO 핀을 초기화하고 서보를 부착합니다
pinMode(PIN_LED,OUTPUT);   //[3030]LED를 연결[3027]
myservo.attach(PIN_SERVO); //[3039]servo를 연결

// 전역 변수 초기화.
dist_min = _DIST_MIN;       //[3030] 측정값의 최소값(측정가능 거리의 최소값)
dist_max= _DIST_MAX;        //[3032] 측정값의 최대값(측정가능 거리의 최대값)
dist_target = _DIST_TARGET; //[3023] 목표 거리 위치
duty_neutral=_DIST_TARGET;

// 서보를 중립 위치로 이동합니다.
myservo.writeMicroseconds(_DUTY_NEU); //[3030]서보를 레일이 수평이 되는 값으로 초기화
Serial.begin(57600);  //[3039] 시리얼 모니터 속도 지정 

// convert angle speed into duty change per interval.
// interval 당 servo의 돌아가는 최대 정도를
  duty_chg_per_interval =(float)(_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE) * (_INTERVAL_SERVO / 1000.0); //[3039] 
//                              (서보 최대값 - 서보 최소값) * (서보 스피드 / 실제 서보의 동작크기) * (서보갱신간격/1000.0)
//_SERVO_ANGLE /_SERVO_SPEED = (_DUTY_MAX - _DUTY_MIN_)* INTERVAL_SERVO/duty_chg_per_interval 
//= interval 반복횟수 * INTERVAL_SERVO = _SERVO_ANGLE만큼 돌아가는데 걸리는 시간 

}  
//====================================================================================================
void loop() {
  dist_raw = ir_distance_filtered();
  event_dist = dist_raw;
  
  if(event_dist) {
    event_dist = false;    // [3037]
    dist_ema = ir_distance_filtered();   // 거리 보정 필터.
    
    // PID control logic
    error_curr = dist_target - dist_ema; //[3034] 목표위치와 실제위치의 오차값 
    pterm =  error_curr*error_curr/error_curr;
    control = _KP * pterm ;
    
    duty_target = duty_neutral + control; //[3034] --> 중립값에서 control만큼 움직임.
    duty_curr = (duty_target + control)*6.6; //[3034] --> 중립값에서 control만큼 움직임.
    // duty_target 값을 [_DUTY_MIN, _DUTY_MAX] 범위 내로 유지
  }
  
   myservo.writeMicroseconds(duty_curr);//[3034] 
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
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
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
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
