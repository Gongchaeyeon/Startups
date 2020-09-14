
#define PIN_LED 13
unsigned int count, toggle;
static int a;

void setup() {

  pinMode(7, OUTPUT);
  Serial.begin(115200); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }

  Serial.println("Hello World!");
  count = toggle = 0;
  digitalWrite(7, toggle); // turn off LED.
  delay(1000);

}

void loop() {
  
  toggle = toggle_state(toggle); //toggle LED value.
  digitalWrite(7, toggle); // update LED status.
  delay(100); 
}

int toggle_state(int toggle) {
  
  a+=1;  
  
  while (a<=10){
    if (toggle ==0){
      return toggle = 1;  
    }else{
      return toggle = 0;
    }   
  }

  if (a>10){
    return toggle = 1;
  }
}
