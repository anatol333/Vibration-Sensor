#define LED 3  // or use LED_BUILTIN for on-board LED
#define INTERRUPT_PIN 4
uint8_t inputPin = 0;
uint8_t switchPinA = 1;
uint8_t switchPinB = 2;
RTC_DATA_ATTR int bootCount = 0;
unsigned long startTime;
const unsigned long analazingDuration = 1800;  //in milliseconds
unsigned long startOfPause;
const unsigned long maxPauseDuration = 100;  //140 - too sensetive; 70 - loud sound not detected; 100 - reasonably
unsigned long startOfSignal;
const unsigned long signalDurationCriterea = 1000;
int inputValue = 111;
int threshold = 1977;  // 3.3V / 4095 = 0.806mV; 2360 = 1.9V; 1.3V=1630; 1900=1.52V  Zero=1825
int threshold_0 = 2000;
int threshold_1 = 2075;
int threshold_2 = 2283;
int threshold_3 = 2490;
bool level = 0;  // 0 - below threshold, bad signal ; 1 - above threshold, good signal
bool detectResult = 0;  // 0 - not flush; 1 - flush.
unsigned long goodReadings = 0;
 bool detectingNow = 0;
 bool switchA = 0;
 bool switchB = 0;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////// FUNCTIONS
void onWakeup(){
  digitalWrite(LED, HIGH);
  delay(60);
  digitalWrite(LED, LOW);
  goodReadings = 0; //needed?
  detectingNow = 1;
  detectResult = 1;  //we are optimistic
  startTime = millis();
  while(detectingNow == 1){
    inputValue = analogRead(inputPin);
//    if(inputValue > threshold && level == 0){startOfPause = millis();}
    if(inputValue > threshold){
      startOfPause = millis();    // constantly shifted at good signal

      if(level ==0){           // positive front
        level = 1;
        startOfSignal = millis();
      }
   
      goodReadings++;
      }

    if(millis() - startOfPause > maxPauseDuration){  // pause detected
      level = 0;
      detectResult = 0;
    }
    if(millis()-startOfSignal > signalDurationCriterea && level == 1){  //positive result
      detectResult = 1;
      goto exitDetecting;
      }

    
    if(millis() - startTime > analazingDuration){
      detectingNow = 0;
      }
//        if(goodReadings == 5000 || goodReadings==10000 || goodReadings== 10310){Serial.println(inputValue);}
 //       if(millis() - 300 < startTime < 302 || millis() - 500 < startTime < 502 || millis() - 700 < startTime < 702){Serial.println(inputValue);}
  }   // end  of loop Loop takes 37 us
  exitDetecting:

  if(detectResult == 1){digitalWrite(LED, HIGH);}
    delay(4000);
    digitalWrite(LED, LOW);
  Serial.print("threshold="); Serial.println(threshold);
  Serial.print("goodReadings="); Serial.println(goodReadings);
  Serial.print("detectResult="); Serial.println(detectResult);
  Serial.println(__FILE__);
delay(400);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////// SETUP
void setup() {
  Serial.begin(115200);
//delay(2000);

  pinMode(LED, OUTPUT);
  pinMode(switchPinA, INPUT_PULLUP);
  pinMode(switchPinB, INPUT_PULLUP);

  switchA = digitalRead(switchPinA);
  switchB = digitalRead(switchPinB);
  if(switchA == 0 && switchB == 0){threshold = threshold_0;}
  if(switchA == 0 && switchB == 1){threshold = threshold_1;}
  if(switchA == 1 && switchB == 0){threshold = threshold_2;}
  if(switchA == 1 && switchB == 1){threshold = threshold_3;}

  onWakeup();
  


//  Serial.println("Going to sleep now");

//  esp_deep_sleep_enable_gpio_wakeup(1 << INTERRUPT_PIN,
//    ESP_GPIO_WAKEUP_GPIO_HIGH);
  esp_deep_sleep_enable_gpio_wakeup(1 << INTERRUPT_PIN,
    ESP_GPIO_WAKEUP_GPIO_LOW);
  //rtc_gpio_isolate(INTERRUPT_PIN);  //was not declared in this scope
 // esp_sleep_config_gpio_isolate();  //compiles but no effect
  esp_deep_sleep_start();

  Serial.println("This will never be printed");
}

void loop() { }