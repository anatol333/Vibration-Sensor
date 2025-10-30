
// serial debug on/off: debugFinishSound() - inside function;  debugIR() - call function line ~626

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "arduinoFFT.h"
#include <Preferences.h>
/*
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc.h"
#include "driver/rtc_io.h"

#include "esp_sleep.h"
*/
Preferences prefs;

 // #define IR_SHOW 100  // uncomment to see LED blinking for IR
//  #define SOUND_SHOW 100  // uncomment to see LED blinking for sound

 int IR_outPin = 6;  //5
 int IR_inPin = 1;
 int ledPin_R = 8;  //6
 int ledPin_G = 2;
// int ledPin_G = 8;
 int batPin = 3;
 int modePin = 7;
  #define CHANNEL 0  //analog input
   //#define ledPin_R 3
    #define INTERRUPT_PIN 4

static const NimBLEAdvertisedDevice* advDevice;
static bool                          doConnect  = false;
static uint32_t                      scanTimeMs = 300; /** scan time in milliseconds, 0 = scan forever */

uint32_t timer = 0;
uint32_t timer2 = 0;
uint32_t timer3 = 0;
const unsigned long totalTimout = 5000;

const NimBLEUUID service_1_uuid = NimBLEUUID ("ABCD");
const NimBLEUUID chMessage_uuid = NimBLEUUID ("A1B2");
String message_1 = ("si123s");
String chSound = ("N");
String chIR = ("N");
String batV = ("123");
String chState = ("9");

////////////////////////////////////////////////////////////////////////////////////////// variables for detector:
const uint16_t samples = 1024; //This value MUST ALWAYS be a power of 2
uint16_t inputFrequencyIndex;
const uint16_t numberOfFrequensiesToJoin = 25;
const uint16_t outputLength = 22; // must be ~= 0.5 samples / numberOfFrequensiesToJoin  init=410; 8192samples; F=20000 Hz
uint16_t outputIndex = 0;
float outputDataFloat[outputLength];
byte outputDataNormalized_0[outputLength];
byte outputDataNormalized_1[outputLength];
byte outputDataNormalized_2[outputLength];
byte outputDataNormalized_3[outputLength];
byte outputDataNormalized_4[outputLength];
byte outputDataNormAverage[outputLength];  // average of 0...1 sub spectrums
byte outputDataCalibrAverage[outputLength];  // average of 2 calibrating records
byte recordedDataEEPROM[outputLength];
float levelRecordedEEPROM = 3;
const float samplingFrequency = 5000; //Hz, must be less than 10000 due to ADC
unsigned int sampling_period_us;
unsigned long microseconds;
unsigned long timeValue = 0;
float outputValue = 0;
unsigned long startDelay = 1000; // ms
byte serialRX = 0;
float maxLevel = 0;  // of the current interval
float recordedMaxLevel_0 = 0;
float recordedMaxLevel_1 = 0;
float recordedMaxLevel_2 = 0;
float recordedMaxLevel_3 = 0;
float recordedMaxLevel_4 = 0;
float levelToRecord = 0;
int toleranceSpectrum = 700;  // 400
int differenceSpectrum = 0;
float toleranceLevel = 5.0;   // for average  2.0
// float toleranceLevel_1 = 2.0;  // for one interval
bool levelOK = 0;
byte level_1_OK = 0;
bool spectrumOK = 0;
bool totalOK = 0;
byte calibrationMode = 0;  // 0- normal; 1- waiting for 1st calibration sound; 2 - starting deadtime till tank fills; 3- waiting for 2nd calibration sound
int inputValue = 111;
int minLevelInst = 4095;
int maxLevelInst = 0;
int peakToPeak = 0;
const int tapThreshold = 4000;
const unsigned long minPause = 300;

const unsigned long blinkingPeriodMode2 = 1E6;
const int blinkingNumber = 30;  //till the tank fills
RTC_DATA_ATTR int bootCount = 0;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
float vReal[samples];
float vImag[samples];

/* Create FFT object with weighing factor storage */
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, samples, samplingFrequency, true);

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03
#define SCL_FREQUENCY_2 0x04

 int batReading = 0;
 int Vx100 = 1;
 const float kBAT = 1081; //Vx100 = 100 * batReading / kBAT;
 bool hardMode = 0; // 1 - normal function; 0 - debug.
///////////////////////////////////////////////////////////////////////////////////////////////// VARIABLES FOR IR:
 const unsigned long IRledPin_RDuration = 200; //microseconds; +80us for 3 ADC reading
 int IRreceivedLevel = 3;
 int reading_1;
 int reading_2;
 int reading_3;
 int levelBefore = 0;
 int levelAfter = 0;
 int IRrxLevIRef = 80;
 int IRrxLevIRefINIT = 11;
 int IRrxLevDifference = 0;
 const int triggerMargin = 8;
 int recheckNumber = 0;
 int triggerNumber = 0;
 const int repeatPulseNumber = 3;
 const unsigned long repeatPulseInterval = 100; //milliseconds
 const uint64_t irBlinkPeriod = 3E6; // us
 bool IRdetected = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// FUNCTIONS for BLUETOOTH
/**  None of these are required as they will be handledPin_R by the library with defaults. **
 **                       Remove as you see fit for your needs                        */
class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) override { Serial.printf("Connected_24\n"); }

    void onDisconnect(NimBLEClient* pClient, int reason) override {
        Serial.printf("%s Disconnected, reason = %d - Starting scan\n", pClient->getPeerAddress().toString().c_str(), reason);
        NimBLEDevice::getScan()->start(scanTimeMs, false, true);
    }

    /********************* Security handledPin_R here *********************/
    void onPassKeyEntry(NimBLEConnInfo& connInfo) override {
        Serial.printf("Server Passkey Entry\n");
        /**
         * This should prompt the user to enter the passkey displayed
         * on the peer device.
         */
        NimBLEDevice::injectPassKey(connInfo, 123456);
    }

    void onConfirmPasskey(NimBLEConnInfo& connInfo, uint32_t pass_key) override {
        Serial.printf("The passkey YES/NO number: %" PRIu32 "\n", pass_key);
        /** Inject false if passkeys don't match. */
        NimBLEDevice::injectConfirmPasskey(connInfo, true);
    }

    /** Pairing process complete, we can check the results in connInfo */
    void onAuthenticationComplete(NimBLEConnInfo& connInfo) override {
        if (!connInfo.isEncrypted()) {
            Serial.printf("Encrypt connection failedPin_R - disconnecting\n");
            /** Find the client with the connection handle provided in connInfo */
            NimBLEDevice::getClientByHandle(connInfo.getConnHandle())->disconnect();
            return;
        }
    }
} clientCallbacks;

/** Handles the provisioning of clients and connects / interfaces with the server */
bool connectToServer() {
    NimBLEClient* pClient = nullptr;

    /** Check if we have a client we should reuse first **/
    if (NimBLEDevice::getCreatedClientCount()) {
        /**
         *  Special case when we already know this device, we send false as the
         *  second argument in connect() to prevent refreshing the service database.
         *  This saves considerable time and power.
         */
        pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
        if (pClient) {
            if (!pClient->connect(advDevice, false)) {
                Serial.printf("Reconnect failedPin_R\n");
                return false;
            }
            Serial.printf("Reconnected client\n");
        } else {
            /**
             *  We don't already have a client that knows this device,
             *  check for a client that is disconnected that we can use.
             */
            pClient = NimBLEDevice::getDisconnectedClient();
        }
    }

    /** No client to reuse? Create a new one. */
    if (!pClient) {
        if (NimBLEDevice::getCreatedClientCount() >= NIMBLE_MAX_CONNECTIONS) {
            Serial.printf("Max clients reached - no more connections available\n");
            return false;
        }

        pClient = NimBLEDevice::createClient();

        Serial.printf("New client created\n");

        pClient->setClientCallbacks(&clientCallbacks, false);
        /**
         *  Set initial connection parameters:
         *  These settings are safe for 3 clients to connect reliably, can go faster if you have less
         *  connections. Timeout should be a multiple of the interval, minimum is 100ms.
         *  Min interval: 12 * 1.25ms = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 150 * 10ms = 1500ms timeout
         */
        pClient->setConnectionParams(12, 12, 0, 150);

        /** Set how long we are willing to wait for the connection to complete (milliseconds), default is 30000. */
        pClient->setConnectTimeout(5 * 1000);

        if (!pClient->connect(advDevice)) {
            /** Created a client but failedPin_R to connect, don't need to keep it as it has no data */
            NimBLEDevice::deleteClient(pClient);
            Serial.printf("FailedPin_R to connect, deleted client\n");
            return false;
        }
    }

    if (!pClient->isConnected()) {
        if (!pClient->connect(advDevice)) {
            Serial.printf("FailedPin_R to connect\n");
            return false;
        }
    }

    Serial.printf("Connected to: %s RSSI: %d\n", pClient->getPeerAddress().toString().c_str(), pClient->getRssi());

    /** Now we can read/write/subscribe the characteristics of the services we are interested in */
    NimBLERemoteService*        pSvc = nullptr;
    NimBLERemoteCharacteristic* pChr = nullptr;
    NimBLERemoteDescriptor*     pDsc = nullptr;

    pSvc = pClient->getService(service_1_uuid);
    if (pSvc) {
        pChr = pSvc->getCharacteristic(chMessage_uuid);
    }

    if (pChr) {
        if (pChr->canRead()) {
            Serial.printf("%s Value: %s\n", pChr->getUUID().toString().c_str(), pChr->readValue().c_str());
        }

        if (pChr->canWrite()) {
            if (pChr->writeValue(message_1)) {
                Serial.printf("Wrote new value to: %s\n", pChr->getUUID().toString().c_str());
            } else {
                pClient->disconnect();
                return false;
            }

            if (pChr->canRead()) {
                Serial.printf("The value of: %s is now: %s\n", pChr->getUUID().toString().c_str(), pChr->readValue().c_str());
            }
        }

        if (pChr->canNotify()) {
            if (!pChr->subscribe(true, notifyCB)) {
                pClient->disconnect();
                return false;
            }
        } else if (pChr->canIndicate()) {
            /** Send false as first argument to subscribe to indications instead of notifications */
            if (!pChr->subscribe(false, notifyCB)) {
                pClient->disconnect();
                return false;
            }
        }
    } else {
        Serial.printf("ABCD service not found.\n");
    }

    Serial.printf("Done with this device!\n");
    return true;
}

/** Define a class to handle the callbacks when scan events are received */
class ScanCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
        Serial.printf("Advertised Device found: %s\n", advertisedDevice->toString().c_str());
        if (advertisedDevice->isAdvertisingService(NimBLEUUID(service_1_uuid))) {
            Serial.printf("Found Our Service\n");
            /** stop scan before connecting */
            NimBLEDevice::getScan()->stop();
            /** Save the device reference in a global for the client to use*/
            advDevice = advertisedDevice;
            /** Ready to connect now */
            doConnect = true;
        }
    }

    /** Callback to process the results of the completed scan or restart it */
    void onScanEnd(const NimBLEScanResults& results, int reason) override {
        Serial.printf("Scan Ended, reason: %d, device count: %d; Restarting scan\n", reason, results.getCount());
        NimBLEDevice::getScan()->start(scanTimeMs, false, true);
    }
} scanCallbacks;

/** Notification / Indication receiving handler callback */
void notifyCB(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    std::string str  = (isNotify == true) ? "Notification" : "Indication";
    str             += " from ";
    str             += pRemoteCharacteristic->getClient()->getPeerAddress().toString();
    str             += ": Service = " + pRemoteCharacteristic->getRemoteService()->getUUID().toString();
    str             += ", Characteristic = " + pRemoteCharacteristic->getUUID().toString();
    str             += ", Value = " + std::string((char*)pData, length);
    Serial.printf("%s\n", str.c_str());
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////// FUNCTIONS FOR DETECTOR
void PrintVector(float *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    float abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
	break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
	break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
	break;
        case SCL_FREQUENCY_2:
        abscissa = ((i * 0.5 * samplingFrequency) / outputLength);
	break;
    }
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY || SCL_FREQUENCY_2)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}

void joinFrequencies(){
  outputIndex = 0;
  inputFrequencyIndex = 0;
  while(inputFrequencyIndex < samples/2){
    uint16_t currentIndex;
    outputValue = vReal[inputFrequencyIndex];
    for(int i=1; i < numberOfFrequensiesToJoin; i++){
      currentIndex = inputFrequencyIndex + i;
      outputValue = outputValue + vReal[currentIndex];
 //     Serial.print("vreal="); Serial.println(vReal[currentIndex]);
    }
  outputValue = outputValue / numberOfFrequensiesToJoin;
  outputDataFloat[outputIndex] = outputValue;
  inputFrequencyIndex = inputFrequencyIndex + numberOfFrequensiesToJoin;
  outputIndex++;
 }
}

void normalizeLevels(byte *outArray){
  maxLevel = 0;
  for(int i=1; i < outputLength; i++){
    if(outputDataFloat[i] > maxLevel){maxLevel = outputDataFloat[i];}
  }
  for(int i=1; i < outputLength; i++){
 //   outputDataNormalized[i] = (outputDataFloat[i] * 255) / maxLevel;
    outArray[i] = (outputDataFloat[i] * 255) / maxLevel;
  }
}

void oneIntervalRecord(){
    for(int i=0; i<samples; i++)
  {
      vReal[i] = analogRead(CHANNEL);
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
  }
  /* Print the results of the sampling according to time */
 // Serial.println("Data:");
 // PrintVector(vReal, samples, SCL_TIME);
  //timeValue = micros();
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);	/* Weigh data */
 // Serial.println("Weighed data:");
 // PrintVector(vReal, samples, SCL_TIME);
  FFT.compute(FFTDirection::Forward); /* Compute FFT */
 // Serial.println("Computed Real values:");
//  PrintVector(vReal, samples, SCL_INDEX);
//  Serial.println("Computed Imaginary values:");
 // PrintVector(vImag, samples, SCL_INDEX);
  FFT.complexToMagnitude(); /* Compute magnitudes */
  if(levelRecordedEEPROM / (toleranceLevel * 1.7) < maxLevel && maxLevel < levelRecordedEEPROM * toleranceLevel * 1.7){level_1_OK++;}
}

void debugFinishSound(){

if(!Serial){
    Serial.begin(115200);
    delay(3000);
  }
  Serial.print("calibrationMode = "); Serial.println(calibrationMode);
  Serial.print("levelRecordedEEPROM = "); Serial.println(levelRecordedEEPROM);
 // Serial.print("levelRecordedEEPROM_check = "); Serial.println(levelRecordedEEPROM_check);
  Serial.print("levelToRecord = "); Serial.println(levelToRecord);
  Serial.print("toleranceSpectrum = "); Serial.println(toleranceSpectrum);
  Serial.print("toleranceLevel = "); Serial.println(toleranceLevel);
  Serial.print("time for calculations (us)= "); Serial.println(timeValue);
  Serial.print("batReading= "); Serial.println(batReading);
  Serial.print("Vx100= "); Serial.println(Vx100);
  Serial.print("hardMode= "); Serial.println(hardMode);
    while(1){
    if(Serial.available()){
      serialRX = Serial.read();
      if(serialRX == 48){
        for(int i=1; i<outputLength; i++){
        Serial.println(outputDataNormalized_0[i]);
        }
        Serial.println(recordedMaxLevel_0);
      }
      if(serialRX == 49){
        for(int i=1; i<outputLength; i++){
        Serial.println(outputDataNormalized_1[i]);
        }
        Serial.println(recordedMaxLevel_1);
      }
      if(serialRX == 50){
        for(int i=1; i<outputLength; i++){
        Serial.println(outputDataNormalized_2[i]);
        }
        Serial.println(recordedMaxLevel_2);
      }
      if(serialRX == 51){
        for(int i=1; i<outputLength; i++){
        Serial.println(outputDataNormalized_3[i]);
        }
        Serial.println(recordedMaxLevel_3);
      }
      if(serialRX == 52){
        for(int i=1; i<outputLength; i++){
        Serial.println(outputDataNormalized_4[i]);
        }
        Serial.println(recordedMaxLevel_4);
      }
      if(serialRX == 101){       //"e" - eeprom
        for(int i=1; i<outputLength; i++){
        Serial.println(recordedDataEEPROM[i]);
        }
        Serial.println(levelRecordedEEPROM);
      }
        if(serialRX == 97 ){       //"a" - average
        for(int i=1; i<outputLength; i++){
        Serial.println(outputDataNormAverage[i]);
        }
        Serial.println(recordedMaxLevel_0);
        Serial.println(recordedMaxLevel_1);
        Serial.println(recordedMaxLevel_2);
        Serial.println(recordedMaxLevel_3);
        Serial.println(recordedMaxLevel_4);
        Serial.println(differenceSpectrum);

        Serial.print("levelOK "); Serial.println(levelOK);
        Serial.print("level_1_OK "); Serial.println(level_1_OK);
        Serial.print("spectrumOK "); Serial.println(spectrumOK);
        Serial.print("totalOK "); Serial.println(totalOK);
      }
      Serial.read();
  }

  if(millis() > totalTimout){                                    //if debug
    Serial.println("Timout, sleep from serial!!!");
    sleep();
   }
 } //end of while(1)
}

void messageString(){
  switch(totalOK){
    case 0:
     chSound = ("N");
  break;
    case 1:
     chSound = ("Y");
  break;
    default:
     chSound = ("?");
  break;
  }

  switch(IRdetected){
    case 0:
     chIR = ("N");
  break;
    case 1:
     chIR = ("Y");
  break;
  }

  switch(calibrationMode){
   case 0:
     chState = ("0");
  break;
   case 1:
     chState = ("1");
  break;
   case 2:
     chState = ("2");
  break;
   case 3:
     chState = ("3");
  break;
  default:
     chState = ("?");
  break;
  }
  message_1 = (chSound + chIR + batV + chState);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// FUNCTIONS FOR IR:
void oneBlink(){
  reading_1 = analogRead(IR_inPin);
  reading_2 = analogRead(IR_inPin);
  reading_3 = analogRead(IR_inPin);
  levelBefore = (reading_1 + reading_2 + reading_3) /3;
  digitalWrite(IR_outPin, HIGH);
  delayMicroseconds(IRledPin_RDuration);
  reading_1 = analogRead(IR_inPin);
  reading_2 = analogRead(IR_inPin);
  reading_3 = analogRead(IR_inPin);
  digitalWrite(IR_outPin, LOW);
  levelAfter = (reading_1 + reading_2 + reading_3) /3;
  IRreceivedLevel = IRreceivedLevel + (levelAfter - levelBefore);
}

void blinkIR(){
  IRreceivedLevel = 0;
  IRdetected = 0;
  oneBlink();
  IRrxLevDifference = IRreceivedLevel - IRrxLevIRef;
  if(IRrxLevDifference > 7){IRrxLevIRef++;}
  if(IRrxLevDifference < -7){IRrxLevIRef--;}
  prefs.begin("data_1");
  prefs.putInt("refLev", IRrxLevIRef);
  prefs.end();

  if(IRreceivedLevel > IRrxLevIRef + triggerMargin){
    recheckNumber++;
    IRreceivedLevel = 0;
    for(int i = 0; i < repeatPulseNumber; i++){
      oneBlink();
      delay(repeatPulseInterval);  //last time not needed!
    }
    IRreceivedLevel = IRreceivedLevel / repeatPulseNumber;
    if(IRreceivedLevel > IRrxLevIRef + triggerMargin){
      triggerNumber++;
      IRdetected = 1;
      #ifdef IR_SHOW
      digitalWrite(ledPin_R, HIGH); delay(400); digitalWrite(ledPin_R, LOW);
      #endif
     }
  }
}

 void debugIR(){
    Serial.begin(115200);
    delay(3000);
    Serial.println(__FILE__);
  Serial.print("IRrxLevIRefINIT = "); Serial.println(IRrxLevIRefINIT);
  Serial.print("IRrxLevIRef = "); Serial.println(IRrxLevIRef);
  Serial.print("levelBefore = "); Serial.println(levelBefore);
 // Serial.print("levelAfter = "); Serial.println(levelAfter);
  Serial.print("IRreceivedLevel = "); Serial.println(IRreceivedLevel);
 // Serial.print("recheckNumber = "); Serial.println(recheckNumber);
//  Serial.print("triggerNumber = "); Serial.println(triggerNumber);
  delay(300);
 }
 
 void sleep(){
  if(calibrationMode != 2){esp_deep_sleep_enable_gpio_wakeup(1 << INTERRUPT_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);}
  if(calibrationMode == 0){esp_sleep_enable_timer_wakeup(irBlinkPeriod);}
  if(calibrationMode == 2){esp_sleep_enable_timer_wakeup(blinkingPeriodMode2);}  // waiting for tank to fill
  // rtc_gpio_isolate(GPIO_NUM_6); //was not declared in this scope   also with  #include "esp_sleep.h"
 // gpio_hold_en(GPIO_NUM_6);  //no effect
//   gpio_deep_sleep_hold_en();
   gpio_hold_en(static_cast<gpio_num_t>(6));
   gpio_hold_en(static_cast<gpio_num_t>(2));
   gpio_hold_en(static_cast<gpio_num_t>(8));
  // gpio_hold_en(static_cast<gpio_num_t>(5));
 // esp_sleep_config_gpio_isolate();  // 1.5V on pin 5
   gpio_deep_sleep_hold_en();
  esp_deep_sleep_start();
 }
  void sleepSmart(){
   // debugIR();
   // debugFinishSound();
  sleep();  
 }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// SETUP
void setup() {
  pinMode(modePin, INPUT_PULLDOWN);
 /* 
  gpio_hold_dis(static_cast<gpio_num_t>(6));  // https://www.reddit.com/r/esp32/comments/1dhh5ez/esp32c3_pin_goes_high_on_deep_sleep/
  gpio_hold_dis(static_cast<gpio_num_t>(2));  // here cause blinking on gpio 2, 8 at IR wakeup
  gpio_hold_dis(static_cast<gpio_num_t>(8));
  */
  pinMode(ledPin_R, OUTPUT);  pinMode(ledPin_G, OUTPUT); // pinMode(ledPin_G, OUTPUT);
  digitalWrite(ledPin_R, HIGH);  digitalWrite(ledPin_G, HIGH); // digitalWrite(ledPin_G, HIGH);   

  gpio_hold_dis(static_cast<gpio_num_t>(6));  // https://www.reddit.com/r/esp32/comments/1dhh5ez/esp32c3_pin_goes_high_on_deep_sleep/
  gpio_hold_dis(static_cast<gpio_num_t>(2));  // here is ok
  gpio_hold_dis(static_cast<gpio_num_t>(8));
  //gpio_hold_dis(static_cast<gpio_num_t>(5));

  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(IR_outPin, OUTPUT);
  digitalWrite(IR_outPin, LOW);
//  pinMode(batPin, INPUT);
 // Serial.begin(115200);
 // delay(2000);
  batReading = analogRead(batPin);
  Vx100 = 100 * batReading / kBAT;
  batV = (Vx100);
 // delay(100);
  hardMode = digitalRead(modePin);

  prefs.begin("data_1");
  calibrationMode = prefs.getChar("state"); 
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
  ///////////////////////////////////////////////////////////////////////////////////////////FOR INFRARED:
   case ESP_SLEEP_WAKEUP_TIMER:
   #ifdef IR_SHOW
    digitalWrite(ledPin_R, HIGH); delay(200); digitalWrite(ledPin_R, LOW); delay(200); digitalWrite(ledPin_R, HIGH); delay(200); digitalWrite(ledPin_R, LOW); delay(200);
  #endif
 //   prefs.begin("data_1");
    if(calibrationMode == 2){  // blinking
      if(bootCount < blinkingNumber){
        bootCount++;
        delay(300); digitalWrite(ledPin_R, 0); delay(300); digitalWrite(ledPin_R, 1);
        sleepSmart();
       }
      calibrationMode = 3;
      bootCount = 0;
      goto bluetooth;
     }
    IRrxLevIRef = prefs.getInt("refLev", 99);
    prefs.end();
    IRrxLevIRefINIT = IRrxLevIRef;
    blinkIR();
  
 if(IRdetected == 0){sleepSmart ();} 
 break;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  FOR SOUND:
   default:
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  if(calibrationMode == 0){
    prefs.getBytes("spectrum_1", recordedDataEEPROM, sizeof(outputDataNormAverage));      // spectrum_1 - currently used; spectrum_2 - 1st of a new calibration
    levelRecordedEEPROM = prefs.getFloat("level");
  }
  if(calibrationMode == 3){ 
    prefs.getBytes("spectrum_2", recordedDataEEPROM, sizeof(outputDataNormAverage));      // spectrum_1 - currently used; spectrum_2 - 1st of a new calibration
    levelRecordedEEPROM = prefs.getFloat("level_2");
  }
  prefs.end();

  delay(150); // to finish 1st tap
  while(millis() < startDelay){
    inputValue = analogRead(CHANNEL);
    if(inputValue > maxLevelInst){maxLevelInst = inputValue;}
    if(inputValue < minLevelInst){minLevelInst = inputValue;}
    peakToPeak = maxLevelInst - minLevelInst;
    if(peakToPeak > tapThreshold){
      if(millis() > minPause){        //double tap detected!   
        if(calibrationMode == 0){
          calibrationMode = 1;                                                       // no timer
          }
        else {
          calibrationMode = 0;           // notfall return                           // timer as usual
          }
        digitalWrite(ledPin_G, 0); delay(200); digitalWrite(ledPin_G, 1); delay(300); digitalWrite(ledPin_G, 0); delay(200); digitalWrite(ledPin_G, 1);
        goto bluetooth;
      }
    }
  }
  //SAMPLING
  #ifdef SOUND_SHOW
  digitalWrite(ledPin_R, HIGH);
  # endif
  timeValue = micros();
  microseconds = micros();
  for(int i=0; i <= 4; i++){
    oneIntervalRecord();
    joinFrequencies();
    if(i==0){normalizeLevels(outputDataNormalized_0); recordedMaxLevel_0 = maxLevel;}
    if(i==1){normalizeLevels(outputDataNormalized_1); recordedMaxLevel_1 = maxLevel;}
    if(i==2){normalizeLevels(outputDataNormalized_2); recordedMaxLevel_2 = maxLevel;}
    if(i==3){normalizeLevels(outputDataNormalized_3); recordedMaxLevel_3 = maxLevel;}
    if(i==4){normalizeLevels(outputDataNormalized_4); recordedMaxLevel_4 = maxLevel;}
  }
  levelToRecord = (recordedMaxLevel_0 + recordedMaxLevel_1 + recordedMaxLevel_2 + recordedMaxLevel_3 + recordedMaxLevel_4) / 5;
  for(int i=1; i < outputLength; i++){
      outputDataNormAverage[i] = (outputDataNormalized_0[i] + outputDataNormalized_1[i] + outputDataNormalized_2[i] + outputDataNormalized_3[i] + outputDataNormalized_4[i]) / 5;
    }
  if((levelRecordedEEPROM / toleranceLevel) < levelToRecord && levelToRecord < (levelRecordedEEPROM * toleranceLevel * 1.3)){levelOK = 1;}
//   if(7000 < levelToRecord && levelToRecord < 20000){levelOK = 1;}
  differenceSpectrum = 0;
  for(int i=1; i < outputLength; i++){
    differenceSpectrum = differenceSpectrum + (int)(abs(outputDataNormAverage[i] - recordedDataEEPROM[i]));
    }
  //differenceSpectrum = differenceSpectrum / outputLength;
  digitalWrite(ledPin_R, 1);
  if(differenceSpectrum < toleranceSpectrum){spectrumOK = 1;}
  if(levelOK == 1 && spectrumOK == 1 && level_1_OK >= 4){totalOK = 1;}
  if(totalOK == 1){
    #ifdef SOUND_SHOW
    delay(400); digitalWrite(ledPin_R, HIGH); delay(400); digitalWrite(ledPin_R, LOW);    // confirm match
    #endif
  }
  //ok
  switch (calibrationMode) {
    case 1:
 //   digitalWrite(ledPin_R, HIGH); delay(1500); digitalWrite(ledPin_R, LOW);    no
      calibrationMode = 2;
      prefs.begin("data_1");
      delay(100);
      prefs.putBytes("spectrum_2", outputDataNormAverage, sizeof(outputDataNormAverage));    // spectrum_1 - currently used; spectrum_2 - 1st of a new calibration
      delay(100);
      prefs.putFloat("level_2", levelToRecord);
      delay(100);
      prefs.end();
      digitalWrite(ledPin_R, 0); delay(1000); digitalWrite(ledPin_R, 1);
      break;
    case 2:   //by blinking we don't come here
     break;

    case 3:
      if(totalOK == 1){
        //make average values:
        for(int i=1; i < outputLength; i++){
        outputDataCalibrAverage[i] = (outputDataNormAverage[i] + recordedDataEEPROM[i]) / 2;
        }
        levelToRecord = (levelRecordedEEPROM + levelToRecord) /2;

        calibrationMode = 0;   // return to normal pinMode
         //    prefs.clear();
        prefs.begin("data_1");
        delay(100);
        prefs.putBytes("spectrum_1", outputDataCalibrAverage, sizeof(outputDataCalibrAverage));    // spectrum_1 - currently used; spectrum_2 - 1st of a new calibration
        delay(100);
        prefs.putFloat("level", levelToRecord);
        delay(100);
        prefs.end();
        digitalWrite(ledPin_G, 0); delay(200); digitalWrite(ledPin_G, 1); delay(300); digitalWrite(ledPin_G, 0); delay(200); digitalWrite(ledPin_G, 1);
        digitalWrite(ledPin_G, 0); delay(200); digitalWrite(ledPin_G, 1); delay(300); digitalWrite(ledPin_G, 0); delay(200); digitalWrite(ledPin_G, 1);
        break;
      }
      else {
        digitalWrite(ledPin_R, 0); delay(200); digitalWrite(ledPin_R, 1); delay(300); digitalWrite(ledPin_R, 0); delay(200); digitalWrite(ledPin_R, 1);
        digitalWrite(ledPin_R, 0); delay(200); digitalWrite(ledPin_R, 1); delay(300); digitalWrite(ledPin_R, 0); delay(200); digitalWrite(ledPin_R, 1);}
    }    
  timeValue = micros() - timeValue;
 break;
  }     //  end of switch (wakeup_reason)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// SETUP FOR BLUETOOTH
 bluetooth:
  prefs.begin("data_1");
  delay(100);
  prefs.putChar("state", calibrationMode);
  prefs.end();
  if(IRdetected == 1  || !(totalOK == 0 && calibrationMode == 0)){
    timer = millis();
    messageString();
    /** Initialize NimBLE and set the device name */
    NimBLEDevice::init("NimBLE-Client");

    /**
     * Set the IO capabilities of the device, each option will trigger a different pairing method.
     *  BLE_HS_IO_KEYBOARD_ONLY   - Passkey pairing
     *  BLE_HS_IO_DISPLAY_YESNO   - Numeric comparison pairing
     *  BLE_HS_IO_NO_INPUT_OUTPUT - DEFAULT setting - just works pairing
     */
    // NimBLEDevice::setSecurityIOCap(BLE_HS_IO_KEYBOARD_ONLY); // use passkey
    // NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO); //use numeric comparison

    /**
     * 2 different ways to set security - both calls achieve the same result.
     *  no bonding, no man in the middle protection, BLE secure connections.
     *  These are the default values, only shown here for demonstration.
     */
    // NimBLEDevice::setSecurityAuth(false, false, true);
    // NimBLEDevice::setSecurityAuth(BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_MITM | BLE_SM_PAIR_AUTHREQ_SC);

    /** Optional: set the transmit power */
    NimBLEDevice::setPower(3); /** 3dbm */
    NimBLEScan* pScan = NimBLEDevice::getScan();

    /** Set the callbacks to call when scan events occur, no duplicates */
    pScan->setScanCallbacks(&scanCallbacks, false);

    /** Set scan interval (how often) and window (how long) in milliseconds */
    pScan->setInterval(100);
    pScan->setWindow(100);

    /**
     * Active scan will gather scan response data from advertisers
     *  but will use more energy from both devices
     */
    pScan->setActiveScan(true);

    /** Start scanning for advertisers */
    pScan->start(scanTimeMs);
    Serial.printf("Scanning for peripherals\n");

    NimBLEScanResults results = pScan->getResults(10 * 1000);
    NimBLEUUID serviceUuid("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
for (int i = 0; i < results.getCount(); i++) {
    const NimBLEAdvertisedDevice *device = results.getDevice(i);
    if (device->isAdvertisingService(serviceUuid)) {
        NimBLEClient *pClient = NimBLEDevice::createClient();
        if (pClient->connect(&device)) {
            //success
            Serial.println("connected");
        } else {
            // failedPin_R to connect
            Serial.println("not connected");
        }
    }
  }
 }    // end of if(IRdetected == 1  || !(totalOK == 0 && calibrationMode == 0))
 else {sleepSmart ();}
}    // end of setup
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  LOOP
void loop() {
    /** Loop here until we find a device we want to connect to */
    delay(10);


    if (doConnect) {
        doConnect = false;
        // Found a device we want to connect to, do it now 
        if (connectToServer()) {
            timer2 = millis();
            timer3 =timer2  - timer;
//Serial.print("timer3= "); Serial.println(timer3);
 //           Serial.printf("Success! sleep!!!\n");
//            debugFinishSound();
            sleepSmart ();
        } else {
 //           Serial.printf("FailedPin_R to connect, starting scan???\n");
        }

//        NimBLEDevice::getScan()->start(scanTimeMs, false, true);
    }

  if(millis() > totalTimout){                                    //if debug
    sleepSmart ();
   }
}
