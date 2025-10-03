// There are two speedup options for some of the FFT code:

// Define this to use reciprocal multiplication for division and some more speedups that might decrease precision
//#define FFT_SPEED_OVER_PRECISION

// Define this to use a low-precision square root approximation instead of the regular sqrt() call
// This might only work for specific use cases, but is significantly faster. Only works for ArduinoFFT<float>.
//#define FFT_SQRT_APPROXIMATION

#include "arduinoFFT.h"

/*
These values can be changed in order to evaluate the functions
*/
#define CHANNEL 0
#define LED 3

const uint16_t samples = 1024; //This value MUST ALWAYS be a power of 2
uint16_t inputFrequencyIndex;
const uint16_t numberOfFrequensiesToJoin = 25;
const uint16_t outputLength = 22; // must be ~= 0.5 samples / numberOfFrequensiesToJoin  init=410; 8192samples; F=20000 Hz
uint16_t outputIndex = 0;
float outputDataFloat[outputLength];
byte outputDataNormalized[outputLength];
const float samplingFrequency = 5000; //Hz, must be less than 10000 due to ADC
unsigned int sampling_period_us;
unsigned long microseconds;
unsigned long timeValue = 0;
float outputValue = 0;
unsigned long startDelay = 300; // ms

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
//////////////////////////////////////////////////////////////////////////////////////////////////////////// FUNCTIONS
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
  float maxLevel = 0;
  for(int i=1; i < outputLength; i++){
    if(outputDataFloat[i] > maxLevel){maxLevel = outputDataFloat[i];}
  }
  for(int i=1; i < outputLength; i++){
 //   outputDataNormalized[i] = (outputDataFloat[i] * 255) / maxLevel;
    outArray[i] = (outputDataFloat[i] * 255) / maxLevel;
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////// SETUP
void setup()
{
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  delay(350);
//  Serial.println("Ready");
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// LOOP
void loop()
{
  /*SAMPLING*/
  digitalWrite(LED, HIGH);
  delay(startDelay);
  microseconds = micros();
  for(int i=0; i<samples; i++)
  {
      vReal[i] = analogRead(CHANNEL);
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
  }
  digitalWrite(LED, LOW);
  /* Print the results of the sampling according to time */
 // Serial.println("Data:");
 // PrintVector(vReal, samples, SCL_TIME);
  timeValue = micros();
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);	/* Weigh data */
 // Serial.println("Weighed data:");
 // PrintVector(vReal, samples, SCL_TIME);
  FFT.compute(FFTDirection::Forward); /* Compute FFT */
 // Serial.println("Computed Real values:");
//  PrintVector(vReal, samples, SCL_INDEX);
//  Serial.println("Computed Imaginary values:");
 // PrintVector(vImag, samples, SCL_INDEX);
  FFT.complexToMagnitude(); /* Compute magnitudes */
  joinFrequencies();
  normalizeLevels(outputDataNormalized);

  timeValue = micros() - timeValue;
  Serial.println("Computed magnitudes:");
  PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
//  float x = FFT.majorPeak();
//  Serial.println(x, 6); //Print out what frequency is the most dominant.
    Serial.print("time for calculations (us)= "); Serial.println(timeValue);
//  PrintVector(outputDataFloat, outputLength, SCL_FREQUENCY_2);  
  while(!Serial.available());
   for(int i=1; i<outputLength; i++){
    Serial.print(outputDataFloat[i]); Serial.print("    ");
    Serial.println(outputDataNormalized[i]);
 //   delay(10);
   }
  Serial.read();
    Serial.read();
  while(1);

}


