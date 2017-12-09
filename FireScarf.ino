/*
CPSC 581 Project 1
-~*`Fire Scarf`*~- -  -    -      ~
By Austin Sullivan

FEATURES
----------------------------------
- thermometer which influences max lighting level and heating level
- microphone makes led strip flicker
----------------------------------


LINKS THAT HELPED ME
----------------------------------
Heating Pads hookup:
https://learn.sparkfun.com/tutorials/heating-pad-hand-warmer-blanket?_ga=2.64455462.846225416.1507958936-2066370721.1507958936
Dallas thermometer hookup:
https://create.arduino.cc/projecthub/TheGadgetBoy/ds18b20-digital-temperature-sensor-and-arduino-9cc806
Microphone hookup:
https://learn.adafruit.com/adafruit-microphone-amplifier-breakout/measuring-sound-levels
----------------------------------
*/


#include <OneWire.h>
#include <DallasTemperature.h>
#include <QuickStats.h>


// Ambient temperature / thermometer properties -----------------------------
#define TEMP_POLL_TIME 5000    // duration in milliseconds to wait until ambient temp is measured again
#define ONE_WIRE_BUS 7    // Thermometer data pin is 7
OneWire oneWire(ONE_WIRE_BUS);        // Setup onewire instance to communicate with thermometer
DallasTemperature thermo(&oneWire);   // Pass onewire reference to dallas temp
float ambientTemp;                    // recorded temperature from thermometer


// Heading pad properties -----------------------------
#define HEATPIN 3
// Note: the heating pads use the setMaxPWM for their heat value.
// Using a 12V power supply and using the Vin pin for power, the heating pads heat up
// to roughly 50 degrees Celsius at full duty cycle (pwm 255).


// Light / LED properties -----------------------------
#define LEDPIN 11              // LED strip data pin is 11
float appliedLEDPWM = 0;        // PWM value output from flicker equation
const float flickerFactor = 1.3;   // how much to cut light by when there is wind


// Air / microphone properties -----------------------------
#define AIR_BASELINE_READINGS 20     // how many samples to take to zero the mic
float micIn = 0;                     // current reading from microphone
const float micMax = 3.3;            // I found this by blowing into the mic to see how high the voltage println went
float micBase = 0;                   // A baseline found in setup() (discovers background sound)
const byte airSampleWindow = 50;     // Sample window width in milliseconds (50 mS = 20Hz)  
unsigned int sample;


// Everything else -----------------------------
QuickStats stats;
byte setMaxPWM = 0;                      // changes based on the current ambient temperature
unsigned long lastTempCheckTime = 0;
bool toggled = false;


void setup() {
  Serial.begin(9600);
  thermo.begin();
  updateTempPWM();
  airBaseline();      // run microphone calibration
  
}


//===========================================================
void loop() {
  
  micIn = getAir();          // constantly check for wind

  if(abs(micIn - micBase) < 0.2)      // shine at brightest if no wind
    analogWrite(LEDPIN, setMaxPWM);
  else {                    // else, flicker the LED strip
    // Flicker equation
    appliedLEDPWM = (float)setMaxPWM - (((float)setMaxPWM / flickerFactor) * (micIn / micMax));    // do some magic
    analogWrite(LEDPIN, appliedLEDPWM);
    Serial.println(appliedLEDPWM);
  }

  // Check to see if the ambient temperature reading should be updated
  if((millis() - lastTempCheckTime) >= TEMP_POLL_TIME){
    lastTempCheckTime = millis();
    updateTempPWM();
  }
}
//===========================================================


// Function to get the rest state of the microphone
// -> gets the most commonly occurring voltage output from mic over the configured amount of samples in 500 ms
//--------------------------------------------------------
void airBaseline() {
  float readings[AIR_BASELINE_READINGS];
  for(int i = 0; i < AIR_BASELINE_READINGS; i++){
    readings[i] = getAir();
    //Serial.println(readings[i]);
  }
  micBase = stats.mode(readings, AIR_BASELINE_READINGS, 0.01);
  //Serial.println("micBase: ");
  //Serial.println(micBase);
}
//--------------------------------------------------------


// Function to sample a sound recording.
// Returns higher voltage if louder sound is picked up.
// When blown on, the mic outputs a higher voltage.
//--------------------------------------------------------
float getAir() {
  unsigned long startMillis = millis();  // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;

  // collect data for 50 mS
  while (millis() - startMillis < airSampleWindow)
  {
    sample = analogRead(0);
    if (sample < 1024)                // toss out spurious readings
    {
      if (sample > signalMax)
        signalMax = sample;           // save just the max levels
      else if (sample < signalMin)
        signalMin = sample;           // save just the min levels
    }
  }
  peakToPeak = signalMax - signalMin;         // max - min = peak-peak amplitude
  float volts = (peakToPeak * 5.0) / 1024;    // convert to volts
/*
  Serial.print("Volts from sound: ");
  Serial.print(volts);
  Serial.println();*/
  return volts;
}
//--------------------------------------------------------


// Function to get a temperature reading and thus a PWM range from the thermometer
//--------------------------------------------------------
void updateTempPWM() {
  thermo.requestTemperatures();                // expensive call
  ambientTemp = thermo.getTempCByIndex(0);     // get current temperature
  Serial.println();
  Serial.print("Ambient temp reading: ");
  Serial.print(ambientTemp);
  
  // Check for potentially garbage reading
  if(ambientTemp < -100)
    return;
  
  // Determine the max brightness of the LED strip according to current ambient temperature
  // Becomes brightest when colder than -20!
  if(ambientTemp > 0)
    setMaxPWM = 50;
  else if(ambientTemp > -10 && ambientTemp < 0)
    setMaxPWM = 85;
  else if(ambientTemp > -20 && ambientTemp < -10)
    setMaxPWM = 170;
  else
    setMaxPWM = 255;
  
  analogWrite(HEATPIN, setMaxPWM);
  
  Serial.println();
  Serial.print("Updated new pwm max to be ");
  Serial.print(setMaxPWM);
}
//--------------------------------------------------------
