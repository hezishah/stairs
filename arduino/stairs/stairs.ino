#include <MIDI.h>
#include <midi_Defs.h>
#include <midi_Message.h>
#include <midi_Namespace.h>
#include <midi_Settings.h>

#include <Wire.h>
#include <VL53L0X.h>

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//#define XSHUT_pin11 not required for address change
#define XSHUT_pin10 49
#define XSHUT_pin9 47
#define XSHUT_pin8 45
#define XSHUT_pin7 43
#define XSHUT_pin6 41
#define XSHUT_pin5 39
#define XSHUT_pin4 37
#define XSHUT_pin3 35
#define XSHUT_pin2 33
#define XSHUT_pin1 31

const int xshutPin[] = {
  -1,
  31,
  33,
  35,
  37,
  39,
  41,
  43,
  45,
  47,
  49
};

//ADDRESS_DEFAULT 0b0101001 or 41

const int sensorNewAddress[] = {
  51,
  50,
  49,
  48,
  47,
  46,
  45,
  44,
  43,
  42,
  41
};

#define NUM_OF_SENSORS 11 /* MAX for now is 11 */
VL53L0X Sensors[NUM_OF_SENSORS];

void rf_setup()
{ /*WARNING*/
  Wire.begin();
  for(int i=0;i<NUM_OF_SENSORS;i++)
  {
    //Shutdown pins of VL53L0X ACTIVE-LOW-ONLY NO TOLERANT TO 5V will fry them
    if(xshutPin[i]>0)
    {
      pinMode(xshutPin[i], OUTPUT);
      digitalWrite(xshutPin[i], 0);
    }
  }
  for(int i=0;i<NUM_OF_SENSORS;i++)
  {
    //Shutdown pins of VL53L0X ACTIVE-LOW-ONLY NO TOLERANT TO 5V will fry them
    if(xshutPin[i]>0)
    {
      pinMode(xshutPin[i], INPUT);
      delay(10); //For power-up procedure t-boot max 1.2ms "Datasheet: 2.9 Power sequence"
      //Change address of sensor and power up next one
    }
    Sensors[i].setAddress(sensorNewAddress[i]);
    Sensors[i].init();
    Sensors[i].setTimeout(500);
#define LONG_RANGE
#if defined LONG_RANGE
    // lower the return signal rate limit (default is 0.25 MCPS)
    Sensors[i].setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    Sensors[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    Sensors[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif
    Sensors[i].startContinuous();
  }
}

static uint16_t lastReading[NUM_OF_SENSORS] = {0};

#define DISTANCE_TO_STAIRS 1850
#define NOISE_MARGIN 150
void rf_loop()
{
  for(int i=0;i<NUM_OF_SENSORS;i++)
  {
    uint16_t reading = Sensors[i].readRangeContinuousMillimeters();
    if(Serial)
    {
      if(i>0)
      {
        Serial.print(',');
      }
      if(reading > DISTANCE_TO_STAIRS) 
      {
        reading = DISTANCE_TO_STAIRS;
      }
      if(reading == 0)
      {
        reading =  lastReading[i];
      }
      Serial.print(reading);
      //Shutdown pins of VL53L0X ACTIVE-LOW-ONLY NO TOLERANT TO 5V will fry them
      if(reading != 0)
      {
        lastReading[i] = reading;
      }
    }
  }
  if(Serial)
    Serial.println("");
}

#endif
/*
  Blink
1
  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/

struct MySettings : public midi::DefaultSettings
{
  static const long BaudRate = 31250;
};
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//Code in here will only be compiled if an Arduino Mega is used.
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial1, MIDI, MySettings);
#else
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial, MIDI, MySettings);
#endif
const int velocity = 90; //Max Velocity (range is 0-127)
const int channel = 1; //MIDI Channel 1 (out of 16)

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//Code in here will only be compiled if an Arduino Mega is used.
  Serial.begin(2000000);
  /* This port will receive sensor data from the small arduino board */
  Serial2.begin(31250);
  /*Call Range Finder Setup Function */
  rf_setup();
#endif
  MIDI.begin();

}

int currentNote=52, previousNote=0;
char midiMessage[10];
int midiInData1Expected = false;
int midiInData2Expected = false;

unsigned int notePitches[NUM_OF_SENSORS] = { 0 };
unsigned long noteTS[NUM_OF_SENSORS] = { 0 };
unsigned int lastNoteReading[NUM_OF_SENSORS] = { 0 };
#define NOTE_DURATION 300

unsigned int scalePitches[] = { 0, 2, 3, 5, 7, 8, 10, 12, 14, 13, 15 };

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  /* Call Range Finder Loop */
  rf_loop();
#endif
  unsigned long mil = millis();
  for(int i=0;i<NUM_OF_SENSORS;i++)
  {
    if(notePitches[i]>0)
    {
      unsigned int dur = mil - noteTS[i];
      if(dur > NOTE_DURATION)
      {
          MIDI.sendNoteOff(notePitches[i], 0, 1);  // Turn the note off.
          notePitches[i] = 0;
      }
    }
    else
    {
      if(lastReading[i] < (DISTANCE_TO_STAIRS - NOISE_MARGIN ))
      {
        int quantizedReading = lastReading[i] / (DISTANCE_TO_STAIRS/3);
        {
          quantizedReading-=1;
          if(quantizedReading != lastNoteReading[i])
          {
              lastNoteReading[i] = quantizedReading;
              /*For now, do not use the quantized value of the actual distance from the sensor*/
              notePitches[i] = 48 /*+ quantizedReading*/ + scalePitches[i];
              MIDI.sendNoteOn(notePitches[i], velocity, 1);  // Turn the note on.
              noteTS[i] = mil;
          }
        }
      }
      else
      {
        lastNoteReading[i] = 4; /*Just give it a unique value*/ 
      }
    }
  }
  if (currentNote >= 93)
  {
    currentNote = 52;
  }
  currentNote++;
  //MIDI.sendNoteOn(currentNote, velocity, channel);  // Turn the note on.
  //MIDI.sendNoteOff(previousNote, 0, channel);  // Turn the note off.

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//Code in here will only be compiled if an Arduino Mega is used.
  while (Serial2.available())
  {
    unsigned char c = Serial2.read();  //gets one byte from serial buffer
    if(c & 0x80)
    {
      midiMessage[0] = c;
      if((c & 0xF0) != 0xF0)
      {
        midiInData1Expected = true;
      }
      else
      {
        //Serial1.write(midiMessage,1);
      }
    }
    else
    {
      if(midiInData1Expected)
      {
        midiMessage[1]=c;
        //MIDI.sendNoteOn(currentNote, velocity, channel);  // Turn the note on.
        //MIDI.sendNoteOff(previousNote, 0, channel);  // Turn the note on.
        midiInData1Expected = false;
        midiInData2Expected = true;
      }
      else
      {
        if(midiInData2Expected)
        {
          midiMessage[2]=c;
          Serial1.write(midiMessage,3);
          midiInData2Expected = false;
        }
      }
    }
  }
#endif
  previousNote = currentNote;
  //Serial.println('.');
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
}


