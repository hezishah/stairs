#include <MIDI.h>
#include <midi_Defs.h>
#include <midi_Message.h>
#include <midi_Namespace.h>
#include <midi_Settings.h>
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
#endif
  MIDI.begin();
}

int currentNote=52, previousNote=0;
char midiMessage[10];
int midiInData1Expected = false;
int midiInData2Expected = false;

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(100);                       // wait for a second
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//Code in here will only be compiled if an Arduino Mega is used.
#endif
  if (currentNote >= 93)
  {
    currentNote = 52;
  }
  currentNote++;
  MIDI.sendNoteOn(currentNote, velocity, channel);  // Turn the note on.
  MIDI.sendNoteOff(previousNote, 0, channel);  // Turn the note on.

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
}


