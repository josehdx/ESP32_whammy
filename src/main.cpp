
// 7MM_MIDI_EXAMPLE_JOSE_20260116
// ron.nelson.ii@gmail.com
// http://sevenmilemountain.etsy.com/

// For testing purposes, I'm using a Seeeduino XIAO SAMD21 board.
// These are small and low-cost. You can get versions with WiFI and BLE.
// Most of my production work uses Adafruit QT PY SAMD21 boards -- same form factor

// IMPORTANT : I am using the 2.0.0 version of the Control_Surface surface library.
//             but all other libraries and board definitions are current.

//=================================================================================
#define CS_USE_NIMBLE // Use NimBLE instead of Bluedroid for BLE MIDI. NimBLE is more efficient and has better performance, but Bluedroid is more widely supported. If you have issues with one, try the other. Note that if you switch, you may need to change the backend class used in the BLEMIDI_Interface definition below (e.g. to ESP32BluedroidBackend for Bluedroid).

#include <Arduino.h>
#include <Control_Surface.h> // I have the 2.0.0 installed -- I'm not currently using 2.1.0
#include <AH/Hardware/MultiPurposeButton.hpp>
#include "esp_sleep.h"
#include <NimBLEDevice.h>
#include "esp_efuse.h"
#include "esp_system.h"
#include <SPI.h>
#include <TFT_eSPI.h> // Hardware-specific library for TFT display (if you have one, otherwise you can remove this)
#include <WiFi.h>
//=================================================================================

#define SERIAL_BAUDRATE       31250  // all the new boards can handle this speed
#define FORCE_CENTER_UPDATE_DELAY 250   // 0.25 seconds -- only force center updates every y secs (prevent overrun)

//=================================================================================

pin_t pinPB = A13; // your PB potentiometer pin -- this is the analog pin that your PB pot is connected to. On my board, this is A13, which is a 14 bit ADC pin. Check your board's pinout and documentation to find the right one for you. Note that some boards have multiple ADCs with different resolutions, so make sure to choose a pin that supports the resolution you want to work with (e.g. 12 bit, 14 bit, etc.).
int channelShift = 0; // 0 based, so 0 = midi channel 1

//=================================================================================
TFT_eSPI tft = TFT_eSPI(); // If you have a TFT display, you can use this for debugging output. If not, you can remove all references to tft and the TFT_eSPI library.
const int buttonPin = 0;          // GPIO0 is the right button on the T-Display
const int backlightPin = TFT_BL;; // GPIO4 controls the screen backlight
//=================================================================================

//Control_Surface output interfaces 
//In this example code, we can output as BT MIDI, USB MIDI, and Serial MIDI (DIN-5) all at the same time.

BluetoothMIDI_Interface btmidi; // output midi to bluetooth -- my test board doesn't have BT, so this is remarked out.
USBMIDI_Interface usbmidi; // output midi to usb
HardwareSerialMIDI_Interface serialmidi {Serial1, MIDI_BAUD}; // output to serial port for DIN-5 work.

MIDI_PipeFactory<3> pipes; // pipes allows you to output to multiple interfaces at the same time.

//This was very confusing for me for a while, but getting Bankable working is very helpful for runtime settings changes.
Bank<16> bankChannel; // Banking allows for runtime channel changes. This sample code doesn't use it, but can.

//Creates a PBPotentiometer object called potPB. It's tied to a bank, so can be bank switched
//It's tied to the pin defined above (pinPB) and DEFAULTS to Channel 1
Bankable::PBPotentiometer potPB {{bankChannel, BankType::ChangeChannel}, pinPB, Channel_1}; // pitch bend

//This sets the filtering, which Control Surface uses to smooth out some of the data
//On my board, it provides 12 bit numbers on the analogRead and I want it to work with 14 bit numbers.
FilteredAnalog<12, 14, uint32_t, uint32_t> filterPB = pinPB;

//=================================================================================

//These are variables for defining the deadzone (the center point of PB pot which we want to ignore)

double PBwiggle = 0.050;
double PBdeadzoneMultiplier = 2.50;
double PBdeadzoneMinimum = 387;
double PBdeadzoneMaximum = 539;
double PBdeadzoneLowerShift = 0; 
double PBdeadzoneUpperShift = 0;

//=================================================================================

//These are variables for managing the sleep/wake button. This is optional, but can be helpful for battery powered devices.
// GPIO35 is an input-only pin, needs external 10k pull-up
#define BUTTON_SLEEP_PIN GPIO_NUM_35 // GPIO35 is the SLEEP button, typically active low (externally pulled high)
#define BUTTON_WAKE_PIN  GPIO_NUM_0  // GPIO0 is the BOOT button, typically active low (internally pulled high)

//=================================================================================


//These are variables for managing the Pitch Bend values. Highs, lows, defaults, centers, etc.

analog_t PBminimumValue = 0; analog_t PBminimumDefault = PBminimumValue; // These should be measured actual 14 bit values.
analog_t PBmaximumValue = 16383; analog_t PBmaximumDefault = PBmaximumValue; // These should be measured actual 14 bit values.
analog_t PBcenter = ((PBmaximumValue-PBminimumValue)/2)+PBminimumValue; // default (rough) value
analog_t PBdeadzone = PBdeadzoneMinimum; // default (rough) value -- this will be updated during setup
analog_t PBminReading = PBminimumValue; analog_t PBmaxReading = PBmaximumValue; // for auto-ranging

bool PBwasOffCenter = false;
analog_t pbLastRawValue = 8192; // default
long PBlastCenteredOn = millis();

//=================================================================================
//=================================================================================
//=================================================================================

analog_t map_PB(analog_t raw) {

    //NON INVERTING -- invert (swap 0,8191 and 16383,8191) if your pot goes in the opposite direction
    //In other words, you should invert or non-invert based in the orientation of your PB pot.
    //DO NOT USE THE INVERT FUNCTION found in the Control Surface 2.0.0 library -- it OVERRIDES MAP!!!

    analog_t result = 0;

    raw = constrain(raw, PBminimumValue, PBmaximumValue);
    
    if (raw <= PBcenter-PBdeadzone-PBdeadzoneLowerShift) {
      result = map(raw, PBminimumValue, PBcenter-PBdeadzone-PBdeadzoneLowerShift, 0, 8191);
      PBwasOffCenter = true;
    }
    else if (raw >= PBcenter+PBdeadzone+PBdeadzoneUpperShift) {
      result = map(raw, PBcenter+PBdeadzone+PBdeadzoneUpperShift, PBmaximumValue, 8191, 16383);
      PBwasOffCenter = true;
    }
    else {
      result = 8192;
    }

    return result;
}

//=================================================================================
//=================================================================================

void calibrateCenterAndDeadzone() {
  
  //IMPORTANT: As the Calibration has been called AFTER CONTROL SURFACE
  //           the values returned by analogRead will be the FILTERS range.
  //           We get 12 bit (0 to 4095) values!

  Serial.println("Calibrating Center and Deadzones...");
  Serial.println("Please Wait...Do Not Touch Stick!");
  
  //Determine joystick center points
  int iNumberOfSamples = 750; 

  //Defaults to inverse so we don't have a fake mid-point
  analog_t calibPBLow = 4095; 
  analog_t calibPBHigh = 0; 

  Serial.print("Sampling center. Number of samples: "); Serial.println(iNumberOfSamples);
  
  //Determine center point of PB
  pinMode(pinPB, INPUT);
  long lSampleSumPB = 0;
  for (int iSample = 1; iSample<=iNumberOfSamples; iSample++) {
    analog_t calibPB = analogRead(pinPB); delay(1);
    lSampleSumPB += calibPB;
    if (calibPB < calibPBLow) { calibPBLow=calibPB; } 
    if (calibPB > calibPBHigh) { calibPBHigh=calibPB; } 
  }
  
  PBcenter=map((analog_t(lSampleSumPB/iNumberOfSamples)), 0, 4095, 0, 16383);
  Serial.print("PB Center: "); Serial.println(PBcenter);

  //2025-05-26 : New safety feature, so that PBcenter never over above or below existing limits
  PBcenter=constrain(PBcenter, PBminimumValue, PBmaximumValue);

  analog_t calibPBLowMidi = map(calibPBLow, 0, 4095, 0, 16383);
  analog_t calibPBHighMidi = map(calibPBHigh, 0, 4095, 0, 16383);
  Serial.print("PB Low MIDI: "); Serial.println(calibPBLowMidi);
  Serial.print("PB High MIDI: "); Serial.println(calibPBHighMidi);
 
  PBdeadzone = (analog_t) ( ( ( calibPBHighMidi - calibPBLowMidi ) * PBdeadzoneMultiplier) );
  Serial.print("PB Deadzone: "); Serial.println(PBdeadzone);
  PBdeadzone = (analog_t) ( constrain((( calibPBHighMidi - calibPBLowMidi ) * PBdeadzoneMultiplier), PBdeadzoneMinimum, PBdeadzoneMaximum) );
  Serial.print("PB Deadzone (Constrained/Value Used): "); Serial.println(PBdeadzone);  
}

//=================================================================================
//=================================================================================


void adjustPB() {

  //The 12 bit getValue is used only for the continous send on low and high. Otherwise, not needed.
  uint32_t pbGetValue = potPB.getValue(); // This is a 12 bit value (0 to 4095)

  //We need to use the 14 bit full value provided by getRawValue to do the deadzone magic.
  uint32_t pbGetRawValue = potPB.getRawValue(); // This is a 14 bit value (0 to 16383)
  analog_t pbMapRawValue = map_PB(pbGetRawValue);

  //Continuous send on low -- OPTIONAL
  if (pbGetValue==0) { Control_Surface.sendPitchBend(Channel(channelShift) , (uint16_t) 0); }

  //If it was off center, but now back to center, force a zero (center)
  if (pbMapRawValue==8192 && PBwasOffCenter) {
    //Serial.print("RE-CENTER REQUEST...");
    //Throttle this behavior. Say, X times in last Y seconds? (like flash update delay).
    if ( (millis()-PBlastCenteredOn) > (FORCE_CENTER_UPDATE_DELAY) ) { 
      Control_Surface.sendPitchBend(Channel(channelShift) , (uint16_t) 8192);
      PBwasOffCenter = false;
      PBlastCenteredOn = millis();
      Serial.println("[FORCE MIDI CENTER]");
    }
  }

  //continuous send on high -- OPTIONAL
  if (pbGetValue==8192) { Control_Surface.sendPitchBend(Channel(channelShift) , (uint16_t) 16383); }

}//adjustPB

//=================================================================================

void debugPrint() {
  //Optional -- if you want to check on what analogRead is really providing
  //The resolution (7, 10, 12, 14, 16) will depend on your MCUs ADC
  Serial.print("AR: "); // This is the raw analogRead value -- on my board, this is 12 bit (0 to 4095)
  Serial.print(analogRead(pinPB)); Serial.print("\t");
  Serial.print("CS: "); // Channel Shift (channel # - 1)
  Serial.print(channelShift); Serial.print("\t");
  Serial.print("PB Min/Cen/Max/Range/DZ: ");
  Serial.print(PBminimumValue); Serial.print(" ");
  Serial.print(PBcenter); Serial.print(" ");
  Serial.print(PBmaximumValue); Serial.print(" ");
  Serial.print(PBmaximumValue-PBminimumValue); Serial.print(" ");
  Serial.print(PBdeadzone); Serial.print(" ");
  Serial.print("PB OffCenter/Raw(14bit)/Val(12bit): ");
  Serial.print(PBwasOffCenter); Serial.print("\t");
  Serial.print(potPB.getRawValue());  Serial.print("\t");
  Serial.print(potPB.getValue()); Serial.print("\t");
  Serial.println();
}

//=================================================================================

void setup() {
    //For debugging output
  Serial.begin(SERIAL_BAUDRATE); // this is the serial debug baud rate -- NOT MIDI

  Serial.println("\nESP32 Awake");

  // Initialize TFT Screen
    pinMode(backlightPin, OUTPUT);    // MUST set the backlight pin to output mode
  digitalWrite(backlightPin, HIGH); // MUST pull the backlight pin HIGH to turn on the screen
  
  tft.init();
  tft.setRotation(1); 
  tft.fillScreen(TFT_WHITE); 
  tft.setTextColor(TFT_BLACK); 
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("Active");
  

    esp_sleep_enable_ext0_wakeup(BUTTON_WAKE_PIN, 0); 
    pinMode(BUTTON_SLEEP_PIN, INPUT);

  // Configure Wake-up pin (GPIO0)
  // Wake up when button is pressed (LOW)
  esp_sleep_enable_ext0_wakeup(BUTTON_WAKE_PIN, 0); 
  
  // Configure Sleep button (GPIO35)
  pinMode(BUTTON_SLEEP_PIN, INPUT);

  // Check if wakeup reason is from deep sleep
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Woke up from GPIO0");
  }

  //Set the BT MIDI device name -- this is what will show up on your computer or phone when you go to connect to it.
  
  btmidi.setName("TTGO_whammy"); //bt device name  

  //Setup Control_Surface filters
  FilteredAnalog<>::setupADC();
  filterPB.resetToCurrentValue();

  //Setup Control_Surface mapping
  potPB.map(map_PB); // pitch bend - non-inverted

  // Manually connect the MIDI interfaces to Control Surface
  //Control_Surface >> pipes >> btmidi; //  -- my test board doesn't have BT, so this is remarked out.
  //Control_Surface >> pipes >> usbmidi;
  //Control_Surface >> pipes >> serialmidi; 

  //This is the fallback/default method. MIDI will be sent out of anything in the pipes (above)
  //usbmidi.setAsDefault();
  btmidi.setAsDefault();


  //Startup MIDI Control Surface
  Control_Surface.begin();

  //Set the default/startup MIDI Channel
  //Zero based (0 = midi channel 1) -- in my full code this is a runtime variable
  bankChannel.select(channelShift);

  //Centering and deadzone are runtime only (no flash settings)
  calibrateCenterAndDeadzone();


}//setup

//=================================================================================
//=================================================================================
//=================================================================================

void loop() {

  Control_Surface.loop();


  adjustPB(); // Handles re-centering

  // Calculate the corrected PB value (the same logic used in adjustPB)
    uint32_t pbGetRawValue = potPB.getRawValue();
    analog_t pbMapRawValue = map_PB(pbGetRawValue);

    // Display PB info on the screen
    tft.setTextColor(TFT_BLACK, TFT_WHITE); // Set text color and background to overwrite old numbers
    tft.setCursor(10, 40);
    tft.print("PB: ");
    tft.print(pbMapRawValue);
    tft.print("     "); // Trailing spaces to clear longer previous numbers

  debugPrint(); 

  yield(); delay(1); // helpful for chips with watchdog timers, short pause to allow other tasks to catch up
  
   // Check if button on GPIO35 is pressed (LOW)
  if (digitalRead(BUTTON_SLEEP_PIN) == LOW) {
    Serial.println("Going to sleep...");
    
    // Turn off the TFT backlight to save power
    digitalWrite(backlightPin, LOW); 
    
    delay(10);
    
    // Enter Light Sleep instead of Deep Sleep
    esp_light_sleep_start();
    
    // -- CPU IS PAUSED HERE --
    // Upon pressing the GPIO0 button, the code will resume on the very next line:
    
    digitalWrite(backlightPin, HIGH); // Turn the screen backlight back on
    Serial.println("Awake from Light Sleep!");
    
    // Debounce delay to prevent bouncing straight back to sleep
    delay(500);
  }

}//loop

//=================================================================================
//=================================================================================
//=================================================================================