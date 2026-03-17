// 7MM_MIDI_EXAMPLE_JOSE_20260116
// ron.nelson.ii@gmail.com
// http://sevenmilemountain.etsy.com/

// For testing purposes, I'm using a Seeeduino XIAO SAMD21 board.
// These are small and low-cost. You can get versions with WiFI and BLE.
// Most of my production work uses Adafruit QT PY SAMD21 boards -- same form factor

// IMPORTANT : I am using the 2.0.0 version of the Control_Surface surface library.
//             but all other libraries and board definitions are current.

//=================================================================================

#include <Arduino.h>
#include <Control_Surface.h>
#include <AH/Hardware/MultiPurposeButton.hpp>
#include <TFT_eSPI.h>
//=================================================================================

#define SERIAL_BAUDRATE       115200  // all the new boards can handle this speed
#define FORCE_CENTER_UPDATE_DELAY 250   // 0.25 seconds -- only force center updates every y secs (prevent overrun)

//  button pin definitions for sleep/wake functionality. Set to 0 or NO_PIN if you don't want to use this feature. If you do want to use it, set these to the pin numbers of the buttons you want to use for sleeping and waking up the board. You can use the same button for both sleep and wake if you want, just set both to the same pin number.
#define BUTTON_SLEEP 0 //  if you want to use a button to wake up the board from sleep, set this to the pin number of the button. Otherwise, set to 0 or NO_PIN.
#define BUTTON_WAKE  0 // if you want to use a button to wake up the board from sleep, set this to the pin number of the button. Otherwise, set to 0 or NO_PIN.

// --- Battery monitoring variables ---
#define BATTERY_PIN 34 // TTGO boards typically use GPIO 34 for the battery voltage divider
unsigned long lastBatteryUpdate = 0;
int lastDisplayedBat = -1; // Force first draw

int batterySampleCount = 0;
long batteryRawSum = 0;

// --- Screen Toggle variables ---
#define BUTTON_SCREEN_OFF 35 // REMINDER: Pin 35 needs a physical pull-up resistor!
bool isScreenOn = true;      // Tracks if the display is active


// --- Power Management Timers ---
#define SLEEP_TIME 120000        // 2 minutes to Light Sleep
unsigned long lastActivityTime = 0;

// --- Bluetooth State Variables ---
bool lastBTConnectedState = false;
bool forceBTUpdate = true; // Forces the text to draw on the very first loop
//=================================================================================

pin_t pinPB = A15; // yours is 15
int channelShift = 0; // 0 based, so 0 = midi channel 1

TFT_eSPI tft = TFT_eSPI(); 
uint16_t lastDisplayedPB = 0xFFFF;
uint16_t lastSentPBValue = 0xFFFF;
//=================================================================================

//Control_Surface output interfaces 
BluetoothMIDI_Interface btmidi; 
//USBMIDI_Interface usbmidi; 
//HardwareSerialMIDI_Interface serialmidi {Serial1, MIDI_BAUD}; 

Bank<16> bankChannel; 

Bankable::PBPotentiometer potPB {{bankChannel, BankType::ChangeChannel}, pinPB, Channel_1}; // pitch bend

FilteredAnalog<12, 14, uint32_t, uint32_t> filterPB = pinPB;

//=================================================================================

double PBwiggle = 0.050;
double PBdeadzoneMultiplier = 2.50;
double PBdeadzoneMinimum = 387;
double PBdeadzoneMaximum = 539;
double PBdeadzoneLowerShift = 0; 
double PBdeadzoneUpperShift = 0;

//=================================================================================

analog_t PBminimumValue = 0; analog_t PBminimumDefault = PBminimumValue; 
analog_t PBmaximumValue = 16383; analog_t PBmaximumDefault = PBmaximumValue; 
analog_t PBcenter = ((PBmaximumValue-PBminimumValue)/2)+PBminimumValue; 
analog_t PBdeadzone = PBdeadzoneMinimum; 
analog_t PBminReading = PBminimumValue; analog_t PBmaxReading = PBmaximumValue; 

bool PBwasOffCenter = false;
analog_t pbLastRawValue = 8192; // default
long PBlastCenteredOn = millis();

//=================================================================================

analog_t map_PB(analog_t raw) {

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

void calibrateCenterAndDeadzone() {
  
  Serial.println("Calibrating Center and Deadzones...");
  Serial.println("Please Wait...Do Not Touch Stick!");
  
  int iNumberOfSamples = 750; 

  analog_t calibPBLow = 4095; 
  analog_t calibPBHigh = 0; 

  Serial.print("Sampling center. Number of samples: "); Serial.println(iNumberOfSamples);
  
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
void resetActivityTimer() {
  lastActivityTime = millis();
  // Notice we NO LONGER force the screen back on here. 
  // If you turned it off with the button, it stays off!
}

//=================================================================================
void enterLightSleep() {
  tft.writecommand(TFT_DISPOFF); 
  tft.writecommand(TFT_SLPIN);   
  
  digitalWrite(TFT_BL, LOW); // Turn backlight fully off

  esp_light_sleep_start(); 

  // --- WAKE UP HAPPENS HERE ---
  while (digitalRead(BUTTON_WAKE) == LOW) { delay(10); } // Prevent immediate re-sleep
  
  tft.writecommand(TFT_SLPOUT); 
  delay(120);                   
  tft.writecommand(TFT_DISPON);  
  digitalWrite(TFT_BL, HIGH);
  isScreenOn = true;                  // Sync the state
  
  resetActivityTimer(); 
  Serial.println("System Restored.");
}

//=================================================================================
//=================================================================================
void updateScreenBattery() {
  // 1. Collect samples non-blockingly (one per call to the function)
  if (batterySampleCount < 20) {
    batteryRawSum += analogRead(BATTERY_PIN);
    batterySampleCount++;
    return; // Exit and return to the main loop to process MIDI
  }

  // 2. Once 20 samples are collected, check the 10-second timer
  if (millis() - lastBatteryUpdate > 10000 || lastDisplayedBat == -1) {
    lastBatteryUpdate = millis();
    
    int rawValue = batteryRawSum / 20; 
    
    // Reset counters for the next 10-second cycle
    batteryRawSum = 0;
    batterySampleCount = 0;

    //=================================================================================
    // Print to Serial for calibration
    //Serial.print("RAW BATTERY READING: ");
    //Serial.println(rawValue);
    //=================================================================================

    // Map your battery values (adjust the first 2 rawValues below based on your Serial findings)
    int batteryPct = map(rawValue, 1750, 2480, 0, 100);
    batteryPct = constrain(batteryPct, 0, 100);
    //=================================================================================

    if (batteryPct > 20) {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
    } else {
      tft.setTextColor(TFT_RED, TFT_BLACK);
    }

    // This prevents the "Battery:" label (which is right-aligned to center) from being covered.
    tft.setTextDatum(TL_DATUM); 
    
    // Set padding to clear old numbers (3-digit "100" needs about 60-70 pixels)
    tft.setTextPadding(70); 
    
    // Draw the Percentage starting slightly to the right of the screen center (width/2 + 5)
    // and at the same height as your label (y = 90)
    tft.drawString(String(batteryPct) + "%", (tft.width() / 2) + 5, 90, 4); 

    /* Draw the RAW value in the bottom right corner as previously requested
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK); 
    tft.setTextDatum(BR_DATUM); 
    tft.setTextPadding(65); 
    tft.drawString("RAW: " + String(rawValue), tft.width(), tft.height(), 2);  */

    tft.setTextPadding(0); // Always reset padding after drawing
    lastDisplayedBat = batteryPct;
  } else {
    // If 10 seconds haven't passed, reset the sum/count to start a fresh batch next time
    batteryRawSum = 0;
    batterySampleCount = 0;
  }
}

//=================================================================================
//=================================================================================

void updateScreenConnection() {
  // Use Control Surface's built-in connection tracker
  bool currentConnectedState = btmidi.isConnected();

  // Only update the screen if the state changed (or on first boot)
  if (currentConnectedState != lastBTConnectedState || forceBTUpdate) {
    
    tft.setTextDatum(TC_DATUM); 
    
    // Increased padding to 200 to clear the background for the larger Font 4
    tft.setTextPadding(200);    

    if (currentConnectedState) {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      // Changed the '2' at the end to a '4' to match PB and Battery
      tft.drawString("BT: Connected", tft.width() / 2, 10, 4); 
    } else {
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      // Changed the '2' at the end to a '4' to match PB and Battery
      tft.drawString("BT: Waiting...", tft.width() / 2, 10, 4); 
    }

    tft.setTextPadding(0); // Reset padding
    
    lastBTConnectedState = currentConnectedState;
    forceBTUpdate = false;
  }
}
//=================================================================================
//=================================================================================


void updateScreenPB(uint16_t currentPB) {
  if (currentPB != lastDisplayedPB) {
    // Interpolate: Convert MIDI 0 - 16383 to -8192 - 8192
    int displayValue = (int)currentPB - 8192; 
    
    char pbStr[12]; 
    if (displayValue == 0) {
      // Clean zero without a sign
      sprintf(pbStr, "0"); 
    } else {
      // %+d forces the + or - sign for all other numbers
      sprintf(pbStr, "%+d", displayValue); 
    }
    
    tft.setTextColor(TFT_CYAN, TFT_BLACK); 
    tft.setTextDatum(TL_DATUM); 
    
    // Wider padding to handle the sign and 4 digits without clipping
    tft.setTextPadding(110); 
    
    // Aligned with the +5 offset to match the battery label fix
    tft.drawString(pbStr, (tft.width() / 2) + 5, 50, 4); 
    
    tft.setTextPadding(0); 
    lastDisplayedPB = currentPB;
  }
}
// ------------------------------------------------------------------
// ------------------------------------------------------------------
void handleManualScreenOff() {
  static bool lastBtnState = HIGH;
  static bool waitingForRelease = false;
  bool currentBtnState = digitalRead(BUTTON_SCREEN_OFF);

  if (lastBtnState == HIGH && currentBtnState == LOW) {
    waitingForRelease = true;
    delay(20); // Debounce
  }

  // Only trigger on release, and ONLY if the screen is currently ON
  if (waitingForRelease && lastBtnState == LOW && currentBtnState == HIGH) {
    if (isScreenOn) {
        // Turn screen completely OFF
        digitalWrite(TFT_BL, LOW);
        tft.writecommand(TFT_DISPOFF); 
        isScreenOn = false;
    }
    waitingForRelease = false;
  }
  lastBtnState = currentBtnState;
}

// ------------------------------------------------------------------
// ------------------------------------------------------------------
void adjustPB() {
  uint32_t pbGetValue = potPB.getValue(); 
  uint32_t pbGetRawValue = potPB.getRawValue(); 
  analog_t pbMapRawValue = map_PB(pbGetRawValue);
  
  static uint32_t lastRaw = 0;

  // Detect movement to reset timers/wake screen
  if (abs((int)(pbGetRawValue - (int)lastRaw)) > 100) { 
    resetActivityTimer(); 
    if (!isScreenOn) {
      tft.writecommand(TFT_DISPON);
      digitalWrite(TFT_BL, HIGH);
      isScreenOn = true;
    }
    lastRaw = pbGetRawValue;
  }

  if (isScreenOn) {
    updateScreenPB(pbMapRawValue);
  }

  uint16_t valueToSend = pbMapRawValue;

  // Handle your specific edge-case overrides
  if (pbGetValue == 0) { 
    valueToSend = 0;
  } else if (pbGetValue == 8192) { 
    valueToSend = 16383; 
  }

  // Only send if the value has changed OR if we need to force a center update
  if (valueToSend != lastSentPBValue) {
      Control_Surface.sendPitchBend(Channel(channelShift), valueToSend);
      lastSentPBValue = valueToSend;
  }
  
  // Keep your existing Force Center Update logic for the deadzone
  if (pbMapRawValue == 8192 && PBwasOffCenter) {
    if ((millis() - PBlastCenteredOn) > FORCE_CENTER_UPDATE_DELAY) { 
      Control_Surface.sendPitchBend(Channel(channelShift), (uint16_t)8192);
      PBwasOffCenter = false;
      PBlastCenteredOn = millis();
      lastSentPBValue = 8192;
      Serial.println("[FORCE MIDI CENTER]"); 
    }
  }
}

//=================================================================================
/*
void debugPrint() {
  Serial.print("AR: ");
  Serial.print(analogRead(pinPB)); Serial.print("\t");
  Serial.print("CS: "); 
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
}*/

//=================================================================================
//=================================================================================
void handlePowerOrchestrator() {
  unsigned long inactiveDuration = millis() - lastActivityTime;

  // --- MANUAL SLEEP BUTTON CHECK ---
  if (digitalRead(BUTTON_SLEEP) == LOW) {
    while (digitalRead(BUTTON_SLEEP) == LOW) { delay(10); } // Wait for release
    Serial.println("Manual Command: Entering Light Sleep...");
    enterLightSleep(); 
    return; // Exit function after waking up
  }

  // --- AUTOMATIC SLEEP TIMER ---
  if (inactiveDuration > SLEEP_TIME) {
    
    // Check if Bluetooth is currently connected
    if (!btmidi.isConnected()) {
      Serial.println("Inactivity: Entering Light Sleep...");
      enterLightSleep(); 
    } else {
      // We are connected! Intercept the sleep command.
      // Reset the timer to keep the device fully awake and looping.
      lastActivityTime = millis(); 
    }
  }
}
//=================================================================================
//=================================================================================

void setup() {

  Serial.begin(SERIAL_BAUDRATE); 
  delay(3000);
  pinMode(BATTERY_PIN, INPUT);

  pinMode(BUTTON_SCREEN_OFF, INPUT_PULLUP);

  // --- Screen setup on start/reboot ---
  tft.init();
  tft.setRotation(1); 
  tft.fillScreen(TFT_BLACK);

#ifdef TFT_BL
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH); 
  #endif

  lastActivityTime = millis(); // Initialize activity timer

  // Draw the fixed text labels directly to the LEFT of the center
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(TR_DATUM); 
  tft.drawString("PB: ", tft.width() / 2, 50, 4);
  tft.drawString("Battery: ", tft.width() / 2, 90, 4);
  // -----------------------------------------

  btmidi.setName("TTGO"); 

  FilteredAnalog<>::setupADC();
  filterPB.resetToCurrentValue();

  potPB.map(map_PB); 

  btmidi.setAsDefault();

  Control_Surface.begin();

  bankChannel.select(channelShift);

  calibrateCenterAndDeadzone();

  pinMode(BUTTON_SLEEP, INPUT_PULLUP);
  pinMode(BUTTON_WAKE, INPUT_PULLUP);

  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_WAKE, 0);

}//setup

//=================================================================================

void loop() {

  Control_Surface.loop();

  adjustPB(); 

  // Only update battery and connection graphics if the screen is actually on
  if (isScreenOn) {
    updateScreenBattery(); 
    updateScreenConnection();
  }
  
  handlePowerOrchestrator();
  handleManualScreenOff();
  //debugPrint(); 

  yield(); delay(5); 
  
}