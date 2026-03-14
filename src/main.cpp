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

#define BUTTON_SLEEP 35 //  if you want to use a button to wake up the board from sleep, set this to the pin number of the button. Otherwise, set to 0 or NO_PIN.
#define BUTTON_WAKE  0 // if you want to use a button to wake up the board from sleep, set this to the pin number of the button. Otherwise, set to 0 or NO_PIN.

// --- Battery monitoring variables ---
#define BATTERY_PIN 34 // TTGO boards typically use GPIO 34 for the battery voltage divider
unsigned long lastBatteryUpdate = 0;
int lastDisplayedBat = -1; // Force first draw
// -----------------------------------------
//=================================================================================

pin_t pinPB = A15; // yours is 15
int channelShift = 0; // 0 based, so 0 = midi channel 1

TFT_eSPI tft = TFT_eSPI(); 
uint16_t lastDisplayedPB = 0xFFFF;

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
// --- FLICKER-FREE SCREEN FUNCTIONS ---
void updateScreenBattery() {
  if (millis() - lastBatteryUpdate > 10000 || lastDisplayedBat == -1) {
    lastBatteryUpdate = millis();
    
    long rawSum = 0;
    for (int i = 0; i < 20; i++) {
      rawSum += analogRead(BATTERY_PIN);
      delay(2); 
    }
    int rawValue = rawSum / 20; 

    int batteryPct = map(rawValue, 1985, 2605, 0, 100);
    batteryPct = constrain(batteryPct, 0, 100);

    if (batteryPct != lastDisplayedBat) {
      char batStr[10];
      // No more manual spaces needed!
      sprintf(batStr, "%d%%", batteryPct); 
      
      if (batteryPct > 20) {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
      } else {
        tft.setTextColor(TFT_RED, TFT_BLACK);
      }
      
      tft.setTextDatum(TL_DATUM); 
      
      // --- NEW: Force the background to paint 80 pixels wide ---
      tft.setTextPadding(80); 
      tft.drawString(batStr, tft.width() / 2, 90, 4); 
      tft.setTextPadding(0); // Reset padding so it doesn't affect other things
      // ---------------------------------------------------------
      
      lastDisplayedBat = batteryPct;
    }
  }
}

void updateScreenPB(uint16_t currentPB) {
  if (currentPB != lastDisplayedPB) {
    char pbStr[10]; 
    // Just the raw number, no manual padding needed!
    sprintf(pbStr, "%u", currentPB); 
    
    tft.setTextColor(TFT_CYAN, TFT_BLACK); 
    tft.setTextDatum(TL_DATUM); 
    
    // --- NEW: Force the background to paint 80 pixels wide ---
    tft.setTextPadding(80);
    tft.drawString(pbStr, tft.width() / 2, 50, 4); 
    tft.setTextPadding(0); // Reset padding 
    // ---------------------------------------------------------
    
    lastDisplayedPB = currentPB;
  }
}
// ------------------------------------------------------------------

void adjustPB() {

  uint32_t pbGetValue = potPB.getValue(); 
  uint32_t pbGetRawValue = potPB.getRawValue(); 
  analog_t pbMapRawValue = map_PB(pbGetRawValue);

  updateScreenPB(pbMapRawValue);

  if (pbGetValue==0) { Control_Surface.sendPitchBend(Channel(channelShift) , (uint16_t) 0); }

  if (pbMapRawValue==8192 && PBwasOffCenter) {
    if ( (millis()-PBlastCenteredOn) > (FORCE_CENTER_UPDATE_DELAY) ) { 
      Control_Surface.sendPitchBend(Channel(channelShift) , (uint16_t) 8192);
      PBwasOffCenter = false;
      PBlastCenteredOn = millis();
      Serial.println("[FORCE MIDI CENTER]");
    }
  }

  if (pbGetValue==8192) { Control_Surface.sendPitchBend(Channel(channelShift) , (uint16_t) 16383); }

}

//=================================================================================

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
}

//=================================================================================
void handlePowerManagement() {
  if (digitalRead(BUTTON_SLEEP) == LOW) {
    while (digitalRead(BUTTON_SLEEP) == LOW) {
      delay(10);
    }
    
    Serial.println("Entering Light Sleep...");
    Serial.flush(); 
    
    tft.writecommand(TFT_DISPOFF); 
    tft.writecommand(TFT_SLPIN);   
    #ifdef TFT_BL
      digitalWrite(TFT_BL, LOW);   
    #endif
    
    esp_light_sleep_start();
    
    Serial.println("Woke up from Light Sleep!");

    tft.writecommand(TFT_SLPOUT);  
    delay(120);                    
    tft.writecommand(TFT_DISPON);  
    #ifdef TFT_BL
      digitalWrite(TFT_BL, HIGH);  
    #endif
  }
}

//=================================================================================

void setup() {

  Serial.begin(SERIAL_BAUDRATE); 

  pinMode(BATTERY_PIN, INPUT);

  // --- Screen setup on start/reboot ---
  tft.init();
  tft.setRotation(1); 
  tft.fillScreen(TFT_BLACK);

  #ifdef TFT_BL
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
  #endif

  // Draw "Active" dead center
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextDatum(TC_DATUM); 
  tft.drawString("Active", tft.width() / 2, 10, 4); 

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

  updateScreenBattery(); 

  handlePowerManagement();

  debugPrint(); 

  yield(); delay(5); 
  
}