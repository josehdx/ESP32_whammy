#include <Arduino.h>
#include <Control_Surface.h>
#include <AH/Hardware/MultiPurposeButton.hpp>
#include <TFT_eSPI.h>

//=================================================================================
#define SERIAL_BAUDRATE           115200
#define FORCE_CENTER_UPDATE_DELAY 250  

#define BUTTON_SLEEP 0
#define BUTTON_WAKE  0
#define BATTERY_PIN 34
#define BUTTON_SCREEN_OFF 35
#define SLEEP_TIME 120000

// --- FreeRTOS Shared Variables ---
volatile uint16_t sharedPBValue = 8192; 
volatile bool sharedScreenOn = true;   
volatile int sharedBatteryPct = -1;    
volatile bool sharedBTConnected = false;
volatile bool forceBTUpdateState = true;
volatile bool forceScreenUpdate = false; 

TaskHandle_t DisplayTask = NULL;

// --- MIDI & Sensor Objects ---
pin_t pinPB = A15;
int channelShift = 0; 
TFT_eSPI tft = TFT_eSPI();

BluetoothMIDI_Interface btmidi; 
Bank<16> bankChannel;
Bankable::PBPotentiometer potPB {{bankChannel, BankType::ChangeChannel}, pinPB, Channel_1};
FilteredAnalog<12, 14, uint32_t, uint32_t> filterPB = pinPB;

// --- Pitch Bend Calibration Variables ---
double PBdeadzoneMultiplier = 14.0; // Increased to clear -16 jitter
double PBdeadzoneMinimum = 950;     // Raised floor for absolute center stability
double PBdeadzoneMaximum = 1600;   

analog_t PBminimumValue = 0;
analog_t PBmaximumValue = 16383;
analog_t PBcenter = 8192;
analog_t PBdeadzone = PBdeadzoneMinimum; 

bool PBwasOffCenter = false;
long PBlastCenteredOn = millis();
unsigned long lastActivityTime = 0;

//=================================================================================
// DEBUG FUNCTION (Restored from single-core logic)
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
  Serial.print(PBmaximumValue-PBminimumValue);
  Serial.print(" ");
  Serial.print(PBdeadzone); Serial.print(" ");
  Serial.print("PB OffCenter/Raw(14bit)/Val(12bit): ");
  Serial.print(PBwasOffCenter); Serial.print("\t");
  Serial.print(potPB.getRawValue());  Serial.print("\t");
  Serial.print(potPB.getValue()); Serial.print("\t");
  Serial.println();
}

//=================================================================================
// SLEEP & POWER ORCHESTRATION
//=================================================================================
void enterLightSleep() {
  tft.writecommand(TFT_DISPOFF);
  tft.writecommand(TFT_SLPIN);  
  digitalWrite(TFT_BL, LOW);

  esp_light_sleep_start();

  // --- WAKE UP HAPPENS HERE ---
  while (digitalRead(BUTTON_WAKE) == LOW) { delay(10); }
  
  tft.writecommand(TFT_SLPOUT);
  delay(120);                  
  tft.writecommand(TFT_DISPON); 
  digitalWrite(TFT_BL, HIGH);
  sharedScreenOn = true;
  
  lastActivityTime = millis();
  Serial.println("System Restored.");
}

void handlePowerOrchestrator() {
  unsigned long inactiveDuration = millis() - lastActivityTime;
  if (digitalRead(BUTTON_SLEEP) == LOW) {
    while (digitalRead(BUTTON_SLEEP) == LOW) { delay(10); }
    Serial.println("Manual Command: Entering Light Sleep...");
    enterLightSleep(); 
    return;
  }

  if (inactiveDuration > SLEEP_TIME) {
    if (!btmidi.isConnected()) {
      Serial.println("Inactivity: Entering Light Sleep...");
      enterLightSleep(); 
    } else {
      lastActivityTime = millis();
    }
  }
}

//=================================================================================
// CORE 0: SYSTEM & UI TASK
//=================================================================================
void systemTaskCode(void * parameter) {
  uint16_t localPB = 0xFFFF;
  int localBat = -2;
  bool localBT = false;
  unsigned long lastBatCheck = 0;
  static long batteryRawSum = 0;
  static int batterySampleCount = 0;

  for(;;) {
    if (batterySampleCount < 20) {
      batteryRawSum += analogRead(BATTERY_PIN);
      batterySampleCount++;
    } else if (millis() - lastBatCheck > 10000 || sharedBatteryPct == -1) {
      lastBatCheck = millis();
      int rawValue = batteryRawSum / 20;
      
      // Serial.print("RAW BATTERY READING: ");
      // Serial.println(rawValue);

      int pct = map(rawValue, 1750, 2480, 0, 100);
      sharedBatteryPct = constrain(pct, 0, 100);
      batteryRawSum = 0;
      batterySampleCount = 0;
    }

    if (sharedScreenOn) {
      if (sharedBTConnected != localBT || forceBTUpdateState) {
        tft.setTextColor(sharedBTConnected ? TFT_GREEN : TFT_ORANGE, TFT_BLACK);
        tft.setTextDatum(TC_DATUM);
        tft.setTextPadding(200);
        tft.drawString(sharedBTConnected ? "BT: Connected" : "BT: Waiting...", tft.width() / 2, 10, 4);
        localBT = sharedBTConnected;
        forceBTUpdateState = false;
      }

      if (sharedPBValue != localPB || forceScreenUpdate) {
        int displayValue = (int)sharedPBValue - 8192;
        char pbStr[25]; 
        if (displayValue == 0) sprintf(pbStr, "      0         ");
        else sprintf(pbStr, "%+d        ", displayValue);
        
        tft.setTextColor(TFT_CYAN, TFT_BLACK);
        tft.setTextDatum(TL_DATUM);
        tft.setTextPadding(220); 
        tft.drawString(pbStr, (tft.width() / 2) + 5, 50, 4);
        localPB = sharedPBValue;
        forceScreenUpdate = false;
      }

      if (sharedBatteryPct != localBat) {
        tft.setTextColor(sharedBatteryPct > 20 ? TFT_GREEN : TFT_RED, TFT_BLACK);
        tft.setTextDatum(TL_DATUM);
        tft.setTextPadding(80);
        tft.drawString(String(sharedBatteryPct) + "%", (tft.width() / 2) + 5, 90, 4);
        localBat = sharedBatteryPct;
      }
    }
    vTaskDelay(2 / portTICK_PERIOD_MS); 
  }
}

//=================================================================================
// CORE 1: MIDI LOGIC FUNCTIONS
//=================================================================================
analog_t map_PB(analog_t raw) {
    raw = constrain(raw, PBminimumValue, PBmaximumValue);
    
    if (raw < (PBcenter - PBdeadzone)) {
      PBwasOffCenter = true;
      return map(raw, PBminimumValue, PBcenter - PBdeadzone, 0, 8191);
    }
    else if (raw > (PBcenter + PBdeadzone)) {
      PBwasOffCenter = true;
      return map(raw, PBcenter + PBdeadzone, PBmaximumValue, 8192, 16383);
    }
    return 8192; 
}

void calibrateCenterAndDeadzone() {
  Serial.println("Calibrating Center and Deadzones...");
  Serial.println("Please Wait...Do Not Touch Stick!");
  int iNumberOfSamples = 1000;
  analog_t calibPBLow = 4095;
  analog_t calibPBHigh = 0;
  long lSampleSumPB = 0;

  Serial.print("Sampling center. Number of samples: "); Serial.println(iNumberOfSamples);

  pinMode(pinPB, INPUT);
  for (int iSample = 0; iSample < iNumberOfSamples; iSample++) {
    analog_t calibPB = analogRead(pinPB); delay(1);
    lSampleSumPB += calibPB;
    if (calibPB < calibPBLow) { calibPBLow=calibPB; }
    if (calibPB > calibPBHigh) { calibPBHigh=calibPB; }
  }
  
  PBcenter = map((lSampleSumPB / iNumberOfSamples), 0, 4095, 0, 16383);
  Serial.print("PB Center: "); Serial.println(PBcenter);

  PBcenter=constrain(PBcenter, PBminimumValue, PBmaximumValue);
  analog_t calibPBLowMidi = map(calibPBLow, 0, 4095, 0, 16383);
  analog_t calibPBHighMidi = map(calibPBHigh, 0, 4095, 0, 16383);

  Serial.print("PB Low MIDI: "); Serial.println(calibPBLowMidi);
  Serial.print("PB High MIDI: "); Serial.println(calibPBHighMidi);
  
  PBdeadzone = (analog_t) ( ( ( calibPBHighMidi - calibPBLowMidi ) * PBdeadzoneMultiplier) );
  Serial.print("PB Deadzone: "); Serial.println(PBdeadzone);

  PBdeadzone = constrain(((calibPBHighMidi - calibPBLowMidi) * PBdeadzoneMultiplier), PBdeadzoneMinimum, PBdeadzoneMaximum);
  Serial.print("PB Deadzone (Constrained/Value Used): "); Serial.println(PBdeadzone);
}

void adjustPB() {
  uint32_t pbGetRawValue = potPB.getRawValue();
  analog_t pbMapRawValue = map_PB(pbGetRawValue);
  static uint16_t lastSentValue = 0xFFFF;
  static uint32_t lastRaw = 0;

  sharedPBValue = pbMapRawValue;
  
  if (pbMapRawValue == 0 || pbMapRawValue == 8192 || pbMapRawValue == 16383) {
      forceScreenUpdate = true;
  }

  if (abs((int)(pbGetRawValue - (int)lastRaw)) > 100) {
    lastActivityTime = millis();
    if (!sharedScreenOn) {
      sharedScreenOn = true;
      digitalWrite(TFT_BL, HIGH);
      tft.writecommand(TFT_DISPON);
    }
    lastRaw = pbGetRawValue;
  }

  if (pbMapRawValue != lastSentValue) {
      Control_Surface.sendPitchBend(Channel(channelShift), (uint16_t)pbMapRawValue);
      lastSentValue = pbMapRawValue;
  }
  
  if (pbMapRawValue == 8192 && PBwasOffCenter) {
    if ((millis() - PBlastCenteredOn) > FORCE_CENTER_UPDATE_DELAY) {
      Control_Surface.sendPitchBend(Channel(channelShift), (uint16_t)8192);
      PBwasOffCenter = false;
      PBlastCenteredOn = millis();
      lastSentValue = 8192;
      Serial.println("[FORCE MIDI CENTER]");
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  delay(1000);
  pinMode(BATTERY_PIN, INPUT);
  pinMode(BUTTON_SCREEN_OFF, INPUT_PULLUP);
  pinMode(BUTTON_SLEEP, INPUT_PULLUP);
  pinMode(BUTTON_WAKE, INPUT_PULLUP);

#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(TR_DATUM);
  tft.drawString("PB: ", tft.width() / 2, 50, 4);
  tft.drawString("Battery: ", tft.width() / 2, 90, 4);

  btmidi.setName("TTGO");
  FilteredAnalog<>::setupADC();
  filterPB.resetToCurrentValue();
  potPB.map(map_PB);
  btmidi.setAsDefault();
  Control_Surface.begin();
  bankChannel.select(channelShift);

  calibrateCenterAndDeadzone();

  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_WAKE, 0);

  xTaskCreatePinnedToCore(systemTaskCode, "SystemTask", 10000, NULL, 1, &DisplayTask, 0);
  lastActivityTime = millis();
}

void loop() {
  Control_Surface.loop();
  adjustPB();
  handlePowerOrchestrator(); 
  sharedBTConnected = btmidi.isConnected();

  static bool lastBtnState = HIGH;
  bool currentBtnState = digitalRead(BUTTON_SCREEN_OFF);
  if (lastBtnState == HIGH && currentBtnState == LOW) {
    sharedScreenOn = !sharedScreenOn;
    digitalWrite(TFT_BL, sharedScreenOn ? HIGH : LOW);
    if (sharedScreenOn) tft.writecommand(TFT_DISPON);
    else tft.writecommand(TFT_DISPOFF);
    delay(200); 
  }
  lastBtnState = currentBtnState;

  yield(); delay(1); 
}