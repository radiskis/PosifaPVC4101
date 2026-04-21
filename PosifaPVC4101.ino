// Requirements: install CheapLCD library
// Sensors: Posifa PVC4101-C (vacuum pressure), Consensic CPS122 (atmospheric pressure)
// Physical connection: 3.3 V power supply, I2C (SCL, SDA)
// Tested with: Iteaduino UNO (switched with 3.3 V),  1602 LCD Keypad Shield For Arduino

#include <Wire.h>
#include <CheapLCD.h>   
#include <stdlib.h>

#define PVC4000_ADDR 0x50
#define CMD_RAW_DATA 0xD0
#define CMD_CAL_TBL_X 0xD1
#define CMD_CAL_TBL_Y 0xD2

static const uint8_t LCD_COLS = 16;

#ifndef DTOSTR_UPPERCASE
#define DTOSTR_UPPERCASE 0x04
#endif

uint16_t calX[15];
uint16_t calY[15];

// ----- Spinner glyphs -----
byte spin0[8] = {B00100,B00100,B00100,B00100,B00100,B00100,B00100,B00000};
byte spin1[8] = {B00001,B00010,B00100,B01000,B10000,B00000,B00000,B00000};
byte spin2[8] = {B00000,B00000,B11111,B00000,B00000,B00000,B00000,B00000};
byte spin3[8] = {B10000,B01000,B00100,B00010,B00001,B00000,B00000,B00000};

void initSpinnerGlyphs(CheapLCD &lcd) {
  lcd.createChar(0, spin0);
  lcd.createChar(1, spin1);
  lcd.createChar(2, spin2);
  lcd.createChar(3, spin3);
}

uint8_t spinnerIdx = 0;
void lcdSpinnerCustom(CheapLCD &lcd, uint8_t col, uint8_t row) {
  lcd.setCursor(col, row);
  lcd.write(spinnerIdx);                 // show current frame
  spinnerIdx = (spinnerIdx + 1) % 4;     // advance frame
}

void lcdPrintRow(CheapLCD &lcd, uint8_t row, const char *text) {
  char line[LCD_COLS + 1];
  uint8_t i = 0;

  for (; i < LCD_COLS && text[i] != '\0'; i++) {
    line[i] = text[i];
  }
  for (; i < LCD_COLS; i++) {
    line[i] = ' ';
  }
  line[LCD_COLS] = '\0';

  lcd.setCursor(0, row);
  lcd.print(line);
}

void formatMbarRow(char *row, const char *label, float value) {
  char sci[12];
  dtostre(value, sci, 2, DTOSTR_UPPERCASE);
  snprintf(row, LCD_COLS + 1, "%s:%smb", label, sci);
}

void readCalibrationTable(uint8_t cmd, uint16_t *table) {
  Wire.beginTransmission(PVC4000_ADDR);
  Wire.write(cmd);
  Wire.endTransmission();
  delay(10);

  Wire.requestFrom(PVC4000_ADDR, 30); // 15 entries * 2 bytes
  for (int i = 0; i < 15; i++) {
    if (Wire.available() >= 2) {
      uint8_t lsb = Wire.read();
      uint8_t msb = Wire.read();
      table[i] = (msb << 8) | lsb; // little endian
    }
  }
}

float applyCalibration(uint16_t rawValue) {
  for (int i = 1; i < 15; i++) {
    if (rawValue <= calX[i]) {
      float X0 = calX[i - 1];
      float X1 = calX[i];
      float Y0 = calY[i - 1];
      float Y1 = calY[i];
      return ((rawValue - X0) / (X1 - X0)) * (Y1 - Y0) + Y0;
    }
  }
  return calY[14];
}

float micronsToMbar(float microns) {
  return (microns / 1000.0) * 1.33322;
}

// ================= CPS122 (datasheet-based) =================
static const uint8_t CPS122_ADDR  = 0x6D; // 7-bit I2C address
static const uint8_t REG_MEAS_REQ = 0x30; // MR register
static const uint8_t MR_VALUE     = 0x0A; // MR value
static const uint8_t REG_GET_DATA = 0x06; // GD pointer

// ================= LCD (CheapLCD) =================
// Try 0x27 first; if your backpack is 0x3F, change it here.
CheapLCD lcd;  // If your version needs begin(): CheapLCD lcd(0x27);

// ---- CPS122 helpers ----
bool cps122RequestMeasurement() {
  Wire.beginTransmission(CPS122_ADDR);
  Wire.write(REG_MEAS_REQ);
  Wire.write(MR_VALUE);
  uint8_t err = Wire.endTransmission();   // AVR: STOP
  if (err != 0) return false;
  delay(6);                               // ~5 ms (datasheet)
  return true;
}

bool cps122GetData(float &kPa, float &degC, float &mbar,
                   uint32_t &rawP, int16_t &rawT) {
  // Point to GD register
  Wire.beginTransmission(CPS122_ADDR);
  Wire.write(REG_GET_DATA);
  if (Wire.endTransmission() != 0) return false;

  const uint8_t N = 5;
  if (Wire.requestFrom(CPS122_ADDR, N) != N) return false;

  uint8_t p23_16 = Wire.read();
  uint8_t p15_8  = Wire.read();
  uint8_t p7_0   = Wire.read();
  uint8_t t_hi   = Wire.read();
  uint8_t t_lo   = Wire.read();

  rawP = (uint32_t(p23_16) << 16) | (uint32_t(p15_8) << 8) | uint32_t(p7_0);
  rawT = int16_t((uint16_t(t_hi) << 8) | uint16_t(t_lo));  // two's complement

  // Datasheet scaling:
  // Pressure [kPa] = (raw / 2^6) / 1000
  kPa  = (float(rawP) / 64.0f) / 1000.0f;
  // Temperature [°C] = int16 / 2^8
  degC = float(rawT) / 256.0f;
  // mbar = kPa * 10
  mbar = kPa * 10.0f;

  return true;
}




void setup() {
  Serial.begin(9600);        // optional debug; match Serial Monitor to 9600
  delay(100);
  Wire.begin();              // keep default 100 kHz for stability on AVR


  readCalibrationTable(CMD_CAL_TBL_X, calX);
  readCalibrationTable(CMD_CAL_TBL_Y, calY);

  Serial.println("Calibration Table Loaded.");

  // ----- CheapLCD init -----
  lcd.begin();                // if your lib uses begin: lcd.begin(16, 2);
  initSpinnerGlyphs(lcd);
  lcd.backlightOn();
  lcd.clear();
  lcdPrintRow(lcd, 0, "CPS122 + LCD");
  lcdPrintRow(lcd, 1, "Init...");
  delay(600);
  lcd.clear();
}

void loop() {
    uint16_t rawPressure = 0;
  uint16_t rawTemp = 0;

  Wire.beginTransmission(PVC4000_ADDR);
  Wire.write(CMD_RAW_DATA);
  Wire.endTransmission();
  delay(10);

  Wire.requestFrom(PVC4000_ADDR, 6);
  if (Wire.available() == 6) {
    uint8_t checksum = Wire.read();
    uint8_t msbPressure = Wire.read();
    uint8_t lsbPressure = Wire.read();
    Wire.read(); // reserved
    uint8_t msbTemp = Wire.read();
    uint8_t lsbTemp = Wire.read();

    rawPressure = (msbPressure << 8) | lsbPressure;
    rawTemp = (msbTemp << 8) | lsbTemp;

    uint8_t sum = msbPressure + lsbPressure + 0xFF + msbTemp + lsbTemp;
    uint8_t calcChecksum = 1 + (~sum);
    if (calcChecksum == checksum) {
      float calibratedMicrons = applyCalibration(rawPressure);
      float calibratedMbar = micronsToMbar(calibratedMicrons);

      //Serial.print("Raw Pressure: ");
      //Serial.print(rawPressure);
      //Serial.print(" | Calibrated: ");
      Serial.print(calibratedMicrons);
      //Serial.print(" µmHg | ");
      Serial.print("\t");
      Serial.println(calibratedMbar);
      //Serial.print(" mbar | Raw Temp: ");
      //Serial.println(rawTemp);

      char row[LCD_COLS + 1];
      formatMbarRow(row, "Vac", calibratedMbar);
      lcdPrintRow(lcd, 0, row);

      // Line 2: Temperature
      lcd.setCursor(10, 0);
      //lcd.print("T:");
      //lcd.print(C, 1);         // e.g., 23.4
      //lcd.print((char)223);    // Degree symbol on HD44780
      //lcd.print("C");
    } else {
      Serial.println("Checksum error!");
      lcdPrintRow(lcd, 0, "Posifa fail!!!!");
    }
  }

  if (!cps122RequestMeasurement()) {
    //lcd.clear();
    lcdPrintRow(lcd, 1, "MR fail!!!!");
    delay(1000);
    return;
  }

  float kPa, C, mbar;
  uint32_t rawP = 0;
  int16_t rawT = 0;

  if (cps122GetData(kPa, C, mbar, rawP, rawT)) {
    // Line 1: Pressure in mbar (fits better than kPa)
    //lcd.clear();
    char row[LCD_COLS + 1];
    formatMbarRow(row, "Atm", mbar);
    lcdPrintRow(lcd, 1, row);

    // Line 2: Temperature
    lcd.setCursor(10, 1);
    //lcd.print("T:");
    //lcd.print(C, 1);         // e.g., 23.4
    //lcd.print((char)223);    // Degree symbol on HD44780
    //lcd.print("C");

    // Optional: minimal serial debug
    /*Serial.print("P=");
    Serial.print(kPa, 3);
    Serial.print("kPa/");
    Serial.print(mbar, 1);
    Serial.print("mbar T=");
    Serial.print(C, 2);
    Serial.println("C");*/
  } else {
    lcd.clear();
    lcdPrintRow(lcd, 0, "GD fail");
  }
  lcdSpinnerCustom(lcd, 15, 0);
  delay(1000);               // update once per second
}
