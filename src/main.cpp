// 硬件版本：USB 电压电流表
// 软件版本：3.8 (纠正电流算法回归原版 505 标定 + 两点电压线性校准 V-Offset)
// 编写时间：2026-08-19
// MCU：STM32F103C8T6

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// 屏幕初始化 (128x64 I2C OLED)
Adafruit_SSD1306 display(128, 64, &Wire, 4);

// ---------------------- 引脚分配 ----------------------
const byte PIN_CURR_HI = PA1; // 电流采样 (ADC1_IN1，硬件主回路)
const byte PIN_CURR_LO = PA2; // 预留通道 (PA2)
const byte PIN_VOLT_DN = PA3; // D- 电压采样 (ADC1_IN3)
const byte PIN_VOLT_DP = PA0; // D+ 电压采样 (ADC1_IN0)
const byte PIN_NTC_IN  = PA4; // NTC 温度采样 (ADC1_IN4)
const byte PIN_VOLT_IN = PA5; // 输入总电压采样 (ADC1_IN5)

const byte PIN_K1 = PA15;     // 按键 1：切页
const byte PIN_K2 = PB4;      // 按键 2：切组 / 菜单项调节 [-] (支持长按连发)
const byte PIN_K3 = PB10;     // 按键 3：旋转屏幕 / 菜单项调节 [+] (支持长按连发)
const byte PIN_K4 = PB1;      // 按键 4：示波器波形源切换 / 设置项光标切换

const byte PIN_M1 = PB9;      // 大电流通道 MOS 控制 (常开 HIGH)
const byte PIN_M2 = PB8;      // 小电流通道 MOS 控制 (常关 LOW)

// AT24Cxx 外部 EEPROM I2C 地址
const uint8_t AT24C_I2C_ADDR = 0x50;

// ---------------------- 核心数据结构 ----------------------
struct DataGroup {
  float mAh;        // 累计毫安时
  float mWh;        // 累计毫瓦时
  uint32_t seconds; // 累计运行时间(秒)
  float vMax;       // 峰值电压 (V)
  float aMax;       // 峰值电流 (A)
  float wMax;       // 峰值功率 (W)
};

struct SystemConfig {
  uint32_t magic;         // 校验魔数
  uint8_t activeGroup;    // 当前激活的数据组 (0 ~ 3)
  uint8_t screenRot;      // 屏幕旋转 (0: 0度, 2: 180度)
  uint8_t waveMode;       // 示波器波形源 (0: 电压波形, 1: 电流波形)
  uint8_t brightness;     // OLED 亮度对比度 (0 ~ 255)
  float voltCoeff;        // 电压斜率校准系数 (标准默认 12.01f)
  float voltOffset;       // 电压零点偏置补偿 (标准默认 0.00f，用于吸收二极管压降)
  float currCoeff;        // 电流分母系数 (标准默认 505.0f，即 101 * 5)
  uint8_t sleepMin;       // 自动熄屏时间(0:常开, 1:1min, 2:2min, 3:3min, 5:5min, 10:10min)
};

const uint32_t STORAGE_MAGIC = 0x55AA2032;
const uint32_t FLASH_PAGE_ADDRESS = 0x0800FC00; // STM32F103 内部 Flash Page 63

DataGroup groups[4];
SystemConfig sysCfg = { STORAGE_MAGIC, 0, 0, 0, 160, 12.01f, 0.00f, 505.0f, 2 };

// ---------------------- 全局测量变量 ----------------------
float V = 0.0f;       // 输入总电压 (V)
float A = 0.0f;       // 实时电流 (A)
float W = 0.0f;       // 实时功率 (W)
float R = 0.0f;       // 负载等效电阻 (Ω)
float VP = 0.0f;      // D+ 电压 (V)
float VN = 0.0f;      // D- 电压 (V)
float TempC = 0.0f;   // NTC 摄氏温度 (°C)

uint8_t currentPage = 0;   // 当前视图 (0: 主仪表, 1: 快充/网格, 2: 示波器, 3: 能量统计, 4: 设置与校准)
uint8_t settingItem = 0;   // 设置页面光标索引 (0: 亮度, 1: V-Cal, 2: V-Off, 3: I-Cal, 4: Sleep, 5: CableR)

// 线阻测试临时变量
float cableV0 = 0.0f;
float cableR = 0.0f;
bool cableV0Locked = false;

// 息屏与休眠状态
bool isScreenSleeping = false;
uint32_t lastActivityMs = 0;

// 波形环形缓冲区 (60点)
const uint8_t WAVE_BUF_SIZE = 60;
float waveBuf[WAVE_BUF_SIZE];
uint8_t waveIndex = 0;

// 计时与状态
uint32_t lastSecondMs = 0;
uint32_t lastAutoSaveMs = 0;
uint32_t lastSerialMs = 0;
bool heartbeatState = false;
bool eepromDetected = false;

// ---------------------- OLED 亮度与休眠控制 ----------------------
void setOledBrightness(uint8_t contrast) {
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(contrast);
}

void sleepScreen() {
  if (!isScreenSleeping) {
    isScreenSleeping = true;
    display.ssd1306_command(SSD1306_DISPLAYOFF);
  }
}

void wakeUpScreen() {
  if (isScreenSleeping) {
    isScreenSleeping = false;
    display.ssd1306_command(SSD1306_DISPLAYON);
    setOledBrightness(sysCfg.brightness);
    lastActivityMs = millis();
  }
}

// ---------------------- AT24Cxx 驱动函数 ----------------------
void writeEEPROM_Byte(uint16_t addr, uint8_t data) {
  Wire.beginTransmission(AT24C_I2C_ADDR);
  Wire.write((uint8_t)(addr & 0xFF));
  Wire.write(data);
  Wire.endTransmission();
  delay(5);
}

uint8_t readEEPROM_Byte(uint16_t addr) {
  uint8_t data = 0xFF;
  Wire.beginTransmission(AT24C_I2C_ADDR);
  Wire.write((uint8_t)(addr & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom((int)AT24C_I2C_ADDR, 1);
  if (Wire.available()) data = Wire.read();
  return data;
}

void writeEEPROM_Buffer(uint16_t addr, const uint8_t *pData, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    writeEEPROM_Byte(addr + i, pData[i]);
  }
}

void readEEPROM_Buffer(uint16_t addr, uint8_t *pData, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    pData[i] = readEEPROM_Byte(addr + i);
  }
}

bool checkEEPROM() {
  Wire.beginTransmission(AT24C_I2C_ADDR);
  return (Wire.endTransmission() == 0);
}

// ---------------------- 内部 Flash 模拟擦写 ----------------------
void saveFlashBackup() {
  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PageError = 0;
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = FLASH_PAGE_ADDRESS;
  EraseInitStruct.NbPages     = 1;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) == HAL_OK) {
    uint32_t *pSys = (uint32_t *)&sysCfg;
    uint16_t sysWords = sizeof(SystemConfig) / 4;
    for (uint16_t i = 0; i < sysWords; i++) {
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_PAGE_ADDRESS + (i * 4), pSys[i]);
    }
    
    uint32_t *pGrp = (uint32_t *)groups;
    uint16_t grpWords = sizeof(groups) / 4;
    uint32_t grpOffset = FLASH_PAGE_ADDRESS + sizeof(SystemConfig);
    for (uint16_t i = 0; i < grpWords; i++) {
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, grpOffset + (i * 4), pGrp[i]);
    }
  }
  HAL_FLASH_Lock();
}

// ---------------------- 数据加载与双重备份保存 ----------------------
void loadAllData() {
  eepromDetected = checkEEPROM();
  bool loaded = false;

  if (eepromDetected) {
    readEEPROM_Buffer(0, (uint8_t *)&sysCfg, sizeof(SystemConfig));
    if (sysCfg.magic == STORAGE_MAGIC) {
      readEEPROM_Buffer(sizeof(SystemConfig), (uint8_t *)groups, sizeof(groups));
      loaded = true;
    }
  }

  if (!loaded) {
    uint32_t *pFlash = (uint32_t *)FLASH_PAGE_ADDRESS;
    if (*pFlash == STORAGE_MAGIC) {
      memcpy(&sysCfg, (void *)FLASH_PAGE_ADDRESS, sizeof(SystemConfig));
      memcpy(groups, (void *)(FLASH_PAGE_ADDRESS + sizeof(SystemConfig)), sizeof(groups));
      loaded = true;
    }
  }

  if (!loaded) {
    sysCfg.magic = STORAGE_MAGIC;
    sysCfg.activeGroup = 0;
    sysCfg.screenRot = 0;
    sysCfg.waveMode = 0;
    sysCfg.brightness = 160;
    sysCfg.voltCoeff = 12.01f;
    sysCfg.voltOffset = 0.00f;
    sysCfg.currCoeff = 505.0f;
    sysCfg.sleepMin = 2;
    memset(groups, 0, sizeof(groups));
  }
}

void saveAllData() {
  sysCfg.magic = STORAGE_MAGIC;
  
  if (eepromDetected) {
    writeEEPROM_Buffer(0, (const uint8_t *)&sysCfg, sizeof(SystemConfig));
    writeEEPROM_Buffer(sizeof(SystemConfig), (const uint8_t *)groups, sizeof(groups));
  }
  
  saveFlashBackup();
}

// ---------------------- 快充协议检测 ----------------------
const char* getProtocolName() {
  if (VP >= 0.40f && VP <= 0.85f && VN >= 0.40f && VN <= 0.85f) return "QC3.0";
  if (VP >= 2.80f && VP <= 3.60f && VN >= 0.40f && VN <= 0.85f) return "QC2.0 9V/12V";
  if (VP >= 2.40f && VP <= 2.90f && VN >= 2.40f && VN <= 2.90f) return "Apple 2.4A";
  if (VP >= 1.80f && VP <= 2.30f && VN >= 2.40f && VN <= 2.90f) return "Apple 2.1A";
  if (VP >= 2.40f && VP <= 2.90f && VN >= 1.80f && VN <= 2.30f) return "Apple 1.0A";
  if (fabs(VP - VN) < 0.15f && VP < 1.20f && VP > 0.10f)        return "DCP 1.5A";
  if (VP < 0.20f && VN < 0.20f)                                return "DIRECT (No-QC)";
  return "UNKNOWN";
}

const char* getShortProtocolName() {
  if (VP >= 0.40f && VP <= 0.85f && VN >= 0.40f && VN <= 0.85f) return "QC3.0";
  if (VP >= 2.80f && VP <= 3.60f && VN >= 0.40f && VN <= 0.85f) return "QC2.0";
  if (VP >= 2.40f && VP <= 2.90f && VN >= 2.40f && VN <= 2.90f) return "Apple";
  if (VP >= 1.80f && VP <= 2.30f && VN >= 2.40f && VN <= 2.90f) return "Apple";
  if (fabs(VP - VN) < 0.15f && VP < 1.20f && VP > 0.10f)        return "DCP";
  if (VP < 0.20f && VN < 0.20f)                                return "DIRECT";
  return "LIVE";
}

// ---------------------- ADC 采样与数据滤波 (严格基于原作者 505 与两点校准) ----------------------
void doSampling() {
  // 1. D+/D- 电压 (PA0 / PA3)
  float vnRaw = map(analogRead(PIN_VOLT_DN), 0, 4095, 0, 3300);
  VN = (vnRaw * 2.0f) / 1000.0f;
  float vpRaw = map(analogRead(PIN_VOLT_DP), 0, 4095, 0, 3300);
  VP = (vpRaw * 2.0f) / 1000.0f;

  // 2. 总电压 V (PA5，50次均值滤波，支持斜率 voltCoeff 与 偏置 voltOffset)
  float sumV = 0;
  for (byte i = 0; i < 50; i++) {
    float vMv = map(analogRead(PIN_VOLT_IN), 0, 4095, 0, 3300);
    sumV += (vMv * sysCfg.voltCoeff) / 1000.0f;
  }
  V = (sumV / 50.0f) + sysCfg.voltOffset;

  // 3. 电流 A (PA1，50次均值滤波，严格采用标准公式: aMv / 505.0)
  float sumA = 0;
  for (byte i = 0; i < 50; i++) {
    float aMv = map(analogRead(PIN_CURR_HI), 0, 4095, 0, 3300);
    sumA += aMv / sysCfg.currCoeff; // 默认 sysCfg.currCoeff = 505.0f (即 101 * 5)
  }
  A = sumA / 50.0f;

  // 4. 功率 W 与等效电阻 R
  W = V * A;
  if (A > 0.08f) {
    R = V / A;
  } else {
    R = 0.0f;
  }

  // 5. NTC 摄氏度计算 (PA4)
  float ntcMv = map(analogRead(PIN_NTC_IN), 0, 4095, 0, 3300);
  if (3300.0f - ntcMv > 10.0f) {
    float rNtc = (10000.0f * ntcMv) / (3300.0f - ntcMv);
    TempC = (3950.0f * 298.15f) / (3950.0f + (298.15f * log(rNtc / 10000.0f))) - 273.15f - 2.0f;
  }

  // 6. 更新当前组极值
  DataGroup &g = groups[sysCfg.activeGroup];
  if (V > g.vMax) g.vMax = V;
  if (A > g.aMax) g.aMax = A;
  if (W > g.wMax) g.wMax = W;
}

// ---------------------- 1秒定时积分与自动息屏检测 ----------------------
void updateEnergyAndTimer() {
  if (millis() - lastSecondMs >= 1000) {
    lastSecondMs = millis();
    heartbeatState = !heartbeatState;

    DataGroup &g = groups[sysCfg.activeGroup];
    g.seconds++;
    g.mWh += (W * 1000.0f) / 3600.0f;
    g.mAh += (A * 1000.0f) / 3600.0f;

    waveBuf[waveIndex] = (sysCfg.waveMode == 0) ? V : A;
    waveIndex = (waveIndex + 1) % WAVE_BUF_SIZE;

    if (sysCfg.sleepMin > 0 && !isScreenSleeping) {
      if (millis() - lastActivityMs >= ((uint32_t)sysCfg.sleepMin * 60000UL)) {
        sleepScreen();
      }
    }
  }

  if (millis() - lastAutoSaveMs >= 5000) {
    lastAutoSaveMs = millis();
    saveAllData();
  }
}

// ---------------------- 设置项加减核心逻辑 ----------------------
void applySettingChange(bool increase) {
  if (settingItem == 0) { // 亮度
    if (increase) {
      if (sysCfg.brightness <= 225) sysCfg.brightness += 30;
    } else {
      if (sysCfg.brightness > 30) sysCfg.brightness -= 30;
    }
    setOledBrightness(sysCfg.brightness);
  } else if (settingItem == 1) { // 电压斜率微调 (0.02 步进)
    if (increase) sysCfg.voltCoeff += 0.02f;
    else          sysCfg.voltCoeff -= 0.02f;
  } else if (settingItem == 2) { // 电压零点偏置微调 (0.02V 步进，解决二极管压降)
    if (increase) sysCfg.voltOffset += 0.02f;
    else          sysCfg.voltOffset -= 0.02f;
  } else if (settingItem == 3) { // 电流系数微调 (2.0 步进)
    if (increase) sysCfg.currCoeff += 2.0f;
    else          sysCfg.currCoeff -= 2.0f;
  } else if (settingItem == 4) { // 熄屏时间 (OFF -> 1min -> 2min -> 3min -> 5min -> 10min -> OFF)
    if (increase) {
      if (sysCfg.sleepMin == 0) sysCfg.sleepMin = 1;
      else if (sysCfg.sleepMin == 1) sysCfg.sleepMin = 2;
      else if (sysCfg.sleepMin == 2) sysCfg.sleepMin = 3;
      else if (sysCfg.sleepMin == 3) sysCfg.sleepMin = 5;
      else if (sysCfg.sleepMin == 5) sysCfg.sleepMin = 10;
      else sysCfg.sleepMin = 0;
    } else {
      if (sysCfg.sleepMin == 0) sysCfg.sleepMin = 10;
      else if (sysCfg.sleepMin == 10) sysCfg.sleepMin = 5;
      else if (sysCfg.sleepMin == 5) sysCfg.sleepMin = 3;
      else if (sysCfg.sleepMin == 3) sysCfg.sleepMin = 2;
      else if (sysCfg.sleepMin == 2) sysCfg.sleepMin = 1;
      else sysCfg.sleepMin = 0;
    }
  } else if (settingItem == 5) { // 线阻测试
    if (!increase) {
      cableV0 = V;
      cableV0Locked = true;
    } else {
      if (cableV0Locked && A > 0.15f && cableV0 > V) {
        cableR = ((cableV0 - V) / A) * 1000.0f;
      }
    }
  }

  saveAllData();
}

// ---------------------- 按键检测与长按极速连发 ----------------------
struct KeyState {
  byte pin;
  bool lastState;
  uint32_t pressTime;
  uint32_t lastRepeatTime;
  bool longPressed;
  bool isRepeating;
};

KeyState keys[4] = {
  { PIN_K1, HIGH, 0, 0, false, false },
  { PIN_K2, HIGH, 0, 0, false, false },
  { PIN_K3, HIGH, 0, 0, false, false },
  { PIN_K4, HIGH, 0, 0, false, false }
};

void handleButtons() {
  for (uint8_t i = 0; i < 4; i++) {
    bool currState = digitalRead(keys[i].pin);

    // 1. 按键按下瞬间
    if (currState == LOW && keys[i].lastState == HIGH) {
      if (isScreenSleeping) {
        wakeUpScreen();
        keys[i].lastState = currState;
        keys[i].pressTime = millis();
        keys[i].longPressed = true;
        return;
      }

      lastActivityMs = millis();
      keys[i].pressTime = millis();
      keys[i].lastRepeatTime = millis();
      keys[i].longPressed = false;
      keys[i].isRepeating = false;
    }
    // 2. 按键保持按住中 (连发/长按)
    else if (currState == LOW && keys[i].lastState == LOW) {
      lastActivityMs = millis();

      if (currentPage == 4 && (i == 1 || i == 2) && (settingItem == 1 || settingItem == 2 || settingItem == 3)) {
        if (millis() - keys[i].pressTime > 400) {
          keys[i].isRepeating = true;
          if (millis() - keys[i].lastRepeatTime > 70) {
            keys[i].lastRepeatTime = millis();
            applySettingChange(i == 2);
          }
        }
      } else {
        if (!keys[i].longPressed && (millis() - keys[i].pressTime > 1500)) {
          keys[i].longPressed = true;
          if (i == 1) { 
            memset(&groups[sysCfg.activeGroup], 0, sizeof(DataGroup));
            saveAllData();
          } else if (i == 2) { 
            saveAllData();
          }
        }
      }
    }
    // 3. 按键释放瞬间 (短按)
    else if (currState == HIGH && keys[i].lastState == LOW) {
      lastActivityMs = millis();

      if (!keys[i].longPressed && !keys[i].isRepeating) {
        if (i == 0) {
          currentPage = (currentPage + 1) % 5;
        } else if (i == 1) {
          if (currentPage == 4) {
            applySettingChange(false);
          } else {
            sysCfg.activeGroup = (sysCfg.activeGroup + 1) % 4;
          }
        } else if (i == 2) {
          if (currentPage == 4) {
            applySettingChange(true);
          } else {
            sysCfg.screenRot = (sysCfg.screenRot == 0) ? 2 : 0;
            display.setRotation(sysCfg.screenRot);
            saveAllData();
          }
        } else if (i == 3) {
          if (currentPage == 4) {
            settingItem = (settingItem + 1) % 6;
          } else {
            sysCfg.waveMode = (sysCfg.waveMode == 0) ? 1 : 0;
            saveAllData();
          }
        }
      }
    }
    keys[i].lastState = currState;
  }
}

// ---------------------- 串口输出 ----------------------
void handleSerialLog() {
  if (millis() - lastSerialMs > 1000) {
    lastSerialMs = millis();
    DataGroup &g = groups[sysCfg.activeGroup];
    Serial.print("DAT,");
    Serial.print(g.seconds); Serial.print(",");
    Serial.print(V, 3); Serial.print(",");
    Serial.print(A, 3); Serial.print(",");
    Serial.print(W, 3); Serial.print(",");
    Serial.print(R, 1); Serial.print(",");
    Serial.print(VP, 2); Serial.print(",");
    Serial.print(VN, 2); Serial.print(",");
    Serial.print(TempC, 1); Serial.print(",");
    Serial.print(sysCfg.activeGroup); Serial.print(",");
    Serial.print(g.mAh, 1); Serial.print(",");
    Serial.println(g.mWh, 1);
  }
}

// =========================================================================
//                           UI 绘制渲染引擎
// =========================================================================

void drawHeader() {
  display.fillRect(0, 0, 22, 9, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 1);
  display.print("G");
  display.print(sysCfg.activeGroup);

  display.setTextColor(SSD1306_WHITE);
  if (heartbeatState) {
    display.fillCircle(28, 4, 2, SSD1306_WHITE);
  } else {
    display.drawCircle(28, 4, 2, SSD1306_WHITE);
  }

  display.setCursor(35, 1);
  display.print(getShortProtocolName());

  display.setCursor(85, 1);
  display.print(TempC, 1);
  display.print("C");

  display.drawFastHLine(0, 10, 128, SSD1306_WHITE);
}

void drawPage0_Main() {
  drawHeader();

  display.drawRoundRect(0, 13, 62, 33, 3, SSD1306_WHITE);
  display.setCursor(4, 16); display.setTextSize(1); display.print("VOLT");
  display.setCursor(4, 27); display.setTextSize(2); display.print(V, 2);

  display.drawRoundRect(65, 13, 63, 33, 3, SSD1306_WHITE);
  display.setCursor(69, 16); display.setTextSize(1); display.print("CURR");
  display.setCursor(69, 27); display.setTextSize(2); display.print(A, 2);

  display.drawRoundRect(0, 48, 128, 16, 2, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(4, 52); display.print("P:"); display.print(W, 1); display.print("W");

  DataGroup &g = groups[sysCfg.activeGroup];
  display.setCursor(65, 52); display.print(g.mAh, 0); display.print("mAh");
}

void drawPage1_Details() {
  drawHeader();

  display.drawRoundRect(0, 13, 62, 24, 2, SSD1306_WHITE);
  display.setCursor(3, 15); display.setTextSize(1);
  display.print("V:"); display.print(V, 3); display.print("V");
  display.setCursor(3, 27);
  display.print("I:"); display.print(A, 3); display.print("A");

  display.drawRoundRect(65, 13, 63, 24, 2, SSD1306_WHITE);
  display.setCursor(68, 15); display.setTextSize(1);
  display.print("P:"); display.print(W, 2); display.print("W");
  display.setCursor(68, 27);
  display.print("R:"); display.print(R, 1); display.print("R");

  display.drawRoundRect(0, 39, 128, 25, 3, SSD1306_WHITE);
  
  display.fillRect(3, 42, 60, 10, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(5, 43); display.print(getProtocolName());

  display.setTextColor(SSD1306_WHITE);
  display.setCursor(68, 43); display.print("D+:"); display.print(VP, 2); display.print("V");
  display.setCursor(68, 53); display.print("D-:"); display.print(VN, 2); display.print("V");
}

void drawPage2_Scope() {
  drawHeader();

  for (uint8_t x = 0; x < 128; x += 16) {
    for (uint8_t y = 12; y < 64; y += 10) {
      display.drawPixel(x, y, SSD1306_WHITE);
    }
  }

  float minVal = 999.0f, maxVal = -999.0f;
  for (uint8_t i = 0; i < WAVE_BUF_SIZE; i++) {
    if (waveBuf[i] < minVal) minVal = waveBuf[i];
    if (waveBuf[i] > maxVal) maxVal = waveBuf[i];
  }
  if (maxVal - minVal < 0.05f) { maxVal = minVal + 0.05f; }

  for (uint8_t i = 0; i < WAVE_BUF_SIZE - 1; i++) {
    uint8_t x1 = i * 2;
    uint8_t x2 = (i + 1) * 2;
    uint8_t y1 = 62 - (uint8_t)(((waveBuf[i] - minVal) / (maxVal - minVal)) * 45.0f);
    uint8_t y2 = 62 - (uint8_t)(((waveBuf[i + 1] - minVal) / (maxVal - minVal)) * 45.0f);
    display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
  }

  display.setCursor(75, 14); display.setTextSize(1);
  display.print((sysCfg.waveMode == 0) ? "V-WAVE" : "A-WAVE");
  display.setCursor(75, 24);
  display.print("H:"); display.print(maxVal, 2);
}

void drawPage3_Energy() {
  drawHeader();

  DataGroup &g = groups[sysCfg.activeGroup];

  display.drawRoundRect(0, 13, 128, 26, 3, SSD1306_WHITE);
  display.setCursor(5, 16); display.setTextSize(2); display.print(g.mAh, 0); display.setTextSize(1); display.print(" mAh");
  display.setCursor(5, 30); display.setTextSize(1); display.print("Energy: "); display.print(g.mWh, 1); display.print(" mWh");

  display.setCursor(0, 42);
  display.print("Vmax:"); display.print(g.vMax, 2); display.print("V ");
  display.print("Imax:"); display.print(g.aMax, 2); display.print("A");

  uint32_t hh = g.seconds / 3600;
  uint32_t mm = (g.seconds % 3600) / 60;
  uint32_t ss = g.seconds % 60;
  display.setCursor(0, 53);
  display.print("T: ");
  if (hh < 10) display.print("0"); display.print(hh); display.print(":");
  if (mm < 10) display.print("0"); display.print(mm); display.print(":");
  if (ss < 10) display.print("0"); display.print(ss);
}

// Page 4: 极客设置、亮度、两点校准与线阻菜单 View
void drawPage4_Settings() {
  drawHeader();

  display.setTextSize(1);
  
  // 0. 亮度
  display.setCursor(0, 12);
  display.print((settingItem == 0) ? "> Bright: " : "  Bright: ");
  display.print((uint16_t)(sysCfg.brightness * 100 / 255)); display.print("%");

  // 1. 电压斜率 (V-Scale)
  display.setCursor(0, 20);
  display.print((settingItem == 1) ? "> V-Scale:" : "  V-Scale:");
  display.print(sysCfg.voltCoeff, 2);

  // 2. 电压偏置 (V-Offset，用于补偿二极管压降)
  display.setCursor(0, 28);
  display.print((settingItem == 2) ? "> V-Off:  " : "  V-Off:  ");
  if (sysCfg.voltOffset >= 0) display.print("+");
  display.print(sysCfg.voltOffset, 2); display.print("V");

  // 3. 电流系数 (I-Cal)
  display.setCursor(0, 36);
  display.print((settingItem == 3) ? "> I-Cal:  " : "  I-Cal:  ");
  display.print(sysCfg.currCoeff, 0);

  // 4. 自动熄屏时间
  display.setCursor(0, 44);
  display.print((settingItem == 4) ? "> Sleep:  " : "  Sleep:  ");
  if (sysCfg.sleepMin == 0) display.print("OFF");
  else { display.print(sysCfg.sleepMin); display.print("min"); }

  // 5. 数据线线阻测试
  display.setCursor(0, 52);
  display.print((settingItem == 5) ? "> CableR: " : "  CableR: ");
  if (cableR > 0.0f) {
    display.print(cableR, 0); display.print("mR");
  } else if (cableV0Locked) {
    display.print("V0="); display.print(cableV0, 2); display.print("V");
  } else {
    display.print("K2:V0 K3:Calc");
  }
}

void renderDisplay() {
  if (isScreenSleeping) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  switch (currentPage) {
    case 0: drawPage0_Main(); break;
    case 1: drawPage1_Details(); break;
    case 2: drawPage2_Scope(); break;
    case 3: drawPage3_Energy(); break;
    case 4: drawPage4_Settings(); break;
  }

  display.display();
}

// ---------------------- 主入口 setup & loop ----------------------
void setup() {
  analogReadResolution(12);
  Serial.begin(115200);

  pinMode(PIN_CURR_HI, INPUT_ANALOG);
  pinMode(PIN_CURR_LO, INPUT_ANALOG);
  pinMode(PIN_VOLT_DN, INPUT_ANALOG);
  pinMode(PIN_VOLT_DP, INPUT_ANALOG);
  pinMode(PIN_NTC_IN,  INPUT_ANALOG);
  pinMode(PIN_VOLT_IN, INPUT_ANALOG);

  pinMode(PIN_K1, INPUT_PULLUP);
  pinMode(PIN_K2, INPUT_PULLUP);
  pinMode(PIN_K3, INPUT_PULLUP);
  pinMode(PIN_K4, INPUT_PULLUP);

  pinMode(PIN_M1, OUTPUT);
  pinMode(PIN_M2, OUTPUT);
  digitalWrite(PIN_M1, HIGH);
  digitalWrite(PIN_M2, LOW);

  Wire.begin();

  loadAllData();

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setRotation(sysCfg.screenRot);
  setOledBrightness(sysCfg.brightness);
  display.clearDisplay();
  display.display();

  lastActivityMs = millis();
}

void loop() {
  doSampling();
  updateEnergyAndTimer();
  handleButtons();
  handleSerialLog();
  renderDisplay();
}
