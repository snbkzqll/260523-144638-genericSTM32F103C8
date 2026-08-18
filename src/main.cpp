// 硬件版本：USB 电压电流表
// 软件版本：3.2 (修复电流采样与MOS状态，恢复原版精准度)
// 编写时间：2026-08-18
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
const byte PIN_K2 = PB4;      // 按键 2：切组 (短按) / 清零当前组 (长按)
const byte PIN_K3 = PB10;     // 按键 3：旋转屏幕 (短按) / 强制保存 (长按)
const byte PIN_K4 = PB1;      // 按键 4：波形通道切换 (电压波形 / 电流波形)

const byte PIN_M1 = PB9;      // 大电流通道 MOS 控制 (常开 HIGH，保证回路无损导通)
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
  uint32_t magic;      // Flash / EEPROM 保存魔数校验
  uint8_t activeGroup; // 当前激活的数据组 (0 ~ 3)
  uint8_t screenRot;   // 屏幕旋转 (0: 0度, 2: 180度)
  uint8_t waveMode;    // 示波器波形源 (0: 电压波形, 1: 电流波形)
};

const uint32_t FLASH_MAGIC = 0x55AA2026;
const uint32_t FLASH_PAGE_ADDRESS = 0x0800FC00; // STM32F103 内部 Flash Page 63

DataGroup groups[4];
SystemConfig sysCfg = { FLASH_MAGIC, 0, 0, 0 };

// ---------------------- 全局测量变量 ----------------------
float V = 0.0f;       // 输入总电压 (V)
float A = 0.0f;       // 实时电流 (A)
float W = 0.0f;       // 实时功率 (W)
float R = 0.0f;       // 负载等效电阻 (Ω)
float VP = 0.0f;      // D+ 电压 (V)
float VN = 0.0f;      // D- 电压 (V)
float TempC = 0.0f;   // NTC 摄氏温度 (°C)

uint8_t currentPage = 0;   // 当前视图 (0: 主仪表, 1: 快充/网格, 2: 示波器, 3: 能量统计, 4: 设置)

// 波形环形缓冲区 (记录 60 个历史采样点)
const uint8_t WAVE_BUF_SIZE = 60;
float waveBuf[WAVE_BUF_SIZE];
uint8_t waveIndex = 0;

// 计时与脉冲控制
uint32_t lastSecondMs = 0;
uint32_t lastAutoSaveMs = 0;
uint32_t lastSerialMs = 0;
bool heartbeatState = false;
bool eepromDetected = false;

// ---------------------- AT24Cxx 优化版分页 I2C EEPROM 驱动 ----------------------
// 页写入 (每次写入最多 8 字节，避免大循环卡顿)
void writeEEPROM_Page(uint16_t addr, const uint8_t *pData, uint8_t len) {
  Wire.beginTransmission(AT24C_I2C_ADDR);
  Wire.write((uint8_t)(addr & 0xFF));
  for (uint8_t i = 0; i < len; i++) {
    Wire.write(pData[i]);
  }
  Wire.endTransmission();
  delay(5); // 仅需等待一次 5ms 页写入周期
}

void writeEEPROM_Buffer(uint16_t addr, const uint8_t *pData, uint16_t len) {
  uint16_t offset = 0;
  while (len > 0) {
    uint8_t chunk = (len > 8) ? 8 : len;
    writeEEPROM_Page(addr + offset, pData + offset, chunk);
    offset += chunk;
    len -= chunk;
  }
}

void readEEPROM_Buffer(uint16_t addr, uint8_t *pData, uint16_t len) {
  Wire.beginTransmission(AT24C_I2C_ADDR);
  Wire.write((uint8_t)(addr & 0xFF));
  Wire.endTransmission();
  
  uint16_t readLen = 0;
  while (readLen < len) {
    uint8_t chunk = (len - readLen > 32) ? 32 : (len - readLen);
    Wire.requestFrom((int)AT24C_I2C_ADDR, (int)chunk);
    for (uint8_t i = 0; i < chunk && Wire.available(); i++) {
      pData[readLen++] = Wire.read();
    }
  }
}

bool checkEEPROM() {
  Wire.beginTransmission(AT24C_I2C_ADDR);
  return (Wire.endTransmission() == 0);
}

// ---------------------- 内部 Flash 模拟擦写 (备用降级方案) ----------------------
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

// ---------------------- 数据加载与自动保存 ----------------------
void loadAllData() {
  eepromDetected = checkEEPROM();
  bool loaded = false;

  if (eepromDetected) {
    readEEPROM_Buffer(0, (uint8_t *)&sysCfg, sizeof(SystemConfig));
    if (sysCfg.magic == FLASH_MAGIC) {
      readEEPROM_Buffer(sizeof(SystemConfig), (uint8_t *)groups, sizeof(groups));
      loaded = true;
    }
  }

  if (!loaded) {
    uint32_t *pFlash = (uint32_t *)FLASH_PAGE_ADDRESS;
    if (*pFlash == FLASH_MAGIC) {
      memcpy(&sysCfg, (void *)FLASH_PAGE_ADDRESS, sizeof(SystemConfig));
      memcpy(groups, (void *)(FLASH_PAGE_ADDRESS + sizeof(SystemConfig)), sizeof(groups));
    } else {
      sysCfg.magic = FLASH_MAGIC;
      sysCfg.activeGroup = 0;
      sysCfg.screenRot = 0;
      sysCfg.waveMode = 0;
      memset(groups, 0, sizeof(groups));
    }
  }
}

void saveAllData() {
  if (eepromDetected) {
    writeEEPROM_Buffer(0, (const uint8_t *)&sysCfg, sizeof(SystemConfig));
    writeEEPROM_Buffer(sizeof(SystemConfig), (const uint8_t *)groups, sizeof(groups));
  } else {
    saveFlashBackup();
  }
}

// ---------------------- 快充协议检测 ----------------------
const char* getProtocolName() {
  if (VP >= 0.40f && VP <= 0.85f && VN >= 0.40f && VN <= 0.85f) return "QC3.0";
  if (VP >= 2.80f && VP <= 3.60f && VN >= 0.40f && VN <= 0.85f) return "QC2.0 9V/12V";
  if (VP >= 2.40f && VP <= 2.90f && VN >= 2.40f && VN <= 2.90f) return "Apple 2.4A";
  if (VP >= 1.80f && VP <= 2.30f && VN >= 2.40f && VN <= 2.90f) return "Apple 2.1A";
  if (VP >= 2.40f && VP <= 2.90f && VN >= 1.80f && VN <= 2.30f) return "Apple 1.0A";
  if (fabs(VP - VN) < 0.15f && VP < 1.20f && VP > 0.10f)        return "DCP 1.5A";
  if (VP < 0.30f && VN < 0.30f)                                return "Std USB 5V";
  return "Unknown/PD";
}

// ---------------------- ADC 采样与滤波 (100% 忠实原版精准算法) ----------------------
void doSampling() {
  // 1. D+/D- 电压 (PA0 / PA3)
  float vnRaw = map(analogRead(PIN_VOLT_DN), 0, 4095, 0, 3300);
  VN = (vnRaw * 2.0f) / 1000.0f;
  float vpRaw = map(analogRead(PIN_VOLT_DP), 0, 4095, 0, 3300);
  VP = (vpRaw * 2.0f) / 1000.0f;

  // 2. 总电压 V (PA5，50 次均值滤波，分压比 12.01)
  float sumV = 0;
  for (byte i = 0; i < 50; i++) {
    float vRaw = map(analogRead(PIN_VOLT_IN), 0, 4095, 0, 3300);
    sumV += (vRaw * 12.01f) / 1000.0f;
  }
  V = sumV / 50.0f;

  // 3. 电流 A (PA1，50 次均值滤波，忠实采用原工程公式 a / 101 / 5)
  float sumA = 0;
  for (byte i = 0; i < 50; i++) {
    float aRaw = map(analogRead(PIN_CURR_HI), 0, 4095, 0, 3300);
    sumA += aRaw / 101.0f / 5.0f;
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
  float ntcRaw = map(analogRead(PIN_NTC_IN), 0, 4095, 0, 3300);
  if (3300.0f - ntcRaw > 1.0f) {
    float rNtc = (10000.0f * ntcRaw) / (3300.0f - ntcRaw);
    TempC = (3950.0f * 298.15f) / (3950.0f + (298.15f * log(rNtc / 10000.0f))) - 273.15f - 2.0f;
  }

  // 更新当前组极值 (Peak Holds)
  DataGroup &g = groups[sysCfg.activeGroup];
  if (V > g.vMax) g.vMax = V;
  if (A > g.aMax) g.aMax = A;
  if (W > g.wMax) g.wMax = W;
}

// ---------------------- 1秒定时积分与电量更新 ----------------------
void updateEnergyAndTimer() {
  if (millis() - lastSecondMs >= 1000) {
    lastSecondMs = millis();
    heartbeatState = !heartbeatState;

    DataGroup &g = groups[sysCfg.activeGroup];
    g.seconds++;
    g.mWh += (W * 1000.0f) / 3600.0f;
    g.mAh += (A * 1000.0f) / 3600.0f;

    // 波形记录
    waveBuf[waveIndex] = (sysCfg.waveMode == 0) ? V : A;
    waveIndex = (waveIndex + 1) % WAVE_BUF_SIZE;
  }

  // 后台 5 秒自动无感保存
  if (millis() - lastAutoSaveMs >= 5000) {
    lastAutoSaveMs = millis();
    saveAllData();
  }
}

// ---------------------- 按键检测与事件 ----------------------
struct KeyState {
  byte pin;
  bool lastState;
  uint32_t pressTime;
  bool longPressed;
};

KeyState keys[4] = {
  { PIN_K1, HIGH, 0, false },
  { PIN_K2, HIGH, 0, false },
  { PIN_K3, HIGH, 0, false },
  { PIN_K4, HIGH, 0, false }
};

void handleButtons() {
  for (uint8_t i = 0; i < 4; i++) {
    bool currState = digitalRead(keys[i].pin);
    if (currState == LOW && keys[i].lastState == HIGH) {
      keys[i].pressTime = millis();
      keys[i].longPressed = false;
    } else if (currState == LOW && keys[i].lastState == LOW) {
      if (!keys[i].longPressed && (millis() - keys[i].pressTime > 1500)) {
        keys[i].longPressed = true;
        if (i == 1) { 
          // K2 长按：清零当前组
          memset(&groups[sysCfg.activeGroup], 0, sizeof(DataGroup));
          saveAllData();
        } else if (i == 2) { 
          // K3 长按：立即手动保存
          saveAllData();
        }
      }
    } else if (currState == HIGH && keys[i].lastState == LOW) {
      if (!keys[i].longPressed) {
        if (i == 0) {
          // K1：切页
          currentPage = (currentPage + 1) % 5;
        } else if (i == 1) {
          // K2：切组 G0 -> G1 -> G2 -> G3
          sysCfg.activeGroup = (sysCfg.activeGroup + 1) % 4;
        } else if (i == 2) {
          // K3：旋转屏幕 180°
          sysCfg.screenRot = (sysCfg.screenRot == 0) ? 2 : 0;
          display.setRotation(sysCfg.screenRot);
          saveAllData();
        } else if (i == 3) {
          // K4：切换示波器波形源 (电压曲线 <-> 电流曲线)
          sysCfg.waveMode = (sysCfg.waveMode == 0) ? 1 : 0;
          saveAllData();
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
  // 数据组标签 [G0]
  display.fillRect(0, 0, 22, 9, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 1);
  display.print("G");
  display.print(sysCfg.activeGroup);

  // 心跳指示点 ●
  display.setTextColor(SSD1306_WHITE);
  if (heartbeatState) {
    display.fillCircle(28, 4, 2, SSD1306_WHITE);
  } else {
    display.drawCircle(28, 4, 2, SSD1306_WHITE);
  }

  // 状态指示
  display.setCursor(35, 1);
  display.print("LIVE");

  // NTC 温度
  display.setCursor(85, 1);
  display.print(TempC, 1);
  display.print("C");

  display.drawFastHLine(0, 10, 128, SSD1306_WHITE);
}

// Page 0: 赛博主仪表盘 View
void drawPage0_Main() {
  drawHeader();

  // 电压卡片 (左上角)
  display.drawRoundRect(0, 13, 62, 33, 3, SSD1306_WHITE);
  display.setCursor(4, 16); display.setTextSize(1); display.print("VOLT");
  display.setCursor(4, 27); display.setTextSize(2); display.print(V, 2);

  // 电流卡片 (右上角)
  display.drawRoundRect(65, 13, 63, 33, 3, SSD1306_WHITE);
  display.setCursor(69, 16); display.setTextSize(1); display.print("CURR");
  display.setCursor(69, 27); display.setTextSize(2); display.print(A, 2);

  // 底部状态栏卡片 (功率 / 电量)
  display.drawRoundRect(0, 48, 128, 16, 2, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(4, 52); display.print("P:"); display.print(W, 1); display.print("W");

  DataGroup &g = groups[sysCfg.activeGroup];
  display.setCursor(65, 52); display.print(g.mAh, 0); display.print("mAh");
}

// Page 1: 快充协议与多维网格 View
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

// Page 2: 示波器波形 View
void drawPage2_Scope() {
  drawHeader();

  // 网格
  for (uint8_t x = 0; x < 128; x += 16) {
    for (uint8_t y = 12; y < 64; y += 10) {
      display.drawPixel(x, y, SSD1306_WHITE);
    }
  }

  // 极值计算
  float minVal = 999.0f, maxVal = -999.0f;
  for (uint8_t i = 0; i < WAVE_BUF_SIZE; i++) {
    if (waveBuf[i] < minVal) minVal = waveBuf[i];
    if (waveBuf[i] > maxVal) maxVal = waveBuf[i];
  }
  if (maxVal - minVal < 0.05f) { maxVal = minVal + 0.05f; }

  // 连线绘制
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

// Page 3: 能量统计与极值仪表 View
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
  display.setCursor(0, 53); display.print("T: ");
  if (hh < 10) display.print("0"); display.print(hh); display.print(":");
  if (mm < 10) display.print("0"); display.print(mm); display.print(":");
  if (ss < 10) display.print("0"); display.print(ss);
}

// Page 4: HUD 系统设置 View
void drawPage4_Settings() {
  drawHeader();

  display.setTextSize(1);
  display.setCursor(0, 14); display.print("> Page: "); display.print(currentPage + 1); display.print("/5");
  display.setCursor(0, 24); display.print("  Storage: "); display.print(eepromDetected ? "AT24C EEPROM" : "Flash Backup");
  display.setCursor(0, 34); display.print("  WaveSrc: "); display.print((sysCfg.waveMode == 0) ? "Volt Curve" : "Curr Curve");
  display.setCursor(0, 44); display.print("  K2 Hold: Clear G"); display.print(sysCfg.activeGroup);
  display.setCursor(0, 54); display.print("  K4: Switch WaveSrc");
}

void renderDisplay() {
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

  // 保持 M1 恒定 HIGH，M2 恒定 LOW，确保大电流主测量回路稳定通畅
  pinMode(PIN_M1, OUTPUT);
  pinMode(PIN_M2, OUTPUT);
  digitalWrite(PIN_M1, HIGH);
  digitalWrite(PIN_M2, LOW);

  Wire.begin();

  loadAllData();

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setRotation(sysCfg.screenRot);
  display.clearDisplay();
  display.display();
}

void loop() {
  doSampling();
  updateEnergyAndTimer();
  handleButtons();
  handleSerialLog();
  renderDisplay();
}
