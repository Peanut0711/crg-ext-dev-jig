// CERAGEM_EXT_DEV_TEST_JIG_FW

#include <ACAN_ESP32.h>
#include <Wire.h>
#include <INA226.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// 핀 정의
#define ADC_PIN         36      // ADC1 핀
#define RLY_5V_PIN      16      // 5V 릴레이 제어 핀
#define RLY_24V_PIN     17      // 24V 릴레이 제어 핀
#define SW_STOP_PIN     34      // 스톱 스위치 모니터링 핀
#define CAN_TX_PIN      GPIO_NUM_19  // CAN TX 핀
#define CAN_RX_PIN      GPIO_NUM_18  // CAN RX 핀

// CAN 통신 설정
#define CAN_SPEED       500000  // CAN 통신 속도 (500kbps)
#define CAN_RETRY_COUNT 3       // CAN 메시지 재전송 횟수
#define CAN_RETRY_DELAY 100     // 재전송 간격 (ms)

// CAN ID 정의
#define RMC_ID          0x01    // RMC CAN ID

// 타이밍 설정
#define CMD_DELAY_1     100     // 명령 간 기본 지연 시간 (ms)
#define CMD_DELAY_2     500     // 마사지 제어 전 지연 시간 (ms)
#define LED_CHECK_TIME  3000    // LED 검사 시간 (3초)
#define PEAK_RESET_TIME 2000    // 피크값 초기화 시간 (2초)
#define CURRENT_PRINT_TIME 300  // 전류 출력 주기 (300ms)

// 제품 연결 상태 판단을 위한 임계값 설정
const float CONNECTED_THRESHOLD = 2.0;  // 2.0V 미만이면 제품 연결로 판단
const unsigned long DEBOUNCE_DELAY = 500;  // 디바운싱 시간 (500ms)

bool isConnected = false;  // 제품 연결 상태
bool lastState = false;    // 이전 상태
bool isCanConnected = false;  // CAN 통신 연결 상태
bool commandsSent = false;    // CAN 명령 전송 완료 여부
unsigned long lastDebounceTime = 0;  // 마지막 디바운싱 시간
unsigned long lastCanCheckTime = 0;  // 마지막 CAN 체크 시간
unsigned long lastCmdTime = 0;       // 마지막 명령 전송 시간

// 타이밍 관련 변수
unsigned long previousMillis = 0;    // 마지막 상태 변경 시간
unsigned long rly5vDelay = 0;        // 5V 릴레이 제어 지연 시간
unsigned long rly24vDelay = 0;       // 24V 릴레이 제어 지연 시간
bool rly5vState = false;             // 5V 릴레이 상태
bool rly24vState = false;            // 24V 릴레이 상태
bool isDisconnecting = false;        // 연결 해제 진행 중 여부

// EMA 관련 변수 추가
float emaCurrent = 0.0;    // EMA 처리된 전류값
const float alpha = 0.1;   // EMA 알파 값
bool isFirstEma = true;    // 최초 EMA 계산 여부

// CAN 메시지 구조체 정의
struct CanMessage {
  uint32_t id;      // CAN ID
  uint8_t bank;     // Bank 번호
  uint8_t type;     // 메시지 타입 (2로 고정)
  uint16_t number;  // Data ID
  uint32_t data;    // 전송할 데이터
};

#define INA226_I2C_ADDR 0x40  // 기본 주소 (하드웨어에 따라 다를 수 있음)
INA226 ina226(INA226_I2C_ADDR);

// 전류 모니터링 상태 변수
bool currentMonitorActive = false;
unsigned long lastCurrentPrintTime = 0;
float peakCurrent = 0.0;  // 피크 전류값 저장
unsigned long lastPeakResetTime = 0;  // 피크값 마지막 초기화 시간
bool justReset = false;    // 피크값이 방금 초기화되었는지 표시

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDR     0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 전류 모니터링 함수 (millis 기반, 함수화)
void handleCurrentMonitor() {
  if (!currentMonitorActive) return;
  unsigned long now = millis();
  
  // 3초마다 피크값 초기화
  if (now - lastPeakResetTime >= PEAK_RESET_TIME) {
    peakCurrent = 0.0;
    lastPeakResetTime = now;
    justReset = true;  // 초기화 순간 표시
  }
  
  if (now - lastCurrentPrintTime >= CURRENT_PRINT_TIME) {
    lastCurrentPrintTime = now;
    float current_mA = ina226.getCurrent_mA();
    float current_A = current_mA / 1000.0;
    
    // EMA 계산
    if (isFirstEma) {
      emaCurrent = current_A;  // 최초 실행시 현재값으로 초기화
      isFirstEma = false;
    } else {
      emaCurrent = alpha * current_A + (1 - alpha) * emaCurrent;  // EMA 계산
    }
    
    // 피크값 업데이트
    if (current_A > peakCurrent) {
      peakCurrent = current_A;
      justReset = false;  // 새로운 피크값이 생기면 초기화 표시 해제
    }
    
    // 시리얼 출력
    Serial.print("[INA226] Instant Current: ");
    Serial.print(current_A, 2);
    Serial.print(" A    |    EMA Current: ");
    Serial.print(emaCurrent, 2);
    Serial.print(" A    |    Peak: ");
    Serial.print(justReset ? "-.-" : String(peakCurrent, 1));
    Serial.println(" A");
    
    // OLED 출력
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0,16);
    display.print("CURR: ");
    display.print(emaCurrent, 1);
    display.println("A");

    display.setCursor(0,32);
    display.print("PEAK: ");
    if (justReset) {
      display.println("    ");  // 공백으로 표시
    } else {
      display.print(peakCurrent, 1);
      display.println("A");
    }
    display.display();
  }
}

// CAN 메시지 전송 함수
bool sendCanMessage(uint32_t id, uint8_t bank, uint16_t number, uint32_t data) {
  CANMessage message;
  message.id = id;
  message.len = 8;  // 고정 길이 8바이트
  
  // 데이터 패킹 (Big Endian)
  message.data[0] = bank;           // Bank 번호
  message.data[1] = 2;              // 고정값
  message.data[2] = (number >> 8);  // Data ID (상위 바이트)
  message.data[3] = (number & 0xFF);// Data ID (하위 바이트)
  message.data[4] = (data >> 24);   // Data (최상위 바이트)
  message.data[5] = (data >> 16);   // Data
  message.data[6] = (data >> 8);    // Data
  message.data[7] = (data & 0xFF);  // Data (최하위 바이트)
  
  // 재전송 로직
  for (int retry = 0; retry < CAN_RETRY_COUNT; retry++) {
    if (ACAN_ESP32::can.tryToSend(message)) {
      Serial.print("CAN 송신 성공 (시도 ");
      Serial.print(retry + 1);
      Serial.print("/");
      Serial.print(CAN_RETRY_COUNT);
      Serial.println(")");
      return true;
    }
    
    Serial.print("CAN 송신 실패 (시도 ");
    Serial.print(retry + 1);
    Serial.print("/");
    Serial.print(CAN_RETRY_COUNT);
    Serial.println(")");
    
    delay(CAN_RETRY_DELAY);
  }
  
  return false;
}

// CAN 통신 초기화
void initCAN() {
  ACAN_ESP32_Settings settings(CAN_SPEED);
  settings.mRxPin = CAN_RX_PIN;
  settings.mTxPin = CAN_TX_PIN;
  
  const uint32_t errorCode = ACAN_ESP32::can.begin(settings);
  
  if (errorCode == 0) {
    Serial.println("CAN 통신 초기화 완료");
  } else {
    Serial.print("CAN 통신 초기화 실패, 에러 코드: 0x");
    Serial.println(errorCode, HEX);
  }
}

// CAN 통신 모니터링
void checkCANCommunication() {
  unsigned long currentMillis = millis();
  
  // 5V 릴레이가 ON이고, 500ms가 지난 후에만 CAN 통신 체크
  if (rly5vState && (currentMillis - lastCanCheckTime >= 500)) {
    lastCanCheckTime = currentMillis;
    
    CANMessage message;
    if (ACAN_ESP32::can.receive(message)) {
      if (!isCanConnected) {  // CAN 연결이 처음 확인된 경우에만
        isCanConnected = true;
        Serial.println("CAN 통신 연결 확인 - 장비 제어 명령 전송 시작");
        sendExtDevTestCommands();  // CAN 연결 확인 시 한 번만 명령 전송
      }
      
      // CAN 수신 메시지 출력 비활성화
      /*
      Serial.print("CAN 수신: ID=0x");
      Serial.print(message.id, HEX);
      Serial.print(", DLC=");
      Serial.print(message.len);
      Serial.print(", Data=");
      
      for (int i = 0; i < message.len; i++) {
        Serial.print("0x");
        Serial.print(message.data[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      */
    }
  }
}

// 전압 측정 및 연결 상태 확인
void checkConnectionStatus() {
  int adcValue = analogRead(ADC_PIN);
  float voltage = (adcValue * 3.3) / 4095.0;
  bool swStopState = (digitalRead(SW_STOP_PIN) == LOW);
  
  // 현재 연결 상태 확인
  bool currentConnectionState = (voltage < CONNECTED_THRESHOLD);
  unsigned long currentMillis = millis();
  
  // 디바운싱 처리
  if (currentConnectionState != lastState) {
    lastDebounceTime = currentMillis;
  }
  
  // 디바운싱 시간이 지났고, 상태가 변경되었을 때만 처리
  if ((currentMillis - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (currentConnectionState != isConnected) {
      isConnected = currentConnectionState;
      handleStateChange(voltage, swStopState);
    }
  }
  
  lastState = currentConnectionState;
}

// 상태 변경 처리
void handleStateChange(float voltage, bool swStopState) {
  unsigned long currentMillis = millis();
  
  if (!isConnected || swStopState) {
    // 연결 해제 또는 스톱 스위치 감지
    if (!isDisconnecting) {
      isDisconnecting = true;
      previousMillis = currentMillis;
      rly24vDelay = currentMillis + 500;  // 24V 릴레이 먼저 LOW
      rly5vDelay = currentMillis + 1000;  // 0.5초 후 5V 릴레이 LOW
    }
  } else {
    // 연결 감지
    isDisconnecting = false;
    previousMillis = currentMillis;
    rly24vDelay = currentMillis + 500;    // 24V 릴레이 먼저 HIGH
    rly5vDelay = currentMillis + 1000;    // 0.5초 후 5V 릴레이 HIGH
  }
  
  // 상태 변경 메시지 출력
  Serial.print("상태 변경: ");
  Serial.print(isConnected ? "연결 됨" : "연결 해제");
  Serial.print(" (전압: ");
  Serial.print(voltage);
  Serial.println("V)");
}

// 릴레이 제어 처리
void handleRelayControl() {
  unsigned long currentMillis = millis();
  
  if (isConnected && !isDisconnecting) {
    handleConnectedState(currentMillis);
  } else if (isDisconnecting) {
    handleDisconnectingState(currentMillis);
  }
}

// 연결 상태일 때 릴레이 제어
void handleConnectedState(unsigned long currentMillis) {
  if (currentMillis >= rly24vDelay && !rly24vState) {
    digitalWrite(RLY_24V_PIN, HIGH);
    rly24vState = true;
    Serial.println("24V 릴레이 ON");
  }
  if (currentMillis >= rly5vDelay && !rly5vState) {
    digitalWrite(RLY_5V_PIN, HIGH);
    rly5vState = true;
    Serial.println("5V 릴레이 ON");
    // 5V 인가 후 500ms 대기 후 CAN 통신 시작
    lastCanCheckTime = currentMillis + 500;
    Serial.println("CAN 통신 대기 중... (500ms)");
  }
}

// 연결 해제 중일 때 릴레이 제어
void handleDisconnectingState(unsigned long currentMillis) {
  if (currentMillis >= rly24vDelay && rly24vState) {
    digitalWrite(RLY_24V_PIN, LOW);
    rly24vState = false;
    Serial.println("24V 릴레이 OFF");
  }
  if (currentMillis >= rly5vDelay && rly5vState) {
    digitalWrite(RLY_5V_PIN, LOW);
    rly5vState = false;
    Serial.println("5V 릴레이 OFF");
    isDisconnecting = false;
    // CAN 관련 상태 초기화
    isCanConnected = false;
    commandsSent = false;
    currentMonitorActive = false; // 장치 분리 시 전류 모니터링 중지
    Serial.println("CAN 통신 상태 및 전류 모니터링 초기화 완료");
    
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(8,24);
    display.println("DISCONNECT");
    display.display();
  }
}

// 장비 제어 명령 전송
void sendExtDevTestCommands() {
  if (commandsSent) return;
  
  Serial.println("\n=== 장비 제어 명령 전송 시작 ===");
  
  // RMC ON 명령
  Serial.println("1. RMC ON 명령 전송 중...");
  if (sendCanMessage(RMC_ID, 0, 1, 2)) {
    Serial.println("   RMC ON 명령 전송 완료");
    delay(CMD_DELAY_1);
    
    // LED 검사 모드 설정 (마사지 모드 4)
    Serial.println("2. LED 검사 모드 설정 중...");
    if (sendCanMessage(RMC_ID, 6, 22, 4)) {
      Serial.println("   LED 검사 모드 설정 완료");
      delay(CMD_DELAY_1);
      
      // LED 검사 시작 (재생)
      Serial.println("3. LED 검사 시작...");
      if (sendCanMessage(RMC_ID, 6, 21, 1)) {
        Serial.println("   LED 검사 시작 완료");
        Serial.println("   LED 동작 확인 중... (3초)");
        delay(LED_CHECK_TIME);
        
        // LED 검사 일시정지
        Serial.println("4. LED 검사 일시정지...");
        if (sendCanMessage(RMC_ID, 6, 21, 2)) {
          Serial.println("   LED 검사 일시정지 완료");
          delay(CMD_DELAY_1);
          
          // 진동 검사 모드 설정 (마사지 모드 1)
          Serial.println("5. 진동 검사 모드 설정 중...");
          if (sendCanMessage(RMC_ID, 6, 22, 1)) {
            Serial.println("   진동 검사 모드 설정 완료");
            delay(CMD_DELAY_1);
            
            // 온도 설정
            Serial.println("6. 온도 설정 중...");
            if (sendCanMessage(RMC_ID, 6, 28, 450)) {
              Serial.println("   온도 설정 완료");
              delay(CMD_DELAY_1);
              
              // 마사지 강도 설정
              Serial.println("7. 마사지 강도 설정 중...");
              if (sendCanMessage(RMC_ID, 6, 23, 3)) {
                Serial.println("   마사지 강도 설정 완료");
                delay(CMD_DELAY_2);
                
                // 진동 검사 시작 (재생)
                Serial.println("8. 진동 검사 시작...");
                if (sendCanMessage(RMC_ID, 6, 21, 1)) {
                  Serial.println("   진동 검사 시작 완료");
                  commandsSent = true;
                  currentMonitorActive = true; // 전류 모니터링 시작
                  lastCurrentPrintTime = millis();
                  Serial.println("전류 모니터링 시작");
                  Serial.println("=== 장비 제어 명령 전송 완료 ===\n");
                } else {
                  Serial.println("   진동 검사 시작 실패");
                }
              } else {
                Serial.println("   마사지 강도 설정 실패");
              }
            } else {
              Serial.println("   온도 설정 실패");
            }
          } else {
            Serial.println("   진동 검사 모드 설정 실패");
          }
        } else {
          Serial.println("   LED 검사 일시정지 실패");
        }
      } else {
        Serial.println("   LED 검사 시작 실패");
      }
    } else {
      Serial.println("   LED 검사 모드 설정 실패");
    }
  } else {
    Serial.println("   RMC ON 명령 전송 실패");
  }
}

void scanI2CDevices() {
  Serial.println("\n[I2C SCAN] 시작");
  byte count = 0;
  for (byte address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("I2C 디바이스 발견: 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
      count++;
    }
  }
  if (count == 0) {
    Serial.println("I2C 디바이스를 찾을 수 없습니다.");
  } else {
    Serial.print("총 발견된 I2C 디바이스 수: ");
    Serial.println(count);
  }
  Serial.println("[I2C SCAN] 종료\n");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // ESP32: SDA=21, SCL=22
  delay(100); // 전원 안정화 대기

  // OLED 초기화
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1315(OLED) 초기화 실패"));
    for(;;); // 멈춤
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(24,24);
  display.println("CERAGEM");
  display.display();
  delay(1000);
  display.clearDisplay();

  scanI2CDevices(); // I2C 스캐너 먼저 실행

  // 핀 모드 설정
  pinMode(ADC_PIN, INPUT);
  pinMode(RLY_5V_PIN, OUTPUT);
  pinMode(RLY_24V_PIN, OUTPUT);
  pinMode(SW_STOP_PIN, INPUT);
  
  // 초기 상태 설정
  digitalWrite(RLY_5V_PIN, LOW);
  digitalWrite(RLY_24V_PIN, LOW);
  
  // CAN 통신 초기화
  initCAN();
  if (ina226.begin()) {
    Serial.println("INA226 초기화 성공");
    int err = ina226.setMaxCurrentShunt(2.0, 0.005); // 최대전류 2.0A, 션트저항 0.005옴
    if (err == INA226_ERR_NONE) {
      Serial.println("INA226 캘리브레이션 성공");
    } else {
      Serial.print("INA226 캘리브레이션 실패, 에러코드: ");
      Serial.println(err, HEX);
    }
  } else {
    Serial.println("INA226 초기화 실패");
    Serial.print("Manufacturer ID: 0x");
    Serial.println(ina226.getManufacturerID(), HEX);
    Serial.print("Die ID: 0x");
    Serial.println(ina226.getDieID(), HEX);
  }
}

void loop() {
  checkConnectionStatus();    // 연결 상태 확인
  handleRelayControl();      // 릴레이 제어
  if (isConnected && !isDisconnecting) {
    checkCANCommunication();
  }
  handleCurrentMonitor(); // 전류 모니터링 주기적 호출
}
