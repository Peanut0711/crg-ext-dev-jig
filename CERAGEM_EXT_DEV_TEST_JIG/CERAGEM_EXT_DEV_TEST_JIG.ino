// CERAGEM_EXT_DEV_TEST_JIG_FW

#include <ACAN_ESP32.h>

// 핀 정의
#define ADC_PIN         36      // ADC1 핀
#define RLY_5V_PIN      16      // 5V 릴레이 제어 핀
#define RLY_24V_PIN     17      // 24V 릴레이 제어 핀
#define SW_STOP_PIN     34      // 스톱 스위치 모니터링 핀
#define CAN_TX_PIN      GPIO_NUM_19  // CAN TX 핀
#define CAN_RX_PIN      GPIO_NUM_18  // CAN RX 핀

// CAN 통신 설정
#define CAN_SPEED       500000  // CAN 통신 속도 (500kbps)

// 제품 연결 상태 판단을 위한 임계값 설정
const float CONNECTED_THRESHOLD = 2.0;  // 2.0V 미만이면 제품 연결로 판단
const unsigned long DEBOUNCE_DELAY = 500;  // 디바운싱 시간 (500ms)

bool isConnected = false;  // 제품 연결 상태
bool lastState = false;    // 이전 상태
unsigned long lastDebounceTime = 0;  // 마지막 디바운싱 시간
unsigned long lastCanCheckTime = 0;  // 마지막 CAN 체크 시간

// 타이밍 관련 변수
unsigned long previousMillis = 0;    // 마지막 상태 변경 시간
unsigned long rly5vDelay = 0;        // 5V 릴레이 제어 지연 시간
unsigned long rly24vDelay = 0;       // 24V 릴레이 제어 지연 시간
bool rly5vState = false;             // 5V 릴레이 상태
bool rly24vState = false;            // 24V 릴레이 상태
bool isDisconnecting = false;        // 연결 해제 진행 중 여부

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
  
  // 1초마다 CAN 통신 상태 체크
  if (currentMillis - lastCanCheckTime >= 1000) {
    lastCanCheckTime = currentMillis;
    
    CANMessage message;
    if (ACAN_ESP32::can.receive(message)) {
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
  }
}

void setup() {
  // 시리얼 통신 초기화 (115200 baud rate)
  Serial.begin(115200);
  
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
}

void loop() {
  checkConnectionStatus();    // 연결 상태 확인
  handleRelayControl();      // 릴레이 제어
  
  // 연결된 상태일 때만 CAN 통신 모니터링
  if (isConnected && !isDisconnecting) {
    checkCANCommunication();
  }
}
