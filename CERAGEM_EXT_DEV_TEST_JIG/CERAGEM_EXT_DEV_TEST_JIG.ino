// CERAGEM_EXT_DEV_TEST_JIG_FW

#include <ACAN_ESP32.h>
#include <Wire.h>
#include <INA226.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ====== 전역 상수 및 매크로 정의 ======
// 디스플레이 관련
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

// 스위치 관련
#define SWITCH_SCAN_INTERVAL 20  // 스위치 스캔 주기 (20ms)
#define SWITCH_DEBOUNCE_COUNT 10 // 디바운스 카운트 (10회)

// 타이밍 관련
#define DISPLAY_UPDATE_INTERVAL 100 // 디스플레이 업데이트 주기 (100ms)
#define CMD_DELAY_1_MS 100          // 명령 간 기본 지연 시간 (100ms)
#define CMD_DELAY_2_MS 500          // 마사지 제어 전 지연 시간 (500ms)
#define LED_CHECK_MS 3000           // LED 검사 시간 (3초)

// ====== 하드웨어 설정 구조체 및 전역 인스턴스 정의 ======
// 하드웨어 설정 구조체
struct HardwareConfig
{
  // 핀 정의
  struct
  {
    const uint8_t adc = 36;               // ADC1 핀
    const uint8_t relay5v = 16;           // 5V 릴레이 제어 핀
    const uint8_t relay24v = 17;          // 24V 릴레이 제어 핀
    const uint8_t swStop = 34;            // 스톱 스위치 모니터링 핀
    const uint8_t swStart = 35;           // 스타트 스위치 모니터링 핀
    const uint8_t ledStop = 32;           // STOP LED 핀
    const uint8_t ledStart = 33;          // START LED 핀
    const gpio_num_t canTx = GPIO_NUM_19; // CAN TX 핀
    const gpio_num_t canRx = GPIO_NUM_18; // CAN RX 핀
  } pins;

  // 타이밍 설정
  struct
  {
    const unsigned long cmdDelay1 = 100;        // 명령 간 기본 지연 시간 (ms)
    const unsigned long cmdDelay2 = 500;        // 마사지 제어 전 지연 시간 (ms)
    const unsigned long ledCheckTime = 3000;    // LED 검사 시간 (3초)
    const unsigned long peakResetTime = 2000;   // 피크값 초기화 시간 (2초)
    const unsigned long currentPrintTime = 500; // 전류 출력 주기 (500ms)
    const unsigned long ledBlinkInterval = 500; // LED 점멸 주기 (0.5초)
  } timing;

  // CAN 통신 설정
  struct
  {
    const uint32_t speed = 500000;   // CAN 통신 속도 (500kbps)
    const uint8_t retryCount = 3;    // CAN 메시지 재전송 횟수
    const uint8_t retryDelay = 100;  // 재전송 간격 (ms)
    const uint16_t timeoutMs = 1500; // CAN 타임아웃 시간 (1.5초)
    const uint32_t rmcId = 0x01;     // RMC CAN ID
  } can;

  // 디스플레이 설정
  struct
  {
    const uint8_t width = 128;
    const uint8_t height = 64;
    const int8_t reset = -1;
    const uint8_t address = 0x3C;
  } display;

  // 전류 측정 설정
  struct
  {
    const uint8_t ina226Addr = 0x40;   // INA226 I2C 주소
    const float maxCurrent = 3.0;      // 최대 전류 (A)
    const float shuntResistor = 0.005; // 션트 저항 (옴)
  } current;

  // 연결 상태 판단 설정
  struct
  {
    const float connectedThreshold = 2.0;    // 2.0V 이하이면 제품 연결로 판단
    const unsigned long debounceDelay = 500; // 디바운싱 시간 (500ms)
  } connection;
};

// 전역 하드웨어 설정 인스턴스
const HardwareConfig hwConfig;

// ====== 전역 객체 및 변수 선언 ======
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
INA226 ina226(hwConfig.current.ina226Addr);

// 검사 상태 정의
enum TestState
{
  READY,    // 초기 상태 및 연결 해제 후
  START,    // 검사 시작 준비 상태
  LED_TEST, // LED 검사 중
  VIB_TEST, // 진동 검사 중
  PAUSE,    // 검사 일시 정지 상태
  DISCON    // 연결 해제
};
TestState currentState = READY; // 현재 검사 상태

// 명령 상태 정의
enum CommandStateEnum
{
  CMD_IDLE,         // 대기 상태
  CMD_RMC_ON,       // RMC ON 명령 전송
  CMD_LED_MODE,     // LED 모드 설정
  CMD_LED_START,    // LED 검사 시작
  CMD_LED_WAIT,     // LED 동작 확인 대기
  CMD_LED_PAUSE,    // LED 검사 일시정지
  CMD_VIB_MODE,     // 진동 모드 설정
  CMD_TEMP_SET,     // 온도 설정
  CMD_STRENGTH_SET, // 강도 설정
  CMD_VIB_START,    // 진동 검사 시작
  CMD_COMPLETE      // 완료
};

// 시스템 상태 구조체
struct SystemState {
    // 연결 상태 관리
    struct ConnectionState {
        bool isConnected = false;        // 제품 연결 상태
        bool lastState = false;          // 이전 상태
        bool isCanConnected = false;     // CAN 통신 연결 상태
        bool isDisconnecting = false;    // 연결 해제 진행 중 여부
        bool forceStopActivated = false; // 강제 중지 활성화 여부
        unsigned long lastDebounceTime = 0;   // 마지막 디바운싱 시간
        unsigned long disconnectTime = 0;     // 연결 해제 시간 저장
    } connection;

    // 릴레이 상태 관리
    struct RelayState {
        bool rly5vState = false;  // 5V 릴레이 상태
        bool rly24vState = false; // 24V 릴레이 상태
        unsigned long rly5vDelay = 0;         // 5V 릴레이 제어 지연 시간
        unsigned long rly24vDelay = 0;        // 24V 릴레이 제어 지연 시간
    } relay;

    // 타이밍 관리
    struct TimingState {
        unsigned long lastCanCheckTime = 0;   // 마지막 CAN 체크 시간
        unsigned long lastCmdTime = 0;        // 마지막 명령 전송 시간
        unsigned long previousMillis = 0;     // 마지막 상태 변경 시간
        unsigned long readyStateDelay = 0;    // READY 상태로 전환하기 위한 지연 시간
        unsigned long lastDisplayUpdate = 0;  // 마지막 디스플레이 업데이트 시간
        unsigned long lastSwitchScanTime = 0; // 마지막 스위치 스캔 시간
        unsigned long lastLedBlinkTime = 0;   // 마지막 LED 점멸 시간
        unsigned long canTimeoutStart = 0;    // CAN 타임아웃 시작 시간
    } timing;

    // 스위치 상태 관리
    struct SwitchState {
        bool lastSwitchState = HIGH;         // 이전 스위치 상태
        bool currentSwitchState = HIGH;      // 현재 스위치 상태
        int switchStateCount = 0;            // 상태 카운트
        bool isSwitchPressed = false;        // 스위치 눌림 상태
        bool lastStartSwitchState = HIGH;    // 이전 START 스위치 상태
        bool currentStartSwitchState = HIGH; // 현재 START 스위치 상태
        int startSwitchStateCount = 0;       // START 스위치 상태 카운트
        bool isStartSwitchPressed = false;   // START 스위치 눌림 상태
    } switchState;

    // CAN 통신 상태 관리
    struct CanState {
        bool canError = false;        // CAN 에러 상태
        bool canCheckStarted = false; // CAN 체크 시작 여부
        bool commandsSent = false;    // CAN 명령 전송 완료 여부
    } can;

    // 전류 모니터링 상태 관리
    struct CurrentMonitorState {
        bool currentMonitorActive = false;      // 전류 모니터링 활성화 상태
        unsigned long lastCurrentPrintTime = 0; // 마지막 전류 출력 시간
        float peakCurrent = 0.0;                // 피크 전류값 저장
        unsigned long lastPeakResetTime = 0;    // 피크값 마지막 초기화 시간
        bool justReset = false;                 // 피크값이 방금 초기화되었는지 표시
        float emaCurrent = 0.0;                 // EMA 처리된 전류값
        bool isFirstEma = true;                 // 최초 EMA 계산 여부
    } currentMonitor;

    // 디스플레이 상태 관리
    struct DisplayState {
        bool stateChanged = false; // 상태 변경 플래그
    } display;

    // 명령 상태 관리
    struct CommandState {
        CommandStateEnum cmdState = CMD_IDLE; // 현재 명령 상태
        unsigned long cmdStartTime = 0;   // 명령 시작 시간
    } command;
};

// 전역 상태 인스턴스
SystemState sysState;

// CAN 메시지 전송 함수 선언
bool sendCanMessage(uint32_t id, uint8_t bank, uint16_t number, uint32_t data);

// CAN 통신 초기화 함수 선언
void initCAN();

// CAN 통신 모니터링 함수 선언
void checkCANCommunication();

// 전류 모니터링 함수 (millis 기반, 함수화)
void handleCurrentMonitor()
{
  // 연결이 해제되었거나 연결 해제 중이면 전류 모니터링 중지
  if (!sysState.connection.isConnected || sysState.connection.isDisconnecting)
  {
    sysState.currentMonitor.currentMonitorActive = false;
    return;
  }

  if (!sysState.currentMonitor.currentMonitorActive)
    return;
  unsigned long now = millis();

  // 3초마다 피크값 초기화
  if (now - sysState.currentMonitor.lastPeakResetTime >= hwConfig.timing.peakResetTime)
  {
    sysState.currentMonitor.peakCurrent = 0.0;
    sysState.currentMonitor.lastPeakResetTime = now;
    sysState.currentMonitor.justReset = true; // 초기화 순간 표시
  }

  if (now - sysState.currentMonitor.lastCurrentPrintTime >= hwConfig.timing.currentPrintTime)
  {
    sysState.currentMonitor.lastCurrentPrintTime = now;
    float current_mA = ina226.getCurrent_mA();
    float current_A = current_mA / 1000.0;

    // EMA 계산
    if (sysState.currentMonitor.isFirstEma)
    {
      sysState.currentMonitor.emaCurrent = current_A; // 최초 실행시 현재값으로 초기화
      sysState.currentMonitor.isFirstEma = false;
    }
    else
    {
      sysState.currentMonitor.emaCurrent = 0.1 * current_A + 0.9 * sysState.currentMonitor.emaCurrent; // EMA 계산
    }

    // 피크값 업데이트
    if (sysState.currentMonitor.emaCurrent > sysState.currentMonitor.peakCurrent)
    {
      sysState.currentMonitor.peakCurrent = sysState.currentMonitor.emaCurrent;
      sysState.currentMonitor.justReset = false; // 새로운 피크값이 생기면 초기화 표시 해제
    }

    // 시리얼 출력
    Serial.print("[INA226] Instant Current: ");
    Serial.print(current_A, 2);
    Serial.print(" A    |    EMA Current: ");
    Serial.print(sysState.currentMonitor.emaCurrent, 2);
    Serial.print(" A    |    Peak: ");
    Serial.print(sysState.currentMonitor.justReset ? "-.-" : String(sysState.currentMonitor.peakCurrent, 1));
    Serial.println(" A");

    // OLED 출력 (전류값만 업데이트)
    if (now - sysState.timing.lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL)
    {
      updateDisplayCurrent(sysState.currentMonitor.emaCurrent);
      sysState.timing.lastDisplayUpdate = now;
    }
  }
}

// LED 상태 설정 함수
void setLEDState(bool startLed, bool stopLed)
{
  digitalWrite(hwConfig.pins.ledStart, startLed);
  digitalWrite(hwConfig.pins.ledStop, stopLed);
}

// LED 제어 함수
void handleLEDControl()
{
  static bool ledState = false;
  unsigned long currentMillis = millis();

  // 에러 상태 체크
  if (sysState.can.canError)
  {
    // 에러 상태: 두 LED 모두 점멸
    if (currentMillis - sysState.timing.lastLedBlinkTime >= hwConfig.timing.ledBlinkInterval)
    {
      sysState.timing.lastLedBlinkTime = currentMillis;
      ledState = !ledState;
      setLEDState(ledState, ledState);
    }
    return;
  }

  // 일반 상태에 따른 LED 제어
  switch (currentState)
  {
  case READY:
  case PAUSE:
  case DISCON:
    // START LED 점멸, STOP LED 꺼짐
    if (currentMillis - sysState.timing.lastLedBlinkTime >= hwConfig.timing.ledBlinkInterval)
    {
      sysState.timing.lastLedBlinkTime = currentMillis;
      ledState = !ledState;
      setLEDState(ledState, false);
    }
    break;

  case START:
  case LED_TEST:
  case VIB_TEST:
    // START LED 꺼짐, STOP LED 점멸
    if (currentMillis - sysState.timing.lastLedBlinkTime >= hwConfig.timing.ledBlinkInterval)
    {
      sysState.timing.lastLedBlinkTime = currentMillis;
      ledState = !ledState;
      setLEDState(false, ledState);
    }
    break;
  }
}

// OLED 전류 표시 함수 (전류값만 표시)
void updateDisplayCurrent(float current)
{
  // 전류값 영역만 지우기
  display.fillRect(0, 32, hwConfig.display.width, 16, SSD1306_BLACK);

  // VIB_TEST 상태일 때만 전류값 표시
  if (currentState == VIB_TEST)
  {
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 32);
    display.print("CURR:");
    display.print(current, 2);
    display.println("A");
  }

  display.display();
}

// OLED 상태 표시 함수 (상태만 표시)
void updateDisplayState()
{
  if (!sysState.display.stateChanged)
    return;

  display.fillRect(0, 0, hwConfig.display.width, 16, SSD1306_BLACK);
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  if (sysState.can.canError)
  {
    display.println("CAN ERROR");
  }
  else
  {
    switch (currentState)
    {
    case READY:
      display.clearDisplay();
      break;
    case START:
      display.println("START");
      // 연결 시점에 전압값 표시
      display.fillRect(0, 16, hwConfig.display.width, 16, SSD1306_BLACK);
      display.setCursor(0, 16);
      display.print("VOLT:");
      display.print((analogRead(hwConfig.pins.adc) * 3.3) / 4095.0, 2);
      display.println("V");
      break;
    case LED_TEST:
      display.println("LED TEST");
      break;
    case VIB_TEST:
      display.println("VIB TEST");
      break;
    case PAUSE:
      display.println("PAUSE");
      break;
    case DISCON:
      display.println("DISCON");
      break;
    }
  }
  display.display();
  sysState.display.stateChanged = false;
}

// 전압 측정 및 연결 상태 확인
void checkConnectionStatus()
{
  int adcValue = analogRead(hwConfig.pins.adc);
  float voltage = (adcValue * 3.3) / 4095.0;
  bool swStopState = (digitalRead(hwConfig.pins.swStop) == LOW);

  // 현재 연결 상태 확인
  bool currentConnectionState = (voltage < hwConfig.connection.connectedThreshold);
  unsigned long currentMillis = millis();

  // 디바운싱 처리
  if (currentConnectionState != sysState.connection.lastState)
  {
    sysState.connection.lastDebounceTime = currentMillis;
  }

  // 디바운싱 시간이 지났고, 상태가 변경되었을 때만 처리
  if ((currentMillis - sysState.connection.lastDebounceTime) > hwConfig.connection.debounceDelay)
  {
    if (currentConnectionState != sysState.connection.isConnected)
    {
      sysState.connection.isConnected = currentConnectionState;

      // 강제 중지 후 재연결 시 처리
      if (sysState.connection.isConnected && sysState.connection.forceStopActivated)
      {
        Serial.println("강제 중지 후 재연결됨 - 검사 재시작 가능");
        sysState.connection.forceStopActivated = false; // 강제 중지 상태 해제
      }

      handleStateChange(voltage, swStopState);
    }
  }

  sysState.connection.lastState = currentConnectionState;
}

// 상태 변경 처리
void handleStateChange(float voltage, bool swStopState)
{
  unsigned long currentMillis = millis();

  if (!sysState.connection.isConnected || swStopState)
  {
    // 연결 해제 또는 스톱 스위치 감지
    if (!sysState.connection.isDisconnecting)
    {
      sysState.connection.isDisconnecting = true;
      sysState.timing.previousMillis = currentMillis;
      sysState.relay.rly24vDelay = currentMillis + 500; // 500ms 이후 24V 릴레이와 5V릴레이 모두 OFF
      sysState.relay.rly5vDelay = currentMillis + 500;

      // PAUSE 상태일 때도 READY로 변경
      currentState = READY;

      sysState.connection.disconnectTime = currentMillis; // 연결 해제 시간 저장
      sysState.display.stateChanged = true;            // 상태 변경 플래그 설정

      // 연결 해제 시 CAN 관련 모든 상태 초기화
      sysState.can.canError = false;
      sysState.connection.isCanConnected = false;
      sysState.can.commandsSent = false;
      sysState.can.canCheckStarted = false;
      sysState.timing.canTimeoutStart = 0;
      sysState.timing.lastCanCheckTime = 0;
      sysState.command.cmdState = CMD_IDLE;
      sysState.can.commandsSent = false;
      sysState.connection.forceStopActivated = false; // 강제 중지 상태도 해제

      updateDisplayState();
      Serial.println("CAN 관련 상태 초기화 완료");
    }
  }
  else
  {
    // 연결 감지
    // 강제 중지 상태일 때는 연결을 무시
    if (sysState.connection.forceStopActivated)
    {
      Serial.println("강제 중지 상태입니다. START 스위치를 눌러 재시작하거나, 장치를 분리했다가 다시 연결해주세요.");
      return;
    }

    sysState.connection.isDisconnecting = false;
    sysState.timing.previousMillis = currentMillis;
    sysState.relay.rly24vDelay = currentMillis + 500; // 24V 릴레이 먼저 HIGH
    sysState.relay.rly5vDelay = currentMillis + 1000; // 0.5초 후 5V 릴레이 HIGH

    // PAUSE 상태일 때는 START로 변경하지 않음
    if (currentState != PAUSE)
    {
      currentState = START;
    }

    sysState.display.stateChanged = true; // 상태 변경 플래그 설정
    updateDisplayState();
  }

  // 상태 변경 메시지 출력
  Serial.print("상태 변경: ");
  Serial.print(sysState.connection.isConnected ? "연결 됨" : "연결 해제");
  Serial.print(" (전압: ");
  Serial.print(voltage);
  Serial.println("V)");
}

// 릴레이 제어 처리
void handleRelayControl()
{
  // 강제 중지 상태일 때는 릴레이 제어를 하지 않음
  if (sysState.connection.forceStopActivated)
  {
    return;
  }

  unsigned long currentMillis = millis();

  if (sysState.connection.isConnected && !sysState.connection.isDisconnecting)
  {
    handleConnectedState(currentMillis);
  }
  else if (sysState.connection.isDisconnecting)
  {
    handleDisconnectingState(currentMillis);
  }
}

// 연결 상태일 때 릴레이 제어
void handleConnectedState(unsigned long currentMillis)
{
  // CAN 에러 상태면 릴레이 제어 중단
  if (sysState.can.canError)
  {
    return;
  }

  if (currentMillis >= sysState.relay.rly24vDelay && !sysState.relay.rly24vState)
  {
    digitalWrite(hwConfig.pins.relay24v, HIGH);
    sysState.relay.rly24vState = true;
    Serial.println("24V 릴레이 ON");
  }
  if (currentMillis >= sysState.relay.rly5vDelay && !sysState.relay.rly5vState)
  {
    digitalWrite(hwConfig.pins.relay5v, HIGH);
    sysState.relay.rly5vState = true;
    Serial.println("5V 릴레이 ON");
    // 5V 인가 후 500ms 대기 후 CAN 통신 시작
    sysState.timing.lastCanCheckTime = currentMillis + hwConfig.can.timeoutMs; // 1.5초 후 CAN 체크 시작
    sysState.timing.canTimeoutStart = currentMillis + hwConfig.can.timeoutMs;  // 1.5초 후 타임아웃 체크
    sysState.can.canCheckStarted = false;                                   // CAN 체크 시작 플래그 초기화
    Serial.println("CAN 통신 대기 중... (1.5초)");
  }
}

// 연결해제 시 릴레이 제어
void handleDisconnectingState(unsigned long currentMillis)
{
  if (currentMillis >= sysState.relay.rly24vDelay && sysState.relay.rly24vState)
  {
    digitalWrite(hwConfig.pins.relay24v, LOW);
    sysState.relay.rly24vState = false;
    Serial.println("24V 릴레이 OFF");
  }
  if (currentMillis >= sysState.relay.rly5vDelay && sysState.relay.rly5vState)
  {
    digitalWrite(hwConfig.pins.relay5v, LOW);
    sysState.relay.rly5vState = false;
    sysState.connection.isDisconnecting = false;
    // CAN 관련 상태 초기화
    sysState.connection.isCanConnected = false;
    sysState.can.commandsSent = false;
    sysState.currentMonitor.currentMonitorActive = false;
    Serial.println("CAN 통신 상태 및 전류 모니터링 초기화 완료");

    // READY 상태로 전환하기 위한 타이머 설정
    sysState.timing.readyStateDelay = currentMillis + 2000;
  }
}

// 장비 제어 명령 전송
void sendExtDevTestCommands()
{
  if (sysState.can.commandsSent || sysState.can.canError)
    return; // CAN 에러 상태면 명령 전송 중단

  unsigned long currentMillis = millis();

  switch (sysState.command.cmdState)
  {
  case CMD_IDLE:
    Serial.println("\n=== 장비 제어 명령 전송 시작 ===");
    sysState.command.cmdState = CMD_RMC_ON;
    break;

  case CMD_RMC_ON:
    Serial.println("1. RMC ON 명령 전송 중...");
    if (sendCanMessage(hwConfig.can.rmcId, 0, 1, 2))
    {
      Serial.println("   RMC ON 명령 전송 완료");
      sysState.command.cmdStartTime = currentMillis;
      sysState.command.cmdState = CMD_LED_MODE;
    }
    else
    {
      Serial.println("   RMC ON 명령 전송 실패");
      sysState.can.canError = true;
      currentState = READY;
      sysState.display.stateChanged = true;
      updateDisplayState();
    }
    break;

  case CMD_LED_MODE:
    if (currentMillis - sysState.command.cmdStartTime >= CMD_DELAY_1_MS)
    {
      Serial.println("2. LED 검사 모드 설정 중...");
      if (sendCanMessage(hwConfig.can.rmcId, 6, 22, 4))
      {
        Serial.println("   LED 검사 모드 설정 완료");
        sysState.command.cmdStartTime = currentMillis;
        sysState.command.cmdState = CMD_LED_START;
      }
      else
      {
        Serial.println("   LED 검사 모드 설정 실패");
        sysState.can.canError = true;
        currentState = READY;
        sysState.display.stateChanged = true;
        updateDisplayState();
      }
    }
    break;

  case CMD_LED_START:
    if (currentMillis - sysState.command.cmdStartTime >= CMD_DELAY_1_MS)
    {
      Serial.println("3. LED 검사 시작...");
      if (sendCanMessage(hwConfig.can.rmcId, 6, 21, 1))
      {
        Serial.println("   LED 검사 시작 완료");
        currentState = LED_TEST;
        sysState.display.stateChanged = true;
        updateDisplayState();
        Serial.println("   LED 동작 확인 중... (3초)");
        sysState.command.cmdStartTime = currentMillis;
        sysState.command.cmdState = CMD_LED_WAIT;
      }
      else
      {
        Serial.println("   LED 검사 시작 실패");
        sysState.can.canError = true;
        currentState = READY;
        sysState.display.stateChanged = true;
        updateDisplayState();
      }
    }
    break;

  case CMD_LED_WAIT:
    if (currentMillis - sysState.command.cmdStartTime >= LED_CHECK_MS)
    {
      sysState.command.cmdState = CMD_LED_PAUSE;
    }
    break;

  case CMD_LED_PAUSE:
    Serial.println("4. LED 검사 일시정지...");
    if (sendCanMessage(hwConfig.can.rmcId, 6, 21, 2))
    {
      Serial.println("   LED 검사 일시정지 완료");
      sysState.command.cmdStartTime = currentMillis;
      sysState.command.cmdState = CMD_VIB_MODE;
    }
    else
    {
      Serial.println("   LED 검사 일시정지 실패");
      sysState.can.canError = true;
      currentState = READY;
      sysState.display.stateChanged = true;
      updateDisplayState();
    }
    break;

  case CMD_VIB_MODE:
    if (currentMillis - sysState.command.cmdStartTime >= CMD_DELAY_1_MS)
    {
      Serial.println("5. 진동 검사 모드 설정 중...");
      if (sendCanMessage(hwConfig.can.rmcId, 6, 22, 1))
      {
        Serial.println("   진동 검사 모드 설정 완료");
        sysState.command.cmdStartTime = currentMillis;
        sysState.command.cmdState = CMD_TEMP_SET;
      }
      else
      {
        Serial.println("   진동 검사 모드 설정 실패");
        sysState.can.canError = true;
        currentState = READY;
        sysState.display.stateChanged = true;
        updateDisplayState();
      }
    }
    break;

  case CMD_TEMP_SET:
    if (currentMillis - sysState.command.cmdStartTime >= CMD_DELAY_1_MS)
    {
      Serial.println("6. 온도 설정 중...");
      if (sendCanMessage(hwConfig.can.rmcId, 6, 28, 450))
      {
        Serial.println("   온도 설정 완료");
        sysState.command.cmdStartTime = currentMillis;
        sysState.command.cmdState = CMD_STRENGTH_SET;
      }
      else
      {
        Serial.println("   온도 설정 실패");
        sysState.can.canError = true;
        currentState = READY;
        sysState.display.stateChanged = true;
        updateDisplayState();
      }
    }
    break;

  case CMD_STRENGTH_SET:
    if (currentMillis - sysState.command.cmdStartTime >= CMD_DELAY_1_MS)
    {
      Serial.println("7. 마사지 강도 설정 중...");
      if (sendCanMessage(hwConfig.can.rmcId, 6, 23, 3))
      {
        Serial.println("   마사지 강도 설정 완료");
        sysState.command.cmdStartTime = currentMillis;
        sysState.command.cmdState = CMD_VIB_START;
      }
      else
      {
        Serial.println("   마사지 강도 설정 실패");
        sysState.can.canError = true;
        currentState = READY;
        sysState.display.stateChanged = true;
        updateDisplayState();
      }
    }
    break;

  case CMD_VIB_START:
    if (currentMillis - sysState.command.cmdStartTime >= CMD_DELAY_2_MS)
    {
      Serial.println("8. 진동 검사 시작...");
      if (sendCanMessage(hwConfig.can.rmcId, 6, 21, 1))
      {
        Serial.println("   진동 검사 시작 완료");
        currentState = VIB_TEST;
        sysState.display.stateChanged = true;
        updateDisplayState();
        sysState.can.commandsSent = true;
        sysState.currentMonitor.currentMonitorActive = true;
        sysState.currentMonitor.lastCurrentPrintTime = currentMillis;
        Serial.println("전류 모니터링 시작");
        Serial.println("=== 장비 제어 명령 전송 완료 ===\n");
        sysState.command.cmdState = CMD_COMPLETE;
      }
      else
      {
        Serial.println("   진동 검사 시작 실패");
        sysState.can.canError = true;
        currentState = READY;
        sysState.display.stateChanged = true;
        updateDisplayState();
      }
    }
    break;

  case CMD_COMPLETE:
    // 모든 작업 완료
    break;
  }
}

void scanI2CDevices()
{
  Serial.println("\n[I2C SCAN] 시작");
  byte count = 0;
  for (byte address = 1; address < 127; ++address)
  {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0)
    {
      Serial.print("I2C 디바이스 발견: 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
      count++;
    }
  }
  if (count == 0)
  {
    Serial.println("I2C 디바이스를 찾을 수 없습니다.");
  }
  else
  {
    Serial.print("총 발견된 I2C 디바이스 수: ");
    Serial.println(count);
  }
  Serial.println("[I2C SCAN] 종료\n");
}

// 스위치 입력 감지 함수
void checkSwitchInput()
{
  unsigned long currentMillis = millis();

  // 스캔 주기 확인
  if (currentMillis - sysState.timing.lastSwitchScanTime >= SWITCH_SCAN_INTERVAL)
  {
    sysState.timing.lastSwitchScanTime = currentMillis;

    // STOP 스위치 처리
    handleStopSwitch();

    // START 스위치 처리
    handleStartSwitch();
  }
}

// STOP 스위치 처리 함수
void handleStopSwitch()
{
  // 현재 스위치 상태 읽기
  sysState.switchState.currentSwitchState = digitalRead(hwConfig.pins.swStop);

  // 상태가 변경되었는지 확인
  if (sysState.switchState.currentSwitchState != sysState.switchState.lastSwitchState)
  {
    sysState.switchState.switchStateCount = 1; // 카운트 초기화
  }
  else
  {
    sysState.switchState.switchStateCount++; // 카운트 증가
  }

  // 디바운스 카운트 확인
  if (sysState.switchState.switchStateCount >= SWITCH_DEBOUNCE_COUNT)
  {
    if (sysState.switchState.currentSwitchState != sysState.switchState.isSwitchPressed)
    {
      sysState.switchState.isSwitchPressed = sysState.switchState.currentSwitchState;

      // 스위치 상태 변경 시 처리
      if (sysState.switchState.isSwitchPressed)
      {
        Serial.println("STOP 스위치 해제됨");
      }
      else
      {
        Serial.println("STOP 스위치 눌림 감지!");

        // 검사 중일 때만 처리 (READY 상태가 아닐 때)
        if (currentState != READY && currentState != PAUSE)
        {
          Serial.println("검사 일시 정지!");

          // 릴레이 즉시 차단
          digitalWrite(hwConfig.pins.relay24v, LOW);
          digitalWrite(hwConfig.pins.relay5v, LOW);
          sysState.relay.rly24vState = false;
          sysState.relay.rly5vState = false;
          Serial.println("24V 릴레이 OFF");
          Serial.println("5V 릴레이 OFF");

          // 상태를 PAUSE로 변경
          currentState = PAUSE;
          sysState.display.stateChanged = true;
          sysState.connection.isCanConnected = false;
          sysState.can.commandsSent = false;
          sysState.currentMonitor.currentMonitorActive = false;

          // 강제 중지 상태 활성화
          sysState.connection.forceStopActivated = true;

          // 디스플레이 업데이트
          updateDisplayState();

          Serial.println("검사가 일시 정지되었습니다. START 스위치를 눌러 재시작하거나, 장치를 분리했다가 다시 연결해주세요.");
        }
      }
    }
  }

  sysState.switchState.lastSwitchState = sysState.switchState.currentSwitchState;
}

// START 스위치 처리 함수
void handleStartSwitch()
{
  // 현재 스위치 상태 읽기
  sysState.switchState.currentStartSwitchState = digitalRead(hwConfig.pins.swStart);

  // 상태가 변경되었는지 확인
  if (sysState.switchState.currentStartSwitchState != sysState.switchState.lastStartSwitchState)
  {
    sysState.switchState.startSwitchStateCount = 1; // 카운트 초기화
  }
  else
  {
    sysState.switchState.startSwitchStateCount++; // 카운트 증가
  }

  // 디바운스 카운트 확인
  if (sysState.switchState.startSwitchStateCount >= SWITCH_DEBOUNCE_COUNT)
  {
    if (sysState.switchState.currentStartSwitchState != sysState.switchState.isStartSwitchPressed)
    {
      sysState.switchState.isStartSwitchPressed = sysState.switchState.currentStartSwitchState;

      // 스위치 상태 변경 시 처리
      if (sysState.switchState.isStartSwitchPressed)
      {
        Serial.println("START 스위치 해제됨");
      }
      else
      {
        Serial.println("START 스위치 눌림 감지!");

        // PAUSE 상태일 때만 처리
        if (currentState == PAUSE)
        {
          Serial.println("검사 재시작!");

          // 강제 중지 상태 해제
          sysState.connection.forceStopActivated = false;

          // 상태를 START로 변경하여 검사 재시작
          currentState = START;
          sysState.display.stateChanged = true;

          // 릴레이 제어 타이밍 설정
          unsigned long currentMillis = millis();
          sysState.timing.previousMillis = currentMillis;
          sysState.relay.rly24vDelay = currentMillis + 500; // 24V 릴레이 먼저 HIGH
          sysState.relay.rly5vDelay = currentMillis + 1000; // 0.5초 후 5V 릴레이 HIGH

          // 디스플레이 업데이트
          updateDisplayState();

          Serial.println("검사가 재시작됩니다.");
        }
      }
    }
  }

  sysState.switchState.lastStartSwitchState = sysState.switchState.currentStartSwitchState;
}

// CAN 메시지 전송 함수 구현
bool sendCanMessage(uint32_t id, uint8_t bank, uint16_t number, uint32_t data)
{
  CANMessage message;
  message.id = id;
  message.len = 8; // 고정 길이 8바이트

  // 데이터 패킹 (Big Endian)
  message.data[0] = bank;            // Bank 번호
  message.data[1] = 2;               // 고정값
  message.data[2] = (number >> 8);   // Data ID (상위 바이트)
  message.data[3] = (number & 0xFF); // Data ID (하위 바이트)
  message.data[4] = (data >> 24);    // Data (최상위 바이트)
  message.data[5] = (data >> 16);    // Data
  message.data[6] = (data >> 8);     // Data
  message.data[7] = (data & 0xFF);   // Data (최하위 바이트)

  // 재전송 로직
  for (int retry = 0; retry < hwConfig.can.retryCount; retry++)
  {
    if (ACAN_ESP32::can.tryToSend(message))
    {
      Serial.print("CAN 송신 성공 (시도 ");
      Serial.print(retry + 1);
      Serial.print("/");
      Serial.print(hwConfig.can.retryCount);
      Serial.println(")");
      sysState.timing.canTimeoutStart = millis(); // CAN 메시지 전송 성공 시 타임아웃 시간 갱신
      return true;
    }

    Serial.print("CAN 송신 실패 (시도 ");
    Serial.print(retry + 1);
    Serial.print("/");
    Serial.print(hwConfig.can.retryCount);
    Serial.println(")");

    delay(hwConfig.can.retryDelay);
  }

  return false;
}

// CAN 통신 초기화 함수 구현
void initCAN()
{
  ACAN_ESP32_Settings settings(hwConfig.can.speed);
  settings.mRxPin = hwConfig.pins.canRx;
  settings.mTxPin = hwConfig.pins.canTx;

  const uint32_t errorCode = ACAN_ESP32::can.begin(settings);

  if (errorCode == 0)
  {
    Serial.println("CAN 통신 초기화 완료");
  }
  else
  {
    Serial.print("CAN 통신 초기화 실패, 에러 코드: 0x");
    Serial.println(errorCode, HEX);
  }
}

// CAN 통신 모니터링 함수 구현
void checkCANCommunication()
{
  // CAN 에러 상태면 체크 중단
  if (sysState.can.canError)
  {
    return;
  }

  unsigned long currentMillis = millis();

  // 5V 릴레이가 ON일 때만 처리
  if (sysState.relay.rly5vState)
  {
    if (!sysState.can.canCheckStarted)
    {
      // 5V 릴레이 ON 후 500ms 대기
      if (currentMillis - sysState.timing.lastCanCheckTime >= 500)
      {
        sysState.can.canCheckStarted = true;          // CAN 체크 시작
        sysState.timing.canTimeoutStart = currentMillis; // 타임아웃 시작 시간 설정
        Serial.println("CAN 통신 체크 시작 - 메시지 송신");

        // CAN 메시지 송신
        if (sendCanMessage(hwConfig.can.rmcId, 0, 1, 2))
        {
          Serial.println("CAN 메시지 송신 완료 - 응답 대기 중");
        }
        else
        {
          Serial.println("CAN 메시지 송신 실패");
          sysState.can.canError = true;
          currentState = READY;
          sysState.display.stateChanged = true;
          updateDisplayState();
        }
      }
    }
    else
    {
      CANMessage message;
      if (ACAN_ESP32::can.receive(message))
      {
        if (!sysState.connection.isCanConnected)
        { // CAN 연결이 처음 확인된 경우에만
          sysState.connection.isCanConnected = true;
          sysState.can.canError = false; // CAN 에러 상태 해제
          Serial.println("CAN 통신 연결 확인 - 장비 제어 명령 전송 시작");

          // 상태 머신 초기화
          sysState.command.cmdState = CMD_IDLE;
          sysState.can.commandsSent = false;
        }
        sysState.timing.canTimeoutStart = currentMillis; // CAN 메시지 수신 시 타임아웃 시작 시간 갱신
      }
      else
      {
        // CAN 메시지가 수신되지 않았고, 1.5초가 지났을 경우
        if (currentMillis - sysState.timing.canTimeoutStart >= hwConfig.can.timeoutMs)
        {
          sysState.can.canError = true;
          currentState = READY;         // READY 상태로 변경
          sysState.display.stateChanged = true; // 상태 변경 플래그 설정
          updateDisplayState();         // 디스플레이 업데이트
          Serial.println("CAN 통신 타임아웃 - CAN 라인 점검 필요");

          // 릴레이 차단
          digitalWrite(hwConfig.pins.relay24v, LOW);
          digitalWrite(hwConfig.pins.relay5v, LOW);
          sysState.relay.rly24vState = false;
          sysState.relay.rly5vState = false;
          sysState.connection.isCanConnected = false;
          sysState.can.canCheckStarted = false;
        }
      }
    }
  }
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(21, 22); // ESP32: SDA=21, SCL=22
  delay(100);         // 전원 안정화 대기

  // OLED 초기화
  if (!display.begin(SSD1306_SWITCHCAPVCC, hwConfig.display.address))
  {
    Serial.println(F("SSD1315(OLED) 초기화 실패"));
    for (;;)
      ; // 멈춤
    // TODO: SW 버튼을 주기적으로 깜빡이게 해서 동작 오류 표시 처리 필요
  }

  currentState = READY;
  sysState.display.stateChanged = true; // 상태 변경 플래그 설정
  updateDisplayState();

  scanI2CDevices(); // I2C 스캐너 먼저 실행

  // 핀 모드 설정
  pinMode(hwConfig.pins.adc, INPUT);
  pinMode(hwConfig.pins.relay5v, OUTPUT);
  pinMode(hwConfig.pins.relay24v, OUTPUT);
  pinMode(hwConfig.pins.swStop, INPUT);
  pinMode(hwConfig.pins.swStart, INPUT);
  pinMode(hwConfig.pins.ledStart, OUTPUT);
  pinMode(hwConfig.pins.ledStop, OUTPUT);

  // LED 초기 상태 설정
  digitalWrite(hwConfig.pins.ledStart, LOW);
  digitalWrite(hwConfig.pins.ledStop, LOW);

  // 초기 상태 설정
  digitalWrite(hwConfig.pins.relay5v, LOW);
  digitalWrite(hwConfig.pins.relay24v, LOW);

  // CAN 통신 초기화
  initCAN();
  if (ina226.begin())
  {
    Serial.println("INA226 초기화 성공");
    int err = ina226.setMaxCurrentShunt(hwConfig.current.maxCurrent, hwConfig.current.shuntResistor); // 최대전류 3.0A, 션트저항 0.005옴
    if (err == INA226_ERR_NONE)
    {
      Serial.println("INA226 캘리브레이션 성공");
    }
    else
    {
      Serial.print("INA226 캘리브레이션 실패, 에러코드: ");
      Serial.println(err, HEX);
    }
  }
  else
  {
    Serial.println("INA226 초기화 실패");
    Serial.print("Manufacturer ID: 0x");
    Serial.println(ina226.getManufacturerID(), HEX);
    Serial.print("Die ID: 0x");
    Serial.println(ina226.getDieID(), HEX);
  }
}

void loop()
{
  checkConnectionStatus(); // 연결 상태 확인
  handleRelayControl();    // 릴레이 제어
  checkSwitchInput();      // 스위치 입력 감지
  handleLEDControl();      // LED 제어

  if (sysState.connection.isConnected && !sysState.connection.isDisconnecting)
  {
    checkCANCommunication(); // CAN 통신 확인
    // CAN이 연결되어 있고, 명령이 전송되지 않았으며, 강제 중지 상태가 아닐 때만 명령 전송
    if (sysState.connection.isCanConnected && !sysState.can.commandsSent && !sysState.connection.forceStopActivated)
    {
      sendExtDevTestCommands();
    }
  }

  handleCurrentMonitor(); // 전류 모니터링 주기적 호출
}