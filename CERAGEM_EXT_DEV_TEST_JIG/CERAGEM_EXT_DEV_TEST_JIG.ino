// CERAGEM_EXT_DEV_TEST_JIG_FW

// 핀 정의
const int adcPin = 36;      // ADC1 핀
const int io17Pin = 17;     // 제어 핀 1
const int io18Pin = 18;     // 제어 핀 2
const int io34Pin = 34;     // 모니터링 핀

// 제품 연결 상태 판단을 위한 임계값 설정
const float CONNECTED_THRESHOLD = 2.0;  // 2.0V 미만이면 제품 연결로 판단
bool isConnected = false;  // 제품 연결 상태
bool lastState = false;    // 이전 상태

// 타이밍 관련 변수
unsigned long previousMillis = 0;    // 마지막 상태 변경 시간
unsigned long io17Delay = 0;         // IO17 제어 지연 시간
unsigned long io18Delay = 0;         // IO18 제어 지연 시간
bool io17State = false;              // IO17 상태
bool io18State = false;              // IO18 상태
bool isDisconnecting = false;        // 연결 해제 진행 중 여부

void setup() {
  // 시리얼 통신 초기화 (115200 baud rate)
  Serial.begin(115200);
  
  // 핀 모드 설정
  pinMode(adcPin, INPUT);
  pinMode(io17Pin, OUTPUT);
  pinMode(io18Pin, OUTPUT);
  pinMode(io34Pin, INPUT);
  
  // 초기 상태 설정
  digitalWrite(io17Pin, LOW);
  digitalWrite(io18Pin, LOW);
}

void loop() {
  unsigned long currentMillis = millis();
  
  // ADC 값 읽기
  int adcValue = analogRead(adcPin);
  float voltage = (adcValue * 3.3) / 4095.0;
  
  // IO34 상태 확인
  bool io34State = (digitalRead(io34Pin) == LOW);
  
  // 제품 연결 상태 확인
  isConnected = (voltage < CONNECTED_THRESHOLD);
  
  // 상태 변경 감지
  if (isConnected != lastState || io34State) {
    if (!isConnected || io34State) {
      // 연결 해제 또는 IO34 LOW 감지
      if (!isDisconnecting) {
        isDisconnecting = true;
        previousMillis = currentMillis;
        io18Delay = currentMillis + 500;  // IO18 먼저 LOW
        io17Delay = currentMillis + 1000; // 0.5초 후 IO17 LOW
      }
    } else {
      // 연결 감지
      isDisconnecting = false;
      previousMillis = currentMillis;
      io17Delay = currentMillis + 500;    // IO17 HIGH
      io18Delay = currentMillis + 1000;   // 0.5초 후 IO18 HIGH
    }
    
    // 상태 변경 메시지 출력
    Serial.print("상태 변경: ");
    Serial.print(isConnected ? "연결 됨" : "연결 해제");
    Serial.print(" (전압: ");
    Serial.print(voltage);
    Serial.println("V)");
    
    lastState = isConnected;
  }
  
  // IO 제어 타이밍 처리
  if (isConnected && !isDisconnecting) {
    // 연결 상태일 때
    if (currentMillis >= io17Delay && !io17State) {
      digitalWrite(io17Pin, HIGH);
      io17State = true;
    }
    if (currentMillis >= io18Delay && !io18State) {
      digitalWrite(io18Pin, HIGH);
      io18State = true;
    }
  } else if (isDisconnecting) {
    // 연결 해제 중일 때
    if (currentMillis >= io18Delay && io18State) {
      digitalWrite(io18Pin, LOW);
      io18State = false;
    }
    if (currentMillis >= io17Delay && io17State) {
      digitalWrite(io17Pin, LOW);
      io17State = false;
      isDisconnecting = false;
    }
  }
}
