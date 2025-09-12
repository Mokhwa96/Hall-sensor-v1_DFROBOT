// DFRobot Hall Sensor V1 테스트 코드
// 신호핀은 Arduino의 D2번 핀에 연결했다고 가정

const int hallPin = 2;    // 센서 출력 연결 핀
int hallState = 0;        // 센서 상태 저장 변수

void setup() {
  pinMode(hallPin, INPUT);  // 홀 센서 핀을 입력으로 설정
  Serial.begin(9600);       // 시리얼 모니터 시작
  Serial.println("Hall Sensor Test Start!");
}

void loop() {
  hallState = digitalRead(hallPin);  // 센서 상태 읽기
  
  if (hallState == LOW) {
    // 자기장이 감지되면 LOW 출력
    Serial.println("No Magnet");
  } else {
    Serial.println("Magnet Detected!");
  }

  delay(500); // 0.5초 간격으로 출력
}
