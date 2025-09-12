// DFRobot Hall Sensor V1 - RPM Counter (Interrupt ver.)
// Wiring: VCC->5V, GND->GND, SIG->D2 (INT0 on UNO)
// Logic: Magnet detected -> FALLING (LOW)

const uint8_t HALL_PIN = 2;          // 센서 출력 핀 (인터럽트 가능 핀)
const uint8_t PPR = 1;               // Pulses Per Revolution: 자석 개수(또는 유효 엣지 수)
const unsigned long SAMPLE_MS = 500; // 샘플 윈도우(ms) — 500ms 권장
const unsigned long DEBOUNCE_US = 1000; // 노이즈 무시 구간(us) ~1ms

volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseUs = 0;

void hallISR() {
  unsigned long now = micros();
  if (now - lastPulseUs >= DEBOUNCE_US) { // 채터링/노이즈 제거
    pulseCount++;
    lastPulseUs = now;
  }
}

void setup() {
  pinMode(HALL_PIN, INPUT_PULLUP);        // 없음=HIGH, 자석 감지=LOW
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);
  Serial.println("Hall RPM counter start");
}

void loop() {
  static unsigned long lastSampleMs = 0;
  unsigned long now = millis();

  if (now - lastSampleMs >= SAMPLE_MS) {
    lastSampleMs += SAMPLE_MS;

    noInterrupts();
    unsigned long count = pulseCount;
    pulseCount = 0;
    interrupts();

    // count(샘플윈도우 내 펄스 수) -> RPM 변환
    // RPM = (count * (60000 / SAMPLE_MS)) / PPR
    float rpm = (count * (60000.0f / SAMPLE_MS)) / PPR;

    Serial.print("RPM: ");
    Serial.println(rpm, 1);
  }
}
