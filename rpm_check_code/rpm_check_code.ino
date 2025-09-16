// Dual Hall RPM (UNO, 180° placement) — high-resolution (period-based)
// Wiring: A->D2(INT0), B->D3(INT1), VCC->5V, GND->GND
// Logic: magnet detected -> FALLING (LOW)

#include <Arduino.h>

const uint8_t HALL_A_PIN = 2;   // INT0
const uint8_t HALL_B_PIN = 3;   // INT1

// 자석 1개 × 센서 2개(180°) = 2 펄스/회전
// 자석 2개(180°) × 센서 2개 = 4  (필요시 4로 변경)
const uint8_t COMBINED_PPR = 2;

// 출력 주기(표시 주기) — 계산은 이벤트(펄스)마다 갱신됨
const unsigned long PRINT_MS     = 250;
const unsigned long DEBOUNCE_US  = 1000;   // 채터링 억제(us)
const unsigned long TIMEOUT_MS   = 1000;   // 이 시간 동안 펄스 없으면 0rpm 처리

// 예상 RPM 범위(노이즈/이상치 거르기용): 프로젝트에 맞춰 조정 가능
const uint16_t MAX_RPM_EXPECTED  = 2000;   // 상한
const uint16_t MIN_RPM_EXPECTED  = 10;     // 하한

// 최근 간격 버퍼 (작을수록 반응 빠름, 클수록 안정적)
const uint8_t  INTERVAL_BUF = 8;

volatile unsigned long lastEdgeUs = 0;                 // 마지막 펄스 시간(µs)
volatile unsigned long lastPulseUsA = 0, lastPulseUsB = 0; // 센서별 디바운스 기준

volatile unsigned long intervals[INTERVAL_BUF];        // 펄스 간격(µs) 링버퍼
volatile uint8_t       isrIdx = 0;
volatile uint8_t       filled = 0;

unsigned long minDtUs, maxDtUs; // 허용 간격 범위(µs) — setup에서 계산

inline void handlePulse(volatile unsigned long &lastSensorUs) {
  unsigned long now = micros();
  if ((now - lastSensorUs) < DEBOUNCE_US) return; // 센서별 디바운스
  lastSensorUs = now;

  unsigned long prev = lastEdgeUs;
  lastEdgeUs = now;

  if (prev == 0) return; // 첫 펄스는 간격 계산 불가

  unsigned long dt = now - prev; // µs 간격(센서 A/B 합산 기준: 1/COMBINED_PPR 회전)

  // RPM 기대 범위 밖(너무 짧거나 너무 긴 간격) 노이즈 제거
  if (dt < minDtUs || dt > maxDtUs) return;

  // 간단한 링버퍼 push (ISR 내 연산 최소화)
  intervals[isrIdx] = dt;
  isrIdx = (isrIdx + 1) % INTERVAL_BUF;
  if (filled < INTERVAL_BUF) filled++;
}

void hallA_ISR() { handlePulse(lastPulseUsA); }
void hallB_ISR() { handlePulse(lastPulseUsB); }

void setup() {
  Serial.begin(115200);
  pinMode(HALL_A_PIN, INPUT_PULLUP);
  pinMode(HALL_B_PIN, INPUT_PULLUP);

  // 기대 RPM → 허용 간격(µs) 범위로 환산
  // dt_per_pulse = 60e6 / (COMBINED_PPR * RPM)
  minDtUs = 60000000UL / (COMBINED_PPR * (unsigned long)MAX_RPM_EXPECTED);
  maxDtUs = 60000000UL / (COMBINED_PPR * (unsigned long)MIN_RPM_EXPECTED);

  attachInterrupt(digitalPinToInterrupt(HALL_A_PIN), hallA_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(HALL_B_PIN), hallB_ISR, FALLING);

  Serial.println(F("Dual Hall RPM (UNO, period-based) start"));
}

float computeRpmFromIntervals() {
  // intervals[]를 복사해서 트림드 평균(최소/최대 1개씩 제거) 후 RPM 계산
  unsigned long buf[INTERVAL_BUF];
  uint8_t n;

  noInterrupts();
  n = filled;
  for (uint8_t i = 0; i < n; i++) buf[i] = intervals[i];
  unsigned long lastUs = lastEdgeUs;
  interrupts();

  if (n == 0) return 0.0f;

  // 타임아웃: 최근 펄스가 오래 없으면 0rpm
  if ((micros() - lastUs) > (TIMEOUT_MS * 1000UL)) return 0.0f;

  // 합/최소/최대
  unsigned long sum = 0, mn = 0xFFFFFFFFUL, mx = 0;
  for (uint8_t i = 0; i < n; i++) {
    unsigned long v = buf[i];
    sum += v;
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }

  // 트림드 평균 (표본 3개 이상일 때만 적용)
  float avgDt;
  if (n >= 3) {
    avgDt = (float)(sum - mn - mx) / (float)(n - 2);
  } else {
    avgDt = (float)sum / (float)n;
  }

  // RPM = 60e6 / (avgDt_us * COMBINED_PPR)
  float rpm = 60000000.0f / (avgDt * (float)COMBINED_PPR);
  return rpm;
}

void loop() {
  static unsigned long lastPrint = 0;
  unsigned long nowMs = millis();

  if (nowMs - lastPrint >= PRINT_MS) {
    lastPrint += PRINT_MS;

    float rpm = computeRpmFromIntervals();

    // 디버깅용: 버퍼 채움 정도도 같이 표시
    noInterrupts();
    uint8_t n = filled;
    unsigned long lastUs = lastEdgeUs;
    unsigned long minU = minDtUs, maxU = maxDtUs;
    interrupts();
  if (rpm != 0) {
    Serial.print(F("RPM: "));
    Serial.print(rpm, 1);
    Serial.print(F("  (samples="));
    Serial.print(n);
    Serial.print(F(", last_dt_ok="));
    Serial.print((micros() - lastUs) <= (TIMEOUT_MS * 1000UL) ? F("Y") : F("N"));
    Serial.print(F(", dt_range[us]="));
    Serial.print(minU);
    Serial.print(F("~"));
    Serial.print(maxU);
    Serial.println(F(")"));
  }
  }
}
