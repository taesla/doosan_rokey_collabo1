// ==========================
// 핀 설정
// ==========================
const int STEP_PIN = 4;   // TB6600 PUL-
const int DIR_PIN  = 3;   // TB6600 DIR-
const int EN_PIN   = 2;   // TB6600 ENA-

const int SENSOR_PIN = 10;         // 근접센서 입력
const int SENSOR_ACTIVE = HIGH;    // 헴 센서는 HIGH = 감지

// ==========================
// 상태 관리 변수
// ==========================

// 로봇 쪽 신호 기다리는 중인지
bool waitingForRobot = false;

// DETECT 신호를 한 번만 보내기 위한 플래그
bool sentDetect = false;

// 비상정지 상태
bool emergencyStop = false;

// RUNNING 메시지 주기 제어
unsigned long lastRunMsgMillis = 0;

// ---- 스텝모터 속도 관련 ----
// 이제 1: 저속, 2: 중속만 사용
const unsigned long STEP_DELAY_SLOW  = 800;  // 느림 (값 클수록 느림)
const unsigned long STEP_DELAY_MID   = 500;  // 중간

int speedLevel = 2;                       // 1:slow, 2:mid
unsigned long stepDelayUs = STEP_DELAY_MID;
unsigned long lastStepMicros = 0;

// ---- 방향 관련 ----
bool dirForward = true;   // true: DIR_FWD, false: DIR_REV

// ---- 센서 디바운싱 관련 ----
bool sensorStable = false;         // 디바운싱된 센서 상태
bool lastRawSensor = false;        // 마지막 생(raw) 센서 상태
unsigned long sensorChangeTime = 0; // 상태 변한 시점(ms)

// ---- DETECT 카운트 (로깅용 ID) ----
unsigned long detectCount = 0;

void setup() {
  Serial.begin(9600);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(SENSOR_PIN, INPUT_PULLUP);

  // 모터 Enable ON (TB6600 기준: ENA- LOW)
  digitalWrite(EN_PIN, LOW);

  // 기본 방향: 정방향
  setDirection(true);

  digitalWrite(STEP_PIN, LOW);

  // 센서 디바운스 초기값
  lastRawSensor = (digitalRead(SENSOR_PIN) == SENSOR_ACTIVE);
  sensorStable = lastRawSensor;
  sensorChangeTime = millis();

  Serial.println("BOOT");  // 디버깅용
}

void loop() {
  // 1) 로봇 ↔ 컨베이어 통신: 명령 수신 체크
  handleSerialCommand();

  // 2) 센서 상태 디바운싱
  unsigned long nowMs = millis();
  bool raw = (digitalRead(SENSOR_PIN) == SENSOR_ACTIVE);

  // raw 상태가 바뀌면 타이머 리셋
  if (raw != lastRawSensor) {
    lastRawSensor = raw;
    sensorChangeTime = nowMs;
  }

  // 10ms 이상 같은 상태 유지되면 확정
  if (nowMs - sensorChangeTime > 10) {
    sensorStable = raw;
  }

  // 3) 아직 로봇을 기다리는 중이 아니고, 비상정지도 아닐 때만 센서 이벤트 처리
  if (!waitingForRobot && !emergencyStop) {
    if (sensorStable && !sentDetect) {
      // 물체가 디바운싱된 상태로 도착한 순간
      detectCount++;
      Serial.print("DETECT#");
      Serial.println(detectCount);   // 로깅용 ID 포함

      waitingForRobot = true;     // 이제 로봇 작업 끝날 때까지 대기
      sentDetect = true;          // 같은 물체에 대해 여러 번 안 보내도록
      // 모터는 아래에서 waitingForRobot 상태/비상정지 상태를 보고 자동으로 멈춤
    }
  }

  // 4) RUNNING 상태 메시지: 벨트가 실제로 움직이고 있을 때만 1초마다 전송
  if (!waitingForRobot && !emergencyStop) {
    if (nowMs - lastRunMsgMillis >= 1000) {
      lastRunMsgMillis = nowMs;
      Serial.println("RUNNING");
    }
  }

  // 5) 모터 회전 제어
  // waitingForRobot == true 이거나 emergencyStop == true 이면 모터를 정지(스텝 펄스 안 보냄)
  if (!waitingForRobot && !emergencyStop) {
    unsigned long nowUs = micros();
    if (nowUs - lastStepMicros >= stepDelayUs) {
      lastStepMicros = nowUs;

      // STEP 핀 토글로 펄스 발생
      digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
    }
  } else {
    // 정지 상태에서는 STEP을 LOW 유지 (원하는 경우)
    digitalWrite(STEP_PIN, LOW);
  }
}

// ==========================
// 방향 설정 함수
// ==========================
void setDirection(bool forward) {
  dirForward = forward;
  if (dirForward) {
    // 정방향
    digitalWrite(DIR_PIN, LOW);
    Serial.println("DIR_FWD_OK");
  } else {
    // 역방향
    digitalWrite(DIR_PIN, HIGH);
    Serial.println("DIR_REV_OK");
  }
}

// ==========================
// 속도 설정 함수
// ==========================
void setSpeedLevel(int level) {
  speedLevel = level;
  switch (speedLevel) {
    case 1:
      stepDelayUs = STEP_DELAY_SLOW;
      Serial.println("SPD1_OK");
      break;
    case 2:
      stepDelayUs = STEP_DELAY_MID;
      Serial.println("SPD2_OK");
      break;
    default:
      // 잘못된 값 오면 중속으로 고정
      speedLevel = 2;
      stepDelayUs = STEP_DELAY_MID;
      Serial.println("SPD_DEFAULT_MID");
      break;
  }
}

// ==========================
// 시리얼 명령 처리 함수
// ==========================
void handleSerialCommand() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.length() == 0) return;

  cmd.toUpperCase();

  // ---------- 비상정지 / 리셋 / 일반 STOP ----------
  if (cmd == "ESTOP") {
    // 비상정지: 무조건 멈추고, 다시 움직이려면 RESET 필요
    emergencyStop = true;
    waitingForRobot = true;  // 모터 회전 막기 위해
    Serial.println("ESTOP_OK");
    return;
  }

  if (cmd == "RESET") {
    // 비상정지 해제 + 상태 초기화
    emergencyStop = false;
    waitingForRobot = false;
    sentDetect = false;
    Serial.println("RESET_OK");
    return;
  }

  if (cmd == "STOP") {
    // 일반 정지: 즉시 멈추지만, RESUME으로 다시 시작 가능
    waitingForRobot = true;
    Serial.println("STOP_OK");
    return;
  }

  // ---------- RESUME ----------
  if (cmd == "RESUME") {
    // 로봇이 "물체이동완료" 작업 끝났다는 의미
    if (!emergencyStop) { // 비상정지 중에는 재개 불가
      waitingForRobot = false;
      sentDetect = false;         // 다음 물체에 대해 다시 DETECT 보낼 수 있게 초기화
      Serial.println("RESUME_OK"); // 디버깅용 응답
    } else {
      Serial.println("RESUME_FAIL_EMG"); // 비상정지 상태라 재개 불가
    }
    return;
  }

  // ---------- 속도 설정 ----------
  if (cmd == "SPD1") {
    setSpeedLevel(1);
    return;
  }
  if (cmd == "SPD2") {
    setSpeedLevel(2);
    return;
  }

  // SPD0, SPD3 등은 이제 아무 의미 없음 (버튼도 안 쓸 거라 굳이 처리 X)

  // ---------- 방향 설정 ----------
  if (cmd == "DIR_FWD") {
    setDirection(true);
    return;
  }
  if (cmd == "DIR_REV") {
    setDirection(false);
    return;
  }

  // ---------- 상태 질의 ----------
  if (cmd == "STATUS") {
    Serial.print("STATE:");
    Serial.print("WAITING=");
    Serial.print(waitingForRobot);
    Serial.print(", SPEED=");
    Serial.print(speedLevel);
    Serial.print(", DIR=");
    Serial.print(dirForward ? "FWD" : "REV");
    Serial.print(", SENSOR=");
    Serial.print(sensorStable ? "ON" : "OFF");
    Serial.print(", EMG=");
    Serial.print(emergencyStop ? "ON" : "OFF");
    Serial.print(", DETECTCNT=");
    Serial.println(detectCount);
    return;
  }

  // 필요하면 여기 아래에 추가 명령 확장 가능
}
