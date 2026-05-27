// 하역 동작을 하는 동안 수신 버퍼에 값이 쌓이지 않고 계속 값을 받을 수 있게 하는 것
// Idle 일 때는 초기값 유지
// ROS 제어 동작일 때는 시리얼로 받은 값을 부드럽게 이동
// 하역 동작일 때는 정해진 동작을 수행하도록 설계
// 시리얼 PWM 입력 형식 : CMD_POS,1,2500,1253,1115,1650,1\n
// 시리얼 DEG 입력 형식 : CMD_POS,1,0.00,-77.62,75.00,3.12,1\n
// const int 부분 각도에 따른 초기값 설정

#include <Arduino.h> // Arduino 기본 함수 사용을 위한 라이브러리
#include <ESP32Servo.h> // ESP32에서 서보모터 제어를 위한 라이브러리
#include "esp_system.h" // ESP32 reset reason 확인을 위한 라이브러리

#define INPUT_DEGREE 0
#define INPUT_PWM 1
#define INPUT_INVALID 2

#define SERVO1_PIN 25 // 1번 서보모터를 GPIO 25번 핀에 연결 (360도) [회전을 담당하는 모터]
#define SERVO2_PIN 32 // 2번 서보모터를 GPIO 32번 핀에 연결 (270도) [어깨 관절]
#define SERVO3_PIN 33 // 3번 서보모터를 GPIO 33번 핀에 연결 (270도) [어깨 관절]
#define SERVO4_PIN 26 // 4번 서보모터를 GPIO 26번 핀에 연결 (180도) [팔꿈치 관절]
#define SERVO5_PIN 27 // 5번 서보모터를 GPIO 27번 핀에 연결 (180도) [엔드 이펙터 관절]
Servo servo1; // 1번 서보모터 객체 생성
Servo servo2; // 2번 서보모터 객체 생성
Servo servo3; // 3번 서보모터 객체 생성
Servo servo4; // 4번 서보모터 객체 생성
Servo servo5; // 5번 서보모터 객체 생성

#define MODE_IDLE 0 // 기본 상태
#define MODE_ROS 1 // ROS 제어 동작
#define MODE_UNLOAD 2 // 하역 동작
#define MODE_ROS_RELEASE 3 // ROS 제어권 해제
#define MODE_UNLOADING 4 // 하역 동작 진행 중
int currentMode = MODE_ROS; // 현재 동작 모드를 저장하는 변수

const int PWM_MIN = 500; // 서보모터에 줄 수 있는 최소 PWM 값
const int PWM_MAX = 2500; // 서보모터에 줄 수 있는 최대 PWM 값


const int ROTATION_PWN_MIN_0 = 500;
const int ROTATION_PWN_MAX_0 = 2500;
const float ROTATION_DEG_MIN_0 = -180.0;
const float ROTATION_DEG_MAX_0 = 0.0;

const int JOINT_PWN_MIN_1 = 910;
const int JOINT_PWN_MAX_1 = 2400;
const float JOINT_DEG_MIN_1 = -90.0;
const float JOINT_DEG_MAX_1 = 115.0;

const int JOINT_PWN_MIN_2 = 850;
const int JOINT_PWN_MAX_2 = 2500;
const float JOINT_DEG_MIN_2 = -90.0;
const float JOINT_DEG_MAX_2 = 75.0;

const int JOINT_PWN_MIN_3 = 860;
const int JOINT_PWN_MAX_3 = 1900;
const float JOINT_DEG_MIN_3 = -24.0;
const float JOINT_DEG_MAX_3 = 70.0;

int targetValue[4]; // 목표 PWM 값을 저장하는 배열
float presentValue[4]; // 현재 PWM 값을 저장하는 배열
float startValue[4]; // 이동 시작 시점의 PWM 값을 저장하는 배열
int inputPWMValue[4]; // 시리얼 입력에서 파싱된 PWM 값을 저장하는 배열
int initialValue[4] = {2500,1253,1115,1650}; // ★★★★★ 초기 로봇팔 기본 자세 PWM 값을 저장하는 배열 ★★★★★

const int INPUT_BUFFER_MAX = 128; // 시리얼 입력 버퍼 최대 크기

unsigned long moveStartTime = 0; // 이동이 시작된 시간을 저장하는 변수
unsigned long MOVE_DURATION = 4000; // 목표 위치까지 이동하는 데 걸리는 시간(ms) (값이 클수록 더 느리고 부드럽게 움직임)

String inputBuffer = ""; // 시리얼 모니터에서 입력받은 문자열을 저장하는 변수


// ESP32 리셋 원인 문자열 변환 함수 --------------------------------------------------------------
const char* resetReasonToString(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_UNKNOWN:   return "UNKNOWN";
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
#ifdef ESP_RST_SDIO
    case ESP_RST_SDIO:      return "SDIO";
#endif
#ifdef ESP_RST_USB
    case ESP_RST_USB:       return "USB";
#endif
    default:                return "OTHER";
  }
}


// MAP 함수 (float 버전)
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// 서보모터 제어 함수들 --------------------------------------------------------------
void writeServoOutput(){ // 현재 PWM 값을 실제 서보모터에 출력하는 함수
  servo1.writeMicroseconds((int)presentValue[0]); // CH1 현재 값을 1번 서보모터에 출력
  servo2.writeMicroseconds((int)presentValue[1]); // CH2 현재 값을 2번 서보모터에 출력
  int reversedCH2 = (PWM_MIN + PWM_MAX) - (int)presentValue[1] + 45; // 2번 서보와 반대로 움직이는 값을 계산 [예: 700이면 2300 방향, 2300이면 700 방향]
  servo3.writeMicroseconds(reversedCH2); // 반전된 CH2 값을 3번 서보모터에 출력
  servo4.writeMicroseconds((int)presentValue[2]); // CH3 현재 값을 4번 서보모터에 출력
  servo5.writeMicroseconds((int)presentValue[3]); // CH4 현재 값을 5번 서보모터에 출력
}


float easeInOut(float t){ // 부드러운 가감속 값을 계산하는 함수
  return t * t * t * (t * (6 * t - 15) + 10); // 처음에는 천천히, 중간에는 빠르게, 끝에서는 다시 천천히 움직이게 함
}


// 서보모터를 부드럽게 이동시키는 함수
void smoothServoMove() {
  // (지난 시간 - 시작 시간) / 전체 이동 시간 = 진행률 계산 [t는 0.0에서 1.0까지 증가]
  float t = (float)(millis() - moveStartTime) / MOVE_DURATION;
  // 이동 진행률이 100% 이상이면 진행률을 1.0으로 고정
  if (t >= 1.0) t = 1.0;
  // 부드러운 가감속이 적용된 진행률 계산
  float e = easeInOut(t);

   // CH1~CH4 반복 [시작값에서 목표값까지 부드럽게 보간하여 현재값 계산]
  for (int ch = 0; ch < 4; ch++) presentValue[ch] = startValue[ch] + (targetValue[ch] - startValue[ch]) * e;
  writeServoOutput(); // 계산된 현재값을 실제 서보모터에 출력
}

// 모드 변경 함수 ---------------------------------------------------------------
      // inputMode = MODE_IDLE 일 때
        // currentMode = MODE_IDLE → MODE_IDLE로 유지
        // currentMode = MODE_ROS → MODE_ROS 그대로 유지
        // currentMode = MODE_UNLOAD → MODE_UNLOAD 그대로 유지
        // currentMode = MODE_ROS_RELEASE → MODE_ROS_RELEASE 그대로 유지
        // currentMode = MODE_UNLOADING → MODE_UNLOADING 그대로 유지

      // inputMode = MODE_ROS 일 때
        // currentMode = MODE_IDLE → MODE_ROS로 변경
        // currentMode = MODE_ROS → MODE_ROS 그대로 유지
        // currentMode = MODE_UNLOAD → MODE_UNLOAD 그대로 유지
        // currentMode = MODE_ROS_RELEASE → MODE_ROS_RELEASE 그대로 유지
        // currentMode = MODE_UNLOADING → MODE_UNLOADING 그대로 유지

      // inputMode = MODE_UNLOAD 일 때
        // currentMode = MODE_IDLE → MODE_UNLOAD로 변경
        // currentMode = MODE_ROS → MODE_ROS 그대로 유지
        // currentMode = MODE_UNLOAD → MODE_UNLOAD 그대로 유지
        // currentMode = MODE_ROS_RELEASE → MODE_ROS_RELEASE 그대로 유지
        // currentMode = MODE_UNLOADING → MODE_UNLOADING 그대로 유지
      
      // inputMode = MODE_ROS_RELEASE 일 때
        // currentMode = MODE_IDLE → MODE_IDLE 그대로 유지
        // currentMode = MODE_ROS → MODE_IDLE로 변경
        // currentMode = MODE_UNLOAD → MODE_UNLOAD 그대로 유지
        // currentMode = MODE_ROS_RELEASE → MODE_ROS_RELEASE 그대로 유지
        // currentMode = MODE_UNLOADING → MODE_UNLOADING 그대로 유지

      // inputMode = MODE_UNLOADING 일 때
        // currentMode = MODE_IDLE → MODE_IDLE 그대로 유지
        // currentMode = MODE_ROS → MODE_ROS 그대로 유지
        // currentMode = MODE_UNLOAD → MODE_UNLOAD 그대로 유지
        // currentMode = MODE_ROS_RELEASE → MODE_ROS_RELEASE 그대로 유지
        // currentMode = MODE_UNLOADING → MODE_UNLOADING 그대로 유지

      // 하역 동작
        // 시작을 알림 (Arduino → ROS), MODE_UNLOADING로 변경
        // 동작 중 입력은 무시 (UNLOADING 상태에서는 입력이 들어와도 MODE_UNLOADING 유지)
        // 동작 후 현재 위치를 MODE_IDLE로 변경
        // 동작 후 종료를 알림 (Arduino → ROS)
      // ROS 제어 동작
        // 시작을 알림 (Arduino → ROS)
        // 동작 중 입력은 계속 받아서 부드럽게 이동
        // 값을 MODE_ROS_RELEASE를 받으면 ROS 제어권 해제, MODE_IDLE로 변경
        // 동작 후 종료를 알림 (Arduino → ROS)
void changeMode(int inputMode) {
  if(inputMode < MODE_IDLE || inputMode > MODE_UNLOADING) {
    Serial.println("ERR,INVALID_MODE");
    return;
  }
  // inputMode = MODE_IDLE 일 때 어떤 상태에서도 현재 모드 유지
  if (inputMode == MODE_IDLE) {
    Serial.println("IDLE_MODE"); // IDLE 모드 진입 알림
    currentMode = currentMode;
  }

  // inputMode = MODE_ROS 일 때 IDLE 상태일 때만 ROS 모드로 진입
  else if (inputMode == MODE_ROS) {
    if (currentMode == MODE_IDLE) {
      currentMode = MODE_ROS;
      Serial.println("ROS_CONTROL_START"); // ROS 제어권 획득 알림
    }
  }

  // inputMode = MODE_UNLOAD 일 때 IDLE 상태일 때만 UNLOAD 모드로 진입
  else if (inputMode == MODE_UNLOAD) {
    if (currentMode == MODE_IDLE) {
      currentMode = MODE_UNLOAD;
    }
  }

  // inputMode = MODE_ROS_RELEASE 일 때 ROS 모드일 때만 IDLE로 복귀
  else if (inputMode == MODE_ROS_RELEASE) {
    if (currentMode == MODE_ROS) {
      Serial.println("ROS_CONTROL_RELEASE"); // ROS 제어권 해제 알림
      currentMode = MODE_IDLE;
    }
  }

  // inputMode = MODE_UNLOADING 일 때 어떤 상태에서도 현재 모드 유지
  else if (inputMode == MODE_UNLOADING) {
    currentMode = currentMode;
  }
}


// 검증 함수 --------------------------------------------------------------
bool isValidInteger(String s){
  if (s.length() == 0) return false; // 문자열 길이가 0이면 종료
  int startIdx = 0; // 숫자 검사를 시작할 위치
  for (int i = startIdx; i < s.length(); i++) if (!isDigit(s[i])) return false;
  return true;
}
bool isPWMRangeValid(float TempPWMValue[4]){
  if ((int)TempPWMValue[0] < ROTATION_PWN_MIN_0 || (int)TempPWMValue[0] > ROTATION_PWN_MAX_0) return false;
  if ((int)TempPWMValue[1] < JOINT_PWN_MIN_1 || (int)TempPWMValue[1] > JOINT_PWN_MAX_1) return false;
  if ((int)TempPWMValue[2] < JOINT_PWN_MIN_2 || (int)TempPWMValue[2] > JOINT_PWN_MAX_2) return false;
  if ((int)TempPWMValue[3] < JOINT_PWN_MIN_3 || (int)TempPWMValue[3] > JOINT_PWN_MAX_3) return false;
  return true;
}
bool isDegreeRangeValid(float TempPWMValue[4]){
  if(TempPWMValue[0] < ROTATION_DEG_MIN_0 || TempPWMValue[0] > ROTATION_DEG_MAX_0) return false;
  if(TempPWMValue[1] < JOINT_DEG_MIN_1 || TempPWMValue[1] > JOINT_DEG_MAX_1) return false;
  if(TempPWMValue[2] < JOINT_DEG_MIN_2 || TempPWMValue[2] > JOINT_DEG_MAX_2) return false;
  if(TempPWMValue[3] < JOINT_DEG_MIN_3 || TempPWMValue[3] > JOINT_DEG_MAX_3) return false;
  return true;
}

int detectInputType(float values[4]) {
  bool all_pwm = true;
  bool all_degree = true;

  for (int i = 0; i < 4; i++)
  {
    float value = values[i];
    float absValue = abs(value);
    if (!(value >= 500 && value <= 2500)) all_pwm = false; // PWM 기준: 500 ~ 2500
    if (!(absValue <= 180)) all_degree = false; // Degree 기준: 절댓값 180 이하
  }

  if (all_pwm) return INPUT_PWM;
  if (all_degree) return INPUT_DEGREE;
  return INPUT_INVALID;
}


// 시리얼 명령 처리 함수 --------------------------------------------------------------
int splitCsv(const String &line, String *out, int max_parts){
  int count = 0;
  int start = 0;

  while (count < max_parts)
  {
    int comma = line.indexOf(',', start);

    if (comma == -1)
    {
      out[count++] = line.substring(start);
      break;
    }

    out[count++] = line.substring(start, comma);
    start = comma + 1;
  }

  return count;
}

// ROTATION_PWN_MIN_0[ROTATION_DEG_MIN_0] ~ ROTATION_PWN_MAX_0[ROTATION_DEG_MAX_0]
// JOINT_PWN_MIN_1[JOINT_DEG_MIN_1] ~ JOINT_PWN_MAX_1[JOINT_DEG_MAX_1]
// JOINT_PWN_MIN_2[JOINT_DEG_MAX_2] ~ JOINT_PWN_MAX_2[JOINT_DEG_MIN_2]
// JOINT_PWN_MIN_3[JOINT_DEG_MAX_3] ~ JOINT_PWN_MAX_3[JOINT_DEG_MIN_3]


unsigned long last_cmd_time = 0; // 마지막 CMD_POS 명령이 처리된 시간을 저장하는 변수 (ms 단위)
void parseCmdPos(const String &cmd){
  // Expected:
  // CMD_POS,seq,j1_deg,j2_deg,j3_deg,j4_deg,Flags
  String parts[7];
  int part_count = splitCsv(cmd, parts, 7);

  if (part_count < 7){
    Serial.println("ERR,CMD_POS_FORMAT");
    return;
  }
  String seq = parts[1];
  float TempPWMValue[4] = {0,};
  TempPWMValue[0] = parts[2].toFloat();
  TempPWMValue[1] = parts[3].toFloat();
  TempPWMValue[2] = parts[4].toFloat();
  TempPWMValue[3] = parts[5].toFloat();

  int inputType = detectInputType(TempPWMValue);
  changeMode(parts[6].toInt()); // Flags 부분을 정수로 변환하여 모드 변경 함수에 전달
  if (inputType == INPUT_INVALID) {
    Serial.println("ERR,CMD_POS_VALUE");
    return;
  }
  else if (inputType == INPUT_DEGREE) { // Degree 입력인 경우 PWM으로 변환
    if(!isDegreeRangeValid(TempPWMValue)) {
    Serial.println("ERR,CMD_DEG_POS_RANGE");
    return;
    }
    inputPWMValue[0] = mapFloat(TempPWMValue[0], ROTATION_DEG_MIN_0, ROTATION_DEG_MAX_0, ROTATION_PWN_MIN_0, ROTATION_PWN_MAX_0);
    inputPWMValue[1] = mapFloat(TempPWMValue[1], JOINT_DEG_MIN_1, JOINT_DEG_MAX_1, JOINT_PWN_MIN_1, JOINT_PWN_MAX_1);
    inputPWMValue[2] = mapFloat(TempPWMValue[2], JOINT_DEG_MIN_2, JOINT_DEG_MAX_2, JOINT_PWN_MAX_2, JOINT_PWN_MIN_2); // JOINT 2는 DEG와 PWM이 반비례 관계이므로 MAX와 MIN이 뒤바뀜
    inputPWMValue[3] = mapFloat(TempPWMValue[3], JOINT_DEG_MIN_3, JOINT_DEG_MAX_3, JOINT_PWN_MAX_3, JOINT_PWN_MIN_3); // JOINT 3는 DEG와 PWM이 반비례 관계이므로 MAX와 MIN이 뒤바뀜
  }
  else if (inputType == INPUT_PWM) { // PWM 입력인 경우 그대로 사용
    if(!isPWMRangeValid(TempPWMValue)) {
    Serial.println("ERR,CMD_PWM_POS_RANGE");
    return;
    }
    for (int i = 0; i < 4; i++) inputPWMValue[i] = (int)TempPWMValue[i];
  }


  last_cmd_time = millis();

  Serial.print("ACK,");
  Serial.println(seq);
}




// 현재 PWM 값을 시리얼 모니터에 출력하는 함수
void printPresent(){
  // 각 채널 현재 PWM 값 출력
  Serial.print("CH1 Current value: ");
  Serial.println("PWM :" + String((int)presentValue[0]) + " / DEG :" + String(mapFloat(presentValue[0], ROTATION_PWN_MIN_0, ROTATION_PWN_MAX_0, ROTATION_DEG_MIN_0, ROTATION_DEG_MAX_0), 2));
  Serial.print("CH2 Current value: ");
  Serial.println("PWM :" + String((int)presentValue[1]) + " / DEG :" + String(mapFloat(presentValue[1], JOINT_PWN_MIN_1, JOINT_PWN_MAX_1, JOINT_DEG_MIN_1, JOINT_DEG_MAX_1), 2));
  Serial.print("CH3 Current value: ");
  Serial.println("PWM :" + String((int)presentValue[2]) + " / DEG :" + String(mapFloat(presentValue[2], JOINT_PWN_MAX_2, JOINT_PWN_MIN_2, JOINT_DEG_MIN_2, JOINT_DEG_MAX_2), 2));
  Serial.print("CH4 Current value: ");
  Serial.println("PWM :" + String((int)presentValue[3]) + " / DEG :" + String(mapFloat(presentValue[3], JOINT_PWN_MAX_3, JOINT_PWN_MIN_3, JOINT_DEG_MIN_3, JOINT_DEG_MAX_3), 2));
  Serial.print("currentMode: ");
  Serial.println(currentMode);
  Serial.println("------------------------------");
}
// 모드별 동작 함수 --------------------------------------------------------------
bool idleMoveStarted = false;

void handleIdleMode() {
  // IDLE 상태가 아니면 다음 IDLE 진입을 위해 초기화
  if (currentMode != MODE_IDLE) {
    idleMoveStarted = false;
    return;
  }
  
  // IDLE 상태에 처음 진입했을 때만 실행
    // ★★★★★ startValue을 현재값으로 설정, targetValue를 초기값으로 설정 ★★★★★
  if (idleMoveStarted == false) {
    moveStartTime = millis();

    for (int ch = 0; ch < 4; ch++) {
      targetValue[ch] = initialValue[ch]; // 목표값을 초기값으로 설정
      startValue[ch] = presentValue[ch]; // 현재값을 시작값으로 저장
    }
    MOVE_DURATION = 4000;
    idleMoveStarted = true;
  }
  smoothServoMove();
}

void handleRosMode() {
  // ROS 제어 모드에서는 시리얼로 받은 값을 부드럽게 이동
  if (currentMode == MODE_ROS) {
    for (int ch = 0; ch < 4; ch++) {
      targetValue[ch] = inputPWMValue[ch]; // 목표값을 시리얼 입력값으로 설정
      if(targetValue[ch] > presentValue[ch]) presentValue[ch] = presentValue[ch] + 1; // 현재값이 목표값보다 작으면 1씩 증가
      else if(targetValue[ch] < presentValue[ch]) presentValue[ch] = presentValue[ch] - 1; // 현재값이 목표값보다 크면 1씩 감소
    }
    writeServoOutput(); // 계산된 현재값을 실제 서보모터에 출력
  }
  else return; // ROS 제어 모드가 아니면 함수 종료
}


unsigned long unloadingStartTime = 0;

int unloadingStep = -1;
unsigned int unloadingStepTable[8][4] = {
  {560,1253,1115,1650}, // 반대편 이동
  {560,1747,1282,1851}, // [들기 전 자세]
  {560,1850,2000,1900}, // [들기 동작]

  {545,1650,2000,1900}, // [들고 위로 이동하는 자세]
  {1500,1650,2000,1900}, // [들고 옆으로 이동하는 자세]

  
  {1500,2227,2278,1886}, // [팔을 뻗는 자세]
  {1500,2227,2278,1200}, // [음식을 내리는 자세]
  {1500,1253,1115,1650} // [팔을 1500의 원위치 자세]
};

void handleUnloadMode() {
  if (currentMode != MODE_UNLOAD && currentMode != MODE_UNLOADING) return;
  currentMode = MODE_UNLOADING;

  // unloadingStep이 -1이면 하역이 시작되지 않은 상태이므로 하역 시작을 알리고 MODE_UNLOADING으로 변경
  // unloadingStartTime을 현재 시간으로 설정하여 하역 진행 시간을 측정하는 데 사용
  if (unloadingStartTime == 0) {
    unloadingStartTime = millis();
    unloadingStep = -1; // 하역 시작 시 unloadingStep 초기화
    Serial.println("UNLOADING START");
  }

  unsigned long elapsed = millis() - unloadingStartTime;
  int nextStep = -1; // nextStep 초기화

  // 진행 시간에 따라 하역 단계 결정
    // 하역 완료 후 MODE_IDLE로 변경, UNLOADING 단계 초기화, 하역 완료 알림
  if (elapsed < 4000) nextStep = 0;
  else if (elapsed < 8000) nextStep = 1; 
  else if (elapsed < 12000) nextStep = 2;
  else if (elapsed < 16000) nextStep = 3;
  else if (elapsed < 20000) nextStep = 4;
  else if (elapsed < 24000) nextStep = 5;
  else if (elapsed < 28000) nextStep = 6;
  else if (elapsed < 32000) nextStep = 7;
  else {
    currentMode = MODE_IDLE;
    Serial.println("UNLOADING COMPLETE");
    // 하역 완료 후 
    unloadingStartTime = 0; // 하역 총 동작 시간 초기화
    unloadingStep = -1; // 하역 단계 초기화
    return;
  }

  // unloadingStep이 nextStep과 다르면 하역 단계가 변경된 것
    // unloadingStep을 nextStep으로 업데이트하여 현재 하역 단계 저장
    // moveStartTime을 현재 시간으로 설정하여 모터 동작 알고리즘에서 이동 시작 시간을 업데이트
    // targetValue를 unloadingStepTable에서 nextStep에 해당하는 값으로 설정하여 다음 하역 단계의 목표값으로 사용
    // startValue를 현재값으로 설정하여 모터 동작 알고리즘에서 이동 시작 시점의 값을 저장
  if (unloadingStep != nextStep) {
    unloadingStep = nextStep;

    moveStartTime = millis(); // 모터 동작 알고리즘에 사용되는 이동 시작 시간 업데이트
    MOVE_DURATION = 4000;

    for (int ch = 0; ch < 4; ch++) {
      targetValue[ch] = unloadingStepTable[nextStep][ch];
      startValue[ch] = presentValue[ch];
    }
  }

  smoothServoMove();
}

// 프로그램 초기 설정을 수행하는 함수 ----------------------------------------------
void setup(){
  Serial.begin(115200); // 시리얼 통신 속도를 115200bps로 설정
  delay(1000); // 시리얼 통신 안정화를 위해 1초 대기

  esp_reset_reason_t resetReason = esp_reset_reason();
  Serial.print("BOOT,reset_reason=");
  Serial.print((int)resetReason);
  Serial.print(",");
  Serial.println(resetReasonToString(resetReason));

  // 서보모터 PWM 주파수 설정
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50); 
  servo3.setPeriodHertz(50); 
  servo4.setPeriodHertz(50);
  servo5.setPeriodHertz(50); 

  // 각 서보모터 지정 핀 연결 및 PWM 범위 설정
  servo1.attach(SERVO1_PIN, PWM_MIN, PWM_MAX);
  servo2.attach(SERVO2_PIN, PWM_MIN, PWM_MAX);
  servo3.attach(SERVO3_PIN, PWM_MIN, PWM_MAX);
  servo4.attach(SERVO4_PIN, PWM_MIN, PWM_MAX);
  servo5.attach(SERVO5_PIN, PWM_MIN, PWM_MAX);

  for(int ch = 0; ch < 4; ch++) {
    presentValue[ch] = initialValue[ch]; // 현재값을 초기값으로 설정
    targetValue[ch] = initialValue[ch]; // 목표값을 초기값으로 설정
    inputPWMValue[ch] = initialValue[ch]; // PWM 입력값을 초기값으로 설정
  }

  Serial.println("Input format: v1,v2,v3,v4,inputMode"); // 입력 형식 안내
  Serial.println("Type 'present' to show current values"); // present 입력 시 현재값 확인 가능 안내
  Serial.println("------------------------------");
}


// 메인 루프 함수: 시리얼 입력을 처리하고 서보모터를 제어하는 함수 --------------------------------------------------------------
void loop() {
  // static unsigned long last_state_time = 0;
  // if (millis() - last_state_time >= 100){
  // last_state_time = millis();
  // Serial.print("STATE,");
  // Serial.print(millis());
  // Serial.print(",");
  // Serial.print(presentValue[0], 3);
  // Serial.print(",");
  // Serial.print(presentValue[1], 3);
  // Serial.print(",");
  // Serial.print(presentValue[2], 3);
  // Serial.print(",");
  // Serial.print(presentValue[3], 3);
  // Serial.println(",OK");
  // }


  handleIdleMode();
  handleRosMode();
  handleUnloadMode();

  while (Serial.available() > 0) {
    // 시리얼 입력에서 한 문자씩 읽어서 inputBuffer에 저장 ----------------------------------------------
    char c = Serial.read();
    if (c != '\n' && c != '\r') {
      if (inputBuffer.length() < INPUT_BUFFER_MAX) {
        inputBuffer += c;
      }
      else {
        inputBuffer = "";
        Serial.println("FAULT: input too long");
      }
      continue;
    }
    if (inputBuffer.length() == 0) continue; // 빈 입력 무시

    // 입력 문자열에 따른 동작 처리 --------------------------------------------------------------

    if (inputBuffer.equalsIgnoreCase("present")) {
      printPresent();
      inputBuffer = "";
      continue;
    }

    if (inputBuffer == "PING"){
    Serial.println("PONG");
    inputBuffer = "";
    continue;
    }

    if (inputBuffer == "STOP"){
      Serial.println("ACK_STOP");
      inputBuffer = "";
      continue;
    }

    if (inputBuffer.startsWith("CMD_POS")){
      parseCmdPos(inputBuffer);
      inputBuffer = "";
      continue;
    }

    Serial.print("ERR,UNKNOWN,");
    Serial.println(inputBuffer);

    inputBuffer = "";
    continue;
  }
}