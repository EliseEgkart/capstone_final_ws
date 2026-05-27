#include <ESP32Servo.h>

#define SERVO_PIN 17

Servo myservo;

const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

const int SERVO_FULL_RANGE = 270;
const int SAFE_MIN = 0;
const int SAFE_MAX = 230;

int currentAngle = 0;

int angleToUs(int angle)
{
  angle = constrain(angle, SAFE_MIN, SAFE_MAX);
  return map(angle, 0, SERVO_FULL_RANGE, SERVO_MIN_US, SERVO_MAX_US);
}

void writeServoAngle(int angle)
{
  angle = constrain(angle, SAFE_MIN, SAFE_MAX);
  myservo.writeMicroseconds(angleToUs(angle));
}

void moveSmooth(int targetAngle)
{
  targetAngle = constrain(targetAngle, SAFE_MIN, SAFE_MAX);

  if (targetAngle == currentAngle)
    return;

  int stepDir = (targetAngle > currentAngle) ? 1 : -1;

  while (currentAngle != targetAngle)
  {
    currentAngle += stepDir;
    writeServoAngle(currentAngle);

    int remaining = abs(targetAngle - currentAngle);

    // 시작/끝은 천천히, 중간은 조금 빠르게
    if (remaining > 40)
      delay(8);
    else if (remaining > 20)
      delay(12);
    else if (remaining > 10)
      delay(16);
    else
      delay(22);
  }
}

void setup()
{
  myservo.setPeriodHertz(50);
  myservo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);

  currentAngle = 0;
  writeServoAngle(currentAngle);
  delay(1000);

  randomSeed(micros());
}

void loop()
{
  // 1. 전체 부드러운 왕복
  moveSmooth(230);
  delay(300);
  moveSmooth(0);
  delay(300);

  // 2. 중간 구간 왕복
  moveSmooth(60);
  moveSmooth(170);
  moveSmooth(100);
  moveSmooth(200);
  moveSmooth(30);

  // 3. 작은 진동
  for (int i = 0; i < 4; i++)
  {
    moveSmooth(110);
    moveSmooth(135);
  }

  // 4. 랜덤 위치 이동
  for (int i = 0; i < 6; i++)
  {
    int target = random(20, 211);
    moveSmooth(target);
    delay(150);
  }

  // 5. 끝점 근처 테스트
  moveSmooth(210);
  moveSmooth(230);
  moveSmooth(180);
  moveSmooth(20);
}