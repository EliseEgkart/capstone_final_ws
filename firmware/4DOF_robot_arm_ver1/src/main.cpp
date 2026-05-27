#include <Arduino.h>

String rx_line = "";

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("OK,0,0,0,0");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      rx_line.trim();

      if (rx_line.startsWith("ARM,")) {
        float j1, j2, j3, j4;
        int parsed = sscanf(rx_line.c_str(), "ARM,%f,%f,%f,%f", &j1, &j2, &j3, &j4);

        if (parsed == 4) {


          Serial.print("OK,");
          Serial.print(j1); Serial.print(",");
          Serial.print(j2); Serial.print(",");
          Serial.print(j3); Serial.print(",");
          Serial.println(j4);
        } else {
          Serial.println("ERR,PARSE");
        }
      }
      else if (rx_line == "GET_STATE") {
        // 현재 각도값 반환 예시
        Serial.println("STATE,90.0,45.0,120.0,30.0");
      }
      else {
        Serial.println("ERR,UNKNOWN_CMD");
      }

      rx_line = "";
    }
    else {
      rx_line += c;
    }
  }
}