#include <Servo.h>

// Servo driver for aperture cover shutter.
// Protocol: receive pulse width in microseconds as ASCII integer + newline.
// Responds with "OK\n" on success, "ERR\n" on bad input.
// Servo signal on pin 9.

#define SERVO_PIN    11
#define PW_MIN_US  500
#define PW_MAX_US  2500

Servo servo;
int current_pw = 0;   // 0 = detached (idle)

void setup() {
    Serial.begin(115200);
    while (!Serial);  // Leonardo: wait for USB serial to enumerate
    Serial.println("READY");
}

void loop() {
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    // detach command
    if (line == "DETACH") {
        servo.detach();
        current_pw = 0;
        Serial.println("OK");
        return;
    }

    int pw = line.toInt();
    if (pw < PW_MIN_US || pw > PW_MAX_US) {
        Serial.println("ERR");
        return;
    }

    if (!servo.attached()) {
        servo.attach(SERVO_PIN, PW_MIN_US, PW_MAX_US);
    }
    servo.writeMicroseconds(pw);
    current_pw = pw;
    Serial.println("OK");
}
