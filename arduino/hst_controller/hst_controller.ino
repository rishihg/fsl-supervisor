/*
 * hst_controller.ino — HST auxiliary controller (Arduino Leonardo)
 *
 * Controls:
 *   - S1FC635 laser interlock (relay on pin 7)
 *   - Primary mirror shutter  (relay on pin 8)
 *
 * Serial protocol (115200 baud, newline terminated):
 *   L1   laser on  (close interlock)
 *   L0   laser off (open interlock)
 *   S1   shutter open
 *   S0   shutter close
 *   ?    query status → "L<0|1> S<0|1>\n"
 *
 * Relay modules: LOW = energised on most 5V relay boards.
 * Change RELAY_ON/RELAY_OFF below if yours is active-high.
 */

#define PIN_LASER   7
#define PIN_SHUTTER 8
#define PIN_ADR     9

#define RELAY_ON  LOW
#define RELAY_OFF HIGH

bool laser_state   = false;
bool shutter_state = false;
bool adr_state     = false;

void setup() {
  pinMode(PIN_LASER,   OUTPUT);
  pinMode(PIN_SHUTTER, OUTPUT);
  pinMode(PIN_ADR,     OUTPUT);

  // Safe state on boot: laser off, shutter closed, ADR disabled
  digitalWrite(PIN_LASER,   RELAY_OFF);
  digitalWrite(PIN_SHUTTER, RELAY_OFF);
  digitalWrite(PIN_ADR,     RELAY_OFF);

  Serial.begin(115200);
  while (!Serial);   // Leonardo: wait for USB serial
  Serial.println("HST controller ready");
}

void loop() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd == "L1") {
    laser_state = true;
    digitalWrite(PIN_LASER, RELAY_ON);
    Serial.println("OK laser on");

  } else if (cmd == "L0") {
    laser_state = false;
    digitalWrite(PIN_LASER, RELAY_OFF);
    Serial.println("OK laser off");

  } else if (cmd == "S1") {
    shutter_state = true;
    digitalWrite(PIN_SHUTTER, RELAY_ON);
    Serial.println("OK shutter open");

  } else if (cmd == "S0") {
    shutter_state = false;
    digitalWrite(PIN_SHUTTER, RELAY_OFF);
    Serial.println("OK shutter closed");

  } else if (cmd == "A1") {
    adr_state = true;
    digitalWrite(PIN_ADR, RELAY_ON);
    Serial.println("OK ADR enabled");

  } else if (cmd == "A0") {
    adr_state = false;
    digitalWrite(PIN_ADR, RELAY_OFF);
    Serial.println("OK ADR disabled");

  } else if (cmd == "?") {
    Serial.print("L");
    Serial.print(laser_state ? "1" : "0");
    Serial.print(" S");
    Serial.print(shutter_state ? "1" : "0");
    Serial.print(" A");
    Serial.println(adr_state ? "1" : "0");

  } else if (cmd.length() > 0) {
    Serial.print("ERR unknown command: ");
    Serial.println(cmd);
  }
}
