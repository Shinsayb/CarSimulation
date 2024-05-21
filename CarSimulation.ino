#include <IRremote.hpp>
#include <MFRC522v2.h> //RFID
#include <MFRC522DriverSPI.h>
#include <MFRC522DriverPinSimple.h>
#include <MFRC522Debug.h>
#include <Stepper.h>
#include <LiquidCrystal_I2C.h>
#include "HCSR04.h" //Ultrasonic
#include "DHT.h" //Temp, Humid

enum PROJECT_CAR {
  SW1 = 2U,
  SW2 = 3U,
  SERVO = 4U,
  RELAY = 22U,
  IR,
  TRIGGER, //Ultrasonic
  ECHO, //Ultrasonic
  BUZZ,
  DHT = A0,
  JOYSTICKX= A1,
  JOYSTICKY = A2
};

const uint16_t STEP_PER_REVOLUTION {2048};
const String MASTER_CARD_UID {String("F48DFF2A")};
static uint8_t cnt {0U};
static volatile uint8_t trg {0U};
static uint16_t previousMillis {0UL};
static uint16_t x_value {0U};
static uint16_t y_value {0U};
static uint16_t joyX_mapped_value {0U};
static uint16_t joyY_mapped_value {0U};
static bool trgReverse {false};
static bool rfidON {false};
static bool sw1_state {false};
static bool sw2_state {false};
static float temperature {0.0F};
static float humidity {0.0F};
static float distance {0.0F};
static float distance_warn {0.0F};
static float distance_mapped_value {0.0F};
class MFRC522DriverPinSimple sda_pin(53);
class MFRC522DriverSPI driver {sda_pin};
class MFRC522 mfrc522 {driver};
class decode_results result;
class Stepper stepping(STEP_PER_REVOLUTION, 11, 9, 10, 8);
class IRrecv irrecv(IR);
class LiquidCrystal_I2C lcd(0x27, 16, 2);
class DHT dht(DHT, 11);
class UltraSonicDistanceSensor ultra_sonic(TRIGGER, ECHO);

void setup() {
  Serial.begin(115200UL);
  Serial1.begin(9600UL);
  mfrc522.PCD_Init();
  stepping.setSpeed(14);
  dht.begin();
  irrecv.begin(IR, LED_BUILTIN);
  lcd.init();
  lcd.backlight();
  lcd.home();
  pinMode(BUZZ, OUTPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(SERVO, OUTPUT);
  pinMode(JOYSTICKX, INPUT);
  pinMode(SW1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SW1), swInterrupt, FALLING);
  Serial1.println("Bluetooth Ready");
}

void loop() {  
  servoJoystick();
  rfidCheck();
  irCheck();
  switch (trg) {
    case 0:
      gearP();
      break;
    case 1:
      gearR();
      break;
    case 2:
      gearN();
      break;
    case 3:
      gearD();
      break;
    default:
      break;
  }
  lcdDisplay();
}

void gearP() {
  Serial.println(F("gear P"));
  Serial1.println(F("gear P"));
  servoJoystick();
  delay(100UL);
}
void gearR() {
  Serial.println(F("gear R"));
  Serial.println(F("Counter Clockwise"));
  Serial1.println(F("gear R"));
  Serial1.println(F("Counter Clockwise"));
  servoJoystick();
  y_value = analogRead(JOYSTICKY);
  joyY_mapped_value = map(y_value, 0, 1023, 1, 14);
  Serial.println(joyY_mapped_value);
  Serial1.println(joyY_mapped_value);
  stepping.setSpeed(joyY_mapped_value);
  stepping.step(STEP_PER_REVOLUTION / 32);
  temperature = dht.readTemperature();
  Serial.println(temperature);
  Serial1.println(temperature);
  distance = ultra_sonic.measureDistanceCm(temperature);
  Serial.println(distance);
  Serial1.println(distance);
  distance_warn = distance;
  if(distance_warn > 70.0F) {
    distance_warn = 70.0F;
  }
  distance_mapped_value = map(distance_warn, 0, 70, 0, 700);
  if(400.0F < distance or distance < 0.0F) {
    Serial.println(F("Out of ranges"));
    Serial1.println(F("Out of ranges"));
  }
  else {
    Serial.print(F("Distance : "));
    Serial.println(distance + String(" CM"));
    Serial.println(distance_mapped_value);
    Serial1.print(F("Distance : "));
    Serial1.println(distance + String(" CM"));
  }
  if(700 > distance_mapped_value and distance_mapped_value > 100) {
    tone(BUZZ, 700);
    delay(100UL);
    noTone(BUZZ);
  }
  else if(distance_mapped_value <= 100) {
    tone(BUZZ, 700);
    Serial.println(F("WARNING! TOO CLOSE!"));
    Serial1.println(F("WARNING! TOO CLOSE!"));
    while (distance_mapped_value <= 100) {
      distance = ultra_sonic.measureDistanceCm(temperature);
      distance_warn = distance;
      if(distance_warn > 70.0F) {
      distance_warn = 70.0F;
      }
      distance_mapped_value = map(distance_warn, 0, 70, 0, 700);
      delay(50UL);
    }
    noTone(BUZZ);
  }
  delay(distance_mapped_value);
}
void gearN() {
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  Serial.println(F("gear N"));
  Serial1.println(F("gear N"));
  servoJoystick();
  delay(100UL);
}
void gearD() {
  Serial.println(F("gear D"));
  Serial.println(F("Clockwise"));
  Serial1.println(F("gear D"));
  Serial1.println(F("Clockwise"));
  servoJoystick();
  y_value = analogRead(JOYSTICKY);
  joyY_mapped_value = map(y_value, 0, 1023, 1, 14);
  Serial.println(joyY_mapped_value);
  Serial1.println(joyY_mapped_value);
  stepping.setSpeed(joyY_mapped_value);
  stepping.step(STEP_PER_REVOLUTION / 32);
}
void irCheck() {
  if(irrecv.decode()) {
    uint8_t data_value = irrecv.decodedIRData.command;    
    Serial.println(data_value, HEX);
    switch (data_value) {
      case 0x16:
        digitalWrite(RELAY, HIGH);
        stepping.step(STEP_PER_REVOLUTION);
        delay(1000UL);
        break;
      case 0x0c:
        digitalWrite(RELAY, LOW);
        stepping.step(-STEP_PER_REVOLUTION);
        delay(1000UL);
        break;
      default:
        break;
    }
    irrecv.resume();
    irrecv.start(10);
  }
}
void rfidCheck() {
  if(!mfrc522.PICC_IsNewCardPresent())
    return;
  if(!mfrc522.PICC_ReadCardSerial())
    return;
  String tagID = "";
  for(uint8_t i {0U}; i < 4; ++i) {
    tagID += String(mfrc522.uid.uidByte[i], HEX);
  }
  tagID.toUpperCase();
  mfrc522.PICC_HaltA();
  if(tagID == MASTER_CARD_UID && !digitalRead(RELAY)) {
    Serial.println(F("Correct Card. Turn on the car."));
    Serial1.println(F("Correct Card. Turn on the car."));
    cnt = 0;
    digitalWrite(RELAY, HIGH);
  }
  else if(tagID == MASTER_CARD_UID && digitalRead(RELAY) && trg == 0) {
    Serial.println(F("Correct Card. Turn off the car."));
    Serial1.println(F("Correct Card. Turn off the car."));
    cnt = 0;
    digitalWrite(RELAY, LOW);
  }
  else if(tagID != MASTER_CARD_UID) {
    if(cnt >= 5) {
      delay(50UL);
      Serial.println(F("The Car is Locked. Contact Service Team 000-0000-0000"));
      Serial1.println(F("The Car is Locked. Contact Service Team 000-0000-0000"));
    }
    else {
      ++cnt;
      Serial.print(F("Incorrect Card. You can try 5times ("));
      Serial.print(cnt);
      Serial.println(F("/5)"));
      Serial1.print(F("Incorrect Card. You can try 5times ("));
      Serial1.print(cnt);
      Serial1.println(F("/5)"));
    }
  }
}
void servoJoystick() {
  x_value = analogRead(JOYSTICKX);
  if(x_value > 1023) {
    x_value = 1023;
  }
  joyX_mapped_value = map(x_value, 0, 1023, 0, 255);
  analogWrite(SERVO, joyX_mapped_value);
  Serial.println(joyX_mapped_value);
  Serial1.println(joyX_mapped_value);
  delay(30UL);
}
void swInterrupt() {
  sw1_state = digitalRead(SW1);
  if (!sw1_state) {
    delay(50UL);
    sw1_state = digitalRead(SW1);
    if (!sw1_state) {
      if(!trgReverse) {
        ++trg;
        if(trg < 4) {
          Serial.print(F("TRIGGER : "));
          Serial.println(trg);
          Serial1.print(F("TRIGGER : "));
          Serial1.println(trg);
        }
        else if(trg == 4) {
          trgReverse = true;
          trg = 2;
          Serial.print(F("TRIGGER : "));
          Serial.println(trg);
          Serial1.print(F("TRIGGER : "));
          Serial1.println(trg);
        }
      }
      else if(trgReverse) {
        --trg;
        if(trg > 0) {
          Serial.print(F("TRIGGER : "));
          Serial.println(trg);
          Serial1.print(F("TRIGGER : "));
          Serial1.println(trg);
        }
        else if(trg == 0) {
          trgReverse = false;
          Serial.print(F("TRIGGER : "));
          Serial.println(trg);
          Serial1.print(F("TRIGGER : "));
          Serial1.println(trg);
        }
      }
    }
    while (!digitalRead(SW1)) {
    }
    delay(50UL);
  }
}
void lcdDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  switch (trg) {
    case 0:
      lcd.print(F("P"));
      lcd.setCursor(0, 1);
      lcd.print(F("ENGINE : "));
      if(digitalRead(RELAY)) {
        lcd.print(F("ON"));
      }
      else if(!digitalRead(RELAY)) {
        lcd.print(F("OFF"));
      }
      break;
    case 1:
      lcd.print(F("R"));
      lcd.setCursor(0, 1);
      lcd.print(F("Distance : "));
      lcd.print(distance);
      break;
    case 2:
      lcd.print(F("N"));
      lcd.setCursor(0, 1);
      lcd.print(F("TEMP:"));
      lcd.print(round(temperature));
      lcd.print(" HUMD:");
      lcd.print(round(humidity));
      break;
    case 3:
      lcd.print(F("D"));
      lcd.setCursor(0, 1);
      lcd.print(F("SPEED : "));
      lcd.print(joyY_mapped_value);
      break;
    default:
      break;
  }
}