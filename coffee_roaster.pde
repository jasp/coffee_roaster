#include <MAX6675.h>

const unsigned int SCK_PIN = 2;
const unsigned int CS_PIN = 3;
const unsigned int SO_PIN = 4;
const unsigned int BUTTON_PIN = 6;
const unsigned int LED_PIN = 13;
const unsigned int RELAY_PIN = 10;
const unsigned int TEMP_SENSOR_PIN = 1;
const unsigned int pinI1=8;//define I1 port
const unsigned int pinI2=9;//define I2 port
const unsigned int speedpin=11;//define EA(PWM speed regulation)port

int current_state = 0;
int buttonState;
int lastButtonState = LOW;

long lastDebounceTime = 0;
long debounceDelay = 50;

long state_start;
long temperature_time;
long serial_time;

const int units = 1;        // Units to readout temp (0 = ÀöF, 1 = ÀöC)
float error = 0.0;    // Temperature compensation error
float temp_out = 0.0; // Temperature output varible
int temp_count = 0;
float temp_sum = 0.0;

int motor_speed = 0;

MAX6675 temp0(CS_PIN,SO_PIN,SCK_PIN,units,error);

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(pinI1,OUTPUT);//define this port as output
  pinMode(pinI2,OUTPUT);
  pinMode(speedpin,OUTPUT);
  digitalWrite(pinI1,LOW);// DC motor rotates clockwise
  digitalWrite(pinI2,HIGH);
  pinMode(SCK_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(SO_PIN, INPUT);
  digitalWrite(CS_PIN, HIGH);
  Serial.begin(9600);
}

void loop() {
  long diff;

  if (millis() - temperature_time > 500) {
    temperature_time = millis();
    temp_out = temp0.read_temp();
    temp_count++;
    temp_sum += temp_out;
  }

  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (buttonState == HIGH && reading == LOW) {
      switch (current_state) {
        case 0:
          current_state = 1;
          state_start = millis();
          break;
        default:
          current_state = 0;
          break;
      }
    }
    buttonState = reading;
  }

  if (temp_out < 130.0) {
    state_start = millis();
  }

  switch (current_state) {
    case 0:
      digitalWrite(LED_PIN, LOW);
      analogWrite(RELAY_PIN, 0);
      motor_speed = 255;
      break;
    case 1:
      digitalWrite(LED_PIN, HIGH);
      analogWrite(RELAY_PIN, 1);
      diff = (millis() - state_start) / 500;
      motor_speed = 255 - ((diff > 115) ? 115 : diff);
      break;
  }
  analogWrite(speedpin, motor_speed);
  lastButtonState = reading;

  if (millis() - serial_time > 2000) {
    serial_time = millis();
    Serial.print(millis());
    Serial.print(",");
    Serial.print(temp_sum / temp_count);
    Serial.print(",");
    Serial.print(motor_speed);
    Serial.println();
    temp_sum = 0.0;
    temp_count = 0;
  }
}

