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

const float initial_temp = 20;
const float desired_temp = 230;
const float roast_time = 8 * 60000;
const float temp_slope = (desired_temp - initial_temp) / roast_time;

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
int heater_state = LOW;
float target_temp = 0;

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
  Serial.print("time,");
  Serial.print("average_temp,");
  Serial.print("target_temp,");
  Serial.print("motor_speed,");
  Serial.print("heater_state");
  Serial.println();
}

void loop() {
  long diff;

  if (millis() - temperature_time > 300) {
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

  switch (current_state) {
    case 0:
      heater_state = LOW;
      motor_speed = 255;
      target_temp = 0;
      break;
    case 1:
      target_temp = initial_temp + (millis() - state_start) * temp_slope;
      //if (temp_out >= desired_temp && target_temp >= desired_temp) current_state = 0;
      //else {
        if (temp_out < target_temp) heater_state = HIGH;
        else heater_state = LOW;
        diff = (target_temp - 160) * 2;
        if (diff < 0) diff = 0;
        if (diff > 115) diff = 115;
        motor_speed = 255 - diff;
      //}
      break;
  }

  if (heater_state == LOW) {
    digitalWrite(LED_PIN, LOW);
    analogWrite(RELAY_PIN, 0);
  } else {
    digitalWrite(LED_PIN, HIGH);
    analogWrite(RELAY_PIN, 1);
  }

  analogWrite(speedpin, motor_speed);
  lastButtonState = reading;

  if (millis() - serial_time > 2000) {
    serial_time = millis();
    Serial.print(millis());
    Serial.print(",");
    Serial.print(temp_sum / temp_count);
    Serial.print(",");
    Serial.print(target_temp);
    Serial.print(",");
    Serial.print(motor_speed);
    Serial.print(",");
    Serial.print(heater_state);
    Serial.println();
    temp_sum = 0.0;
    temp_count = 0;
  }
}

