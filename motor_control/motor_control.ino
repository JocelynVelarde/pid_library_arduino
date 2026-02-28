#include "PID.h"
#include "Arduino.h"

#define PHASEA_GPIO 35  
#define PHASEB_GPIO 34  
#define POT_GPIO 32     
#define LED_GPIO 2
#define IN1_GPIO 25     
#define IN2_GPIO 27     
#define PWM_GPIO 26     
#define STOP_BUTTON_GPIO 33 

volatile int countPhaseA = 0;
double setPointRPM = 0; 
double currentRPM = 0; 
int pwmValue = 0;      
const double MAX_RPM = 463; 

unsigned long previousMillis = 0;
const unsigned long sampleTime = 15; 
unsigned long previousSerialMillis = 0;
const unsigned long serialInterval = 100;

// PID Tuning
double Kp = 1.5; 
double Ki = 60.0;
double Kd = 0.0; 
double max_error = 1000; 

PID myPID(Kp, Ki, Kd, max_error);

void IRAM_ATTR isr() {
  countPhaseA++;
}

void setup() {
  Serial.begin(115200);
  
  pinMode(IN1_GPIO, OUTPUT);
  pinMode(IN2_GPIO, OUTPUT);
  pinMode(LED_GPIO, OUTPUT);
  pinMode(PWM_GPIO, OUTPUT);
  pinMode(PHASEA_GPIO, INPUT_PULLUP);
  pinMode(PHASEB_GPIO, INPUT_PULLUP);
  
  pinMode(STOP_BUTTON_GPIO, INPUT_PULLUP);
  
  attachInterrupt(PHASEA_GPIO, isr, FALLING);
  ledcAttach(PWM_GPIO, 5000, 10); 
}

void loop() {
  unsigned long currentMillis = millis();
  
  bool isStopPressed = (digitalRead(STOP_BUTTON_GPIO) == HIGH);

  int potValue = analogRead(POT_GPIO);
  setPointRPM = map(potValue, 0, 4095, 0, MAX_RPM);
  if (abs(setPointRPM) < 25) setPointRPM = 0; 

  if (currentMillis - previousMillis >= sampleTime) {
    float dt = (currentMillis - previousMillis) / 1000.0; 
    
    noInterrupts();
    int pulseCount = countPhaseA;
    countPhaseA = 0;
    previousMillis = currentMillis; 
    interrupts();
    
    currentRPM = (pulseCount * 60.0) / (115 * dt);

    if (isStopPressed) {
      pwmValue = 0;
    } else {
      double pidResult = myPID.Calculate(abs(setPointRPM), abs(currentRPM), dt);
      pwmValue = constrain((int)pidResult, 0, 1023);
    }

    if (setPointRPM > 0 && !isStopPressed) {
      digitalWrite(IN1_GPIO, LOW);
      digitalWrite(IN2_GPIO, HIGH);
    } else {
      digitalWrite(IN1_GPIO, HIGH);
      digitalWrite(IN2_GPIO, HIGH);
      pwmValue = 0; 
    }

    ledcWrite(PWM_GPIO, pwmValue); 

    double currentRpmPercent = (currentRPM / MAX_RPM) * 100.0;
    double setpointRpmPercent = (setPointRPM / MAX_RPM) * 100.0;

    if (currentMillis - previousSerialMillis >= serialInterval) {
        previousSerialMillis = currentMillis;
        
        Serial.print("{\"target\":"); 
        Serial.print(setpointRpmPercent);
        Serial.print(", \"real\":"); 
        Serial.print(currentRpmPercent);
        Serial.print(", \"pwm\":"); 
        Serial.print(pwmValue);
        Serial.print(", \"stop\":"); 
        Serial.print(isStopPressed ? 1 : 0);
        Serial.println("}");
    }
    
    digitalWrite(LED_GPIO, !digitalRead(LED_GPIO));
  }
}