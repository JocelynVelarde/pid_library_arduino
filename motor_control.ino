#include "PID.h"
#include "Arduino.h"

#define PHASEA_GPIO 35  
#define PHASEB_GPIO 34  
#define POT_GPIO 32     
#define LED_GPIO 2
#define IN1_GPIO 25     
#define IN2_GPIO 26     
#define PWM_GPIO 27     

volatile int countPhaseA = 0;
float rpm = 0;
double setPointRPM = 0; 
double currentRPM = 0; 
int pwmValue = 0;      

unsigned long previousMillis = 0;
const unsigned long sampleTime = 50; 

double Kp = 1.2; 
double Ki = 0.5; 
double Kd = 0.1; 
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
  
  attachInterrupt(PHASEA_GPIO, isr, FALLING);
  
  ledcAttach(PWM_GPIO, 5000, 10); 
}

void loop() {
  unsigned long currentMillis = millis();

  int potValue = analogRead(POT_GPIO);
  setPointRPM = map(potValue, 0, 4095, -500, 500);
  
  // Zona muerta para evitar que el motor zumbe en el centro
  if (abs(setPointRPM) < 25) setPointRPM = 0; 

  if (currentMillis - previousMillis >= sampleTime) {
    float dt = (currentMillis - previousMillis) / 1000.0; 
    previousMillis = currentMillis;

    noInterrupts();
    int pulseCount = countPhaseA;
    countPhaseA = 0;
    interrupts();
    
    currentRPM = ((double)pulseCount / 231.0) * (60.0 / dt);
    
    double pidResult = myPID.Calculate(abs(setPointRPM), abs(currentRPM));
    pwmValue = (int)pidResult;

    if (setPointRPM > 0) {
      digitalWrite(IN1_GPIO, LOW);
      digitalWrite(IN2_GPIO, HIGH);
    } else if (setPointRPM < 0) {
      digitalWrite(IN1_GPIO, HIGH);
      digitalWrite(IN2_GPIO, LOW);
    } else {
      digitalWrite(IN1_GPIO, HIGH);
      digitalWrite(IN2_GPIO, HIGH);
      pwmValue = 0;
    }

    pwmValue = constrain(pwmValue, 0, 1023);
    ledcWrite(PWM_GPIO, pwmValue);

    Serial.print("Setpoint:"); Serial.print(setPointRPM);
    Serial.print(",");
    Serial.print("Real:"); Serial.println(currentRPM);
    
    digitalWrite(LED_GPIO, !digitalRead(LED_GPIO));
  }
}