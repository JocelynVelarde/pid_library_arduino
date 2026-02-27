#include "PID.h"
#include "Arduino.h"

#define PHASEA_GPIO 35  
#define PHASEB_GPIO 34  
#define POT_GPIO 32     
#define LED_GPIO 2
#define IN1_GPIO 25     
#define IN2_GPIO 27     
#define PWM_GPIO 26     

volatile int countPhaseA = 0;

double setPointRPM = 0; 
double currentRPM = 0; 
int pwmValue = 0;      
const double MAX_RPM = 463; 
const int two_hundred = 200;

unsigned long previousMillis = 0;
const unsigned long sampleTime = 15; 

// PID Tuning Parameters
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
  
  attachInterrupt(PHASEA_GPIO, isr, FALLING);
  
  ledcAttach(PWM_GPIO, 5000, 10); 
}

void loop() {
  unsigned long currentMillis = millis();

  // Read potentiometer and map to RPM
  int potValue = analogRead(POT_GPIO);
  setPointRPM = map(potValue, 0, 4095, 0, MAX_RPM);
  
  // Deadzone to prevent motor buzzing at the center
  if (abs(setPointRPM) < 25) setPointRPM = 0; 

  if (currentMillis - previousMillis >= sampleTime) {
    // 1. Calculate precise dt in seconds
    float dt = (currentMillis - previousMillis) / 1000.0; 
    
    // 2. Safely grab the pulse count
    noInterrupts();
    int pulseCount = countPhaseA;
    countPhaseA = 0;
    // Update previousMillis right after resetting the counter for accurate timing
    previousMillis = currentMillis; 
    interrupts();
    
    currentRPM = (pulseCount * 60) / (102 * dt);
    
    double pidResult = myPID.Calculate(abs(setPointRPM), abs(currentRPM), dt);
    pwmValue = (int)pidResult;

    // 5. Motor Direction and PWM Application
    if (setPointRPM > 0) {
      digitalWrite(IN1_GPIO, LOW);
      digitalWrite(IN2_GPIO, HIGH);
    } else {
      digitalWrite(IN1_GPIO, HIGH);
      digitalWrite(IN2_GPIO, HIGH);
      pwmValue = 0; // Force PWM to 0 if we want to stop
    }

    pwmValue = constrain(pwmValue, 0, 1023);
    ledcWrite(PWM_GPIO, pwmValue);

    // 6. Calculate Percentages (0 to 100%)
    double currentRpmPercent = (currentRPM / MAX_RPM) * 100.0;
    double setpointRpmPercent = (setPointRPM / MAX_RPM) * 100.0;

    // Constrain percentages for a clean graph just in case of overshoot
    currentRpmPercent = constrain(currentRpmPercent, 0, 100);
    setpointRpmPercent = constrain(setpointRpmPercent, 0, 100);

    // 7. Output for Serial Plotter
    Serial.print("Target_%:"); Serial.print(setpointRpmPercent);
    Serial.print(",");
    Serial.print("Real_%:"); Serial.println(currentRpmPercent);
    Serial.println();
    Serial.print("twohundred%:"); Serial.println(200);
    Serial.print("minustwo%:"); Serial.println(-200);
    // Toggle diagnostic LED
    digitalWrite(LED_GPIO, !digitalRead(LED_GPIO));
  }
}