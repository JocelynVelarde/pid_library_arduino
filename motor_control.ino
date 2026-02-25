#include "PID.h"
#include "Arduino.h"

// Configuración de Pines
#define PHASEA_GPIO 35  
#define PHASEB_GPIO 34  
#define POT_GPIO 32     
#define LED_GPIO 2
#define IN1_GPIO 25     
#define IN2_GPIO 26     
#define PWM_GPIO 27     

// Variables Globales
volatile int countPhaseA = 0;
float rpm = 0;
double setPointRPM = 0; 
double currentRPM = 0; // Para el PID usamos double
int pwmValue = 0;      

unsigned long previousMillis = 0;
const unsigned long sampleTime = 50; // 50ms es ideal para control de motores

// Configuración PID
// Nota: Ajusta Kp, Ki, Kd según el comportamiento de tu motor
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
  
  // ESP32 PWM: 5kHz, 10 bits (0-1023)
  ledcAttach(PWM_GPIO, 5000, 10); 
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. Lectura del Setpoint (Potenciómetro)
  int potValue = analogRead(POT_GPIO);
  // Mapeamos de -500 a 500 RPM
  setPointRPM = map(potValue, 0, 4095, -500, 500);
  
  // Zona muerta para evitar que el motor zumbe en el centro
  if (abs(setPointRPM) < 25) setPointRPM = 0; 

  // 2. Bloque de Control y Medición (Ejecución cada 50ms)
  if (currentMillis - previousMillis >= sampleTime) {
    float dt = (currentMillis - previousMillis) / 1000.0; // Delta tiempo en segundos
    previousMillis = currentMillis;

    // Sección Crítica: Captura de pulsos
    noInterrupts();
    int pulseCount = countPhaseA;
    countPhaseA = 0;
    interrupts();
    
    // Calcular RPM actuales: (pulsos / 231) / (dt_segundos / 60)
    currentRPM = ((double)pulseCount / 231.0) * (60.0 / dt);
    
    // 3. Ejecución del PID
    // Usamos abs() porque el PID calcula "esfuerzo", la dirección la damos nosotros
    double pidResult = myPID.Calculate(abs(setPointRPM), abs(currentRPM));
    pwmValue = (int)pidResult;

    // 4. Manejo de Dirección y Salida
    if (setPointRPM > 0) {
      digitalWrite(IN1_GPIO, LOW);
      digitalWrite(IN2_GPIO, HIGH);
    } else if (setPointRPM < 0) {
      digitalWrite(IN1_GPIO, HIGH);
      digitalWrite(IN2_GPIO, LOW);
    } else {
      // Freno activo si el setpoint es 0
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