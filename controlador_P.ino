// ============================================================================
// CONTROLADOR PROPORCIONAL (P) PARA MOTOR CD - ARDUINO UNO / NANO
// ============================================================================

// Asignación de pines
const uint8_t PIN_ENCODER = 2; // Pin de interrupción externa INT0
const uint8_t PIN_PWM     = 9; // Salida PWM hacia el driver de potencia

// Parámetros del controlador
const float Kp           = 0.8;
const float setpoint_rpm = 1800.0;

// Constantes de tiempo y sensor
const uint16_t PULSOS_POR_REV = 20; // Ranuras del encoder
const unsigned long DT_MS     = 20; // Período de muestreo: 20 ms (50 Hz)

// Variables de estado
volatile unsigned long contador_pulsos = 0;
unsigned long t_previo = 0;

// Rutina de Servicio de Interrupción (ISR estándar para AVR/Arduino UNO)
void ISR_contarPulsos() {
  contador_pulsos++;
}

void setup() {
  pinMode(PIN_ENCODER, INPUT_PULLUP);
  pinMode(PIN_PWM, OUTPUT);

  // Adjunta interrupción en el pin digital 2
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER), ISR_contarPulsos, RISING);

  Serial.begin(115200);
  t_previo = millis();
}

void loop() {
  unsigned long t_actual = millis();

  // Bucle temporal periódico determinista cada 20 ms
  if (t_actual - t_previo >= DT_MS) {
    t_previo = t_actual;

    // 1. Lectura atómica de pulsos acumulados
    noInterrupts();
    unsigned long pulsos = contador_pulsos;
    contador_pulsos = 0;
    interrupts();

    // 2. Conversión a RPM: (pulsos / 20) * (1000 / 20) * 60 = pulsos * 150
    float rpm_actual = ((float)pulsos / PULSOS_POR_REV) * (1000.0 / DT_MS) * 60.0;

    // 3. Cálculo del error instantáneo
    float error = setpoint_rpm - rpm_actual;

    // 4. Algoritmo Proporcional: u = Kp * e
    float u = Kp * error;

    // 5. Saturación física del actuador (PWM de 8 bits: 0 a 255)
    int salida_pwm = constrain((int)u, 0, 255);
    analogWrite(PIN_PWM, salida_pwm);

    // 6. Monitoreo serial (compatible con la interfaz de Python)
    Serial.print("Setpoint:");

    Serial.print(setpoint_rpm);
    Serial.print(",");
    Serial.print("RPM:");
    Serial.print(rpm_actual);
    Serial.print(",");
    Serial.print("PWM:");
    Serial.println(salida_pwm);
  }
}