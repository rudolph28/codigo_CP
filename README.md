# Control Proporcional (P) de Velocidad para Motor CD

Implementación integral de un sistema de control en lazo cerrado para la velocidad angular de un motor de corriente directa con imanes permanentes[cite: 1, 2]. El proyecto abarca desde el modelado analítico basado en leyes físicas fundamentales (Kirchhoff y Newton)[cite: 1, 2], su discretización temporal (Euler hacia adelante), la simulación computacional y el despliegue en hardware embebido con Arduino y una interfaz gráfica (GUI) en Python.

---

## 📋 Tabla de Contenidos
- [Características del Proyecto](#-características-del-proyecto)
- [Especificaciones del Sistema](#-especificaciones-del-sistema)
- [Modelado Matemático](#-modelado-matemático)
- [Estructura del Repositorio](#-estructura-del-repositorio)
- [Requisitos e Instalación](#-requisitos-e-instalación)
- [Diagrama de Conexiones (Hardware)](#-diagrama-de-conexiones-hardware)
- [Guía de Uso](#-guía-de-uso)
  - [Carga del Firmware (Arduino)](#1-carga-del-firmware-arduino)
  - [Ejecución de la Interfaz (Python)](#2-ejecución-de-la-interfaz-python)
- [Análisis Teórico del Régimen Permanente](#-análisis-teórico-del-régimen-permanente)

---

## 🚀 Características del Proyecto

* **Firmware determinista:** Ejecución a $50\text{ Hz}$ ($\Delta t = 20\text{ ms}$) mediante interrupciones de hardware e intervalos de muestreo precisos con `millis()`.
* **Lectura atómica:** Adquisición de pulsos del encoder óptico mediante secciones críticas (`noInterrupts()` / `interrupts()`) para evitar condiciones de carrera.
* **Interfaz Gráfica en Python:** Visualizador en tiempo real desarrollado en Tkinter e incrustado con Matplotlib para monitorizar Setpoint, RPM actuales y esfuerzo de control (PWM).
* **Conexión Serial Dinámica:** Capacidad para conectar y desconectar el Arduino en caliente desde la ventana sin reiniciar la aplicación.
* **Simulador Teórico Integrado:** Posibilidad de ejecutar la simulación por diferencias finitas de la planta física sin depender del hardware conectado.

---

## ⚙️ Especificaciones del Sistema

* **Alimentación nominal:** $12\text{ V}$[cite: 1, 2].
* **Microcontrolador:** Arduino UNO / Nano.
* **Driver de potencia:** Puente H (L298N, L293D o etapa MOSFET).
* **Rango del actuador digital:** Modulación por Ancho de Pulsos (PWM) de 8 bits ($u \in [0, 255]$ cuentas)[cite: 2].
* **Sensor de realimentación:** Encoder incremental de $N = 20\text{ ranuras/pulsos por revolución}$[cite: 2].
* **Velocidad máxima medida (a PWM = 255):** $y_{\max} = 3000\text{ RPM}$[cite: 2].
* **Constante de tiempo mecánica:** $\tau = 0.4\text{ s}$ ($t_{63.2\%}$)[cite: 2].
* **Frecuencia de muestreo:** $f_s = 50\text{ Hz}$ ($\Delta t = 0.02\text{ s}$)[cite: 2].

---

## 📐 Modelado Matemático

### 1. Función de Transferencia de la Planta
Despreciando la inductancia de armadura ($L \approx 0$)[cite: 1, 2], el comportamiento dinámico se simplifica a un sistema de primer orden[cite: 2]:

$$G(s) = \frac{\Omega(s)}{U(s)} = \frac{K}{\tau s + 1}$$

Donde:
* **Ganancia estática:** $K = \frac{\Delta y}{\Delta u} = \frac{3000\text{ RPM}}{255\text{ PWM}} \approx 11.7647\text{ }\frac{\text{RPM}}{\text{cuenta}}$[cite: 2]
* **Constante de tiempo:** $\tau = 0.4\text{ s}$[cite: 2]

### 2. Discretización (Euler hacia adelante)
Para evaluar la ecuación en diferencias en microcontroladores y simulación[cite: 2]:

$$y[k] = y[k-1] + \frac{\Delta t}{\tau} \Big( K \cdot u[k-1] - y[k-1] \Big)$$

---

## 📁 Estructura del Repositorio

```text
├── arduino/
│   └── control_motor_p.ino       # Firmware determinista en C++ para Arduino UNO/Nano
├── python/
│   └── app_control_motor.py      # Interfaz GUI (Tkinter + Matplotlib + PySerial)
├── docs/
│   └── guia_control_motor.tex    # Documentación pedagógica y deducciones formales en LaTeX
├── LICENSE                       # Licencia del proyecto (ej. MIT)
└── README.md                     # Documentación principal
