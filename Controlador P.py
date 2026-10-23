import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import collections
import threading
import time

try:
    import serial
    import serial.tools.list_ports
    SERIAL_DISPONIBLE = True
except ImportError:
    SERIAL_DISPONIBLE = False

class ControlMotorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Monitor y Control de Motor CD - Arduino Serial")
        self.root.geometry("1150x720")
        self.root.configure(bg="#f0f2f5")

        # --- Parámetros del Modelo de la Planta ---
        self.K = 11.764705       # Ganancia estática (RPM / cuenta PWM)
        self.tau = 0.4           # Constante de tiempo (s)
        self.dt = 0.02           # 20 ms (50 Hz)
        self.t_final = 3.0

        # --- Variables de Comunicación Serial ---
        self.ser = None
        self.conectado = False
        self.hilo_lectura = None
        self.t0 = None

        # Historial de datos para el buffer gráfico (últimas 300 muestras)
        self.max_muestras = 300
        self.hist_t = collections.deque(maxlen=self.max_muestras)
        self.hist_sp = collections.deque(maxlen=self.max_muestras)
        self.hist_y = collections.deque(maxlen=self.max_muestras)
        self.hist_u = collections.deque(maxlen=self.max_muestras)

        # Construcción visual
        self._construir_barra_control()
        self._construir_lienzo_grafico()

        # Asegurar cierre limpio al presionar la 'X' de la ventana
        self.root.protocol("WM_DELETE_WINDOW", self._cerrar_aplicacion)

    def _construir_barra_control(self):
        # Panel superior de controles
        panel = tk.LabelFrame(
            self.root, 
            text=" Configuración y Conexión de Hardware ", 
            font=("Segoe UI", 10, "bold"), 
            bg="white", 
            padx=12, 
            pady=10
        )
        panel.pack(side=tk.TOP, fill=tk.X, padx=12, pady=8)

        # 1. Parámetros de prueba
        tk.Label(panel, text="Setpoint (RPM):", bg="white", font=("Segoe UI", 9)).grid(row=0, column=0, padx=4, sticky="e")
        self.txt_sp = ttk.Entry(panel, width=7)
        self.txt_sp.insert(0, "1800")
        self.txt_sp.grid(row=0, column=1, padx=4)

        tk.Label(panel, text="Kp:", bg="white", font=("Segoe UI", 9)).grid(row=0, column=2, padx=4, sticky="e")
        self.txt_kp = ttk.Entry(panel, width=6)
        self.txt_kp.insert(0, "0.8")
        self.txt_kp.grid(row=0, column=3, padx=4)

        self.btn_simular = tk.Button(
            panel, 
            text="Simular Planta", 
            bg="#e7f1ff", 
            fg="#0d6efd", 
            font=("Segoe UI", 9, "bold"), 
            command=self.ejecutar_simulacion
        )
        self.btn_simular.grid(row=0, column=4, padx=8)

        ttk.Separator(panel, orient='vertical').grid(row=0, column=5, sticky='ns', padx=10)

        # 2. Conexión Serial con Arduino
        tk.Label(panel, text="Puerto:", bg="white", font=("Segoe UI", 9)).grid(row=0, column=6, padx=4, sticky="e")
        self.combo_puertos = ttk.Combobox(panel, width=10, state="readonly")
        self.combo_puertos.grid(row=0, column=7, padx=4)

        self.btn_actualizar = ttk.Button(panel, text="↻", width=3, command=self.actualizar_lista_puertos)
        self.btn_actualizar.grid(row=0, column=8, padx=2)

        # Botón para CONECTAR / DESCONECTAR en cualquier momento
        self.btn_conectar = tk.Button(
            panel, 
            text="Conectar Arduino", 
            bg="#198754", 
            fg="white", 
            font=("Segoe UI", 9, "bold"), 
            padx=10, 
            command=self.alternar_conexion_serial
        )
        self.btn_conectar.grid(row=0, column=9, padx=8)

        # Indicador de estado del puerto
        self.lbl_estado = tk.Label(
            panel, 
            text="Desconectado", 
            fg="#6c757d", 
            bg="white", 
            font=("Segoe UI", 9, "italic")
        )
        self.lbl_estado.grid(row=0, column=10, padx=6)

        # Botón Limpiar Gráfica
        self.btn_limpiar = ttk.Button(panel, text="Limpiar Gráfica", command=self.limpiar_datos)
        self.btn_limpiar.grid(row=0, column=11, padx=6)

        # 3. Métricas actuales en tiempo real
        panel_metricas = tk.Frame(panel, bg="#f8f9fa", bd=1, relief=tk.SOLID, padx=8, pady=2)
        panel_metricas.grid(row=0, column=12, padx=12, sticky="e")

        self.lbl_rpm_act = tk.Label(panel_metricas, text="RPM: 0.0", font=("Consolas", 11, "bold"), fg="#0d6efd", bg="#f8f9fa")
        self.lbl_rpm_act.pack(anchor="w")
        self.lbl_pwm_act = tk.Label(panel_metricas, text="PWM: 0", font=("Consolas", 10), fg="#dc3545", bg="#f8f9fa")
        self.lbl_pwm_act.pack(anchor="w")

        self.actualizar_lista_puertos()

    def _construir_lienzo_grafico(self):
        # Lienzo Matplotlib dentro de Tkinter
        self.fig, (self.ax_vel, self.ax_u) = plt.subplots(
            2, 1, figsize=(10, 6), sharex=True, 
            gridspec_kw={'height_ratios': [2.5, 1]}
        )
        self.fig.patch.set_facecolor("#ffffff")

        # Gráfico superior (Velocidad angular y Setpoint)
        self.ax_vel.set_ylabel("Velocidad (RPM)", fontweight="bold")
        self.ax_vel.set_title("Respuesta Dinámica del Motor CD - Lazo Cerrado", fontsize=11, fontweight="bold")
        self.ax_vel.grid(True, linestyle=":", alpha=0.7)
        self.line_sp, = self.ax_vel.plot([], [], 'r--', linewidth=1.5, label="Setpoint r(t)")
        self.line_y, = self.ax_vel.plot([], [], 'b-', linewidth=2.0, label="Velocidad Real y(t)")
        self.ax_vel.legend(loc="lower right")

        # Gráfico inferior (Acción de control PWM)
        self.ax_u.set_xlabel("Tiempo transcurrido (s)", fontweight="bold")
        self.ax_u.set_ylabel("PWM (0 - 255)", fontweight="bold")
        self.ax_u.set_ylim(-10, 265)
        self.ax_u.grid(True, linestyle=":", alpha=0.7)
        self.line_u, = self.ax_u.plot([], [], color="#dc3545", linewidth=1.6, label="Señal de Control u(t)")
        self.ax_u.legend(loc="upper right")

        contenedor_plot = tk.Frame(self.root, bg="white")
        contenedor_plot.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=12, pady=4)

        self.canvas = FigureCanvasTkAgg(self.fig, master=contenedor_plot)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        toolbar = NavigationToolbar2Tk(self.canvas, contenedor_plot)
        toolbar.update()
        toolbar.pack(side=tk.BOTTOM, fill=tk.X)

    def actualizar_lista_puertos(self):
        if SERIAL_DISPONIBLE:
            puertos = [p.device for p in serial.tools.list_ports.comports()]
            self.combo_puertos['values'] = puertos
            if puertos:
                if self.combo_puertos.get() not in puertos:
                    self.combo_puertos.current(0)
            else:
                self.combo_puertos.set("")
        else:
            self.combo_puertos['values'] = ["Sin PySerial"]

    def alternar_conexion_serial(self):
        """Permite al usuario conectar y desconectar el Arduino en cualquier momento."""
        if not self.conectado:
            # --- Proceso de Conexión ---
            if not SERIAL_DISPONIBLE:
                messagebox.showerror("Error", "La librería 'pyserial' no está instalada.\nEjecuta: pip install pyserial")
                return

            puerto = self.combo_puertos.get()
            if not puerto:
                messagebox.showwarning("Aviso", "No se detectó ningún puerto COM. Conecta el Arduino y presiona '↻'.")
                return

            try:
                # Abrir puerto a 115200 baudios (igual que en el código de Arduino)
                self.ser = serial.Serial(puerto, 115200, timeout=1.0)
                time.sleep(1.8)  # Breve pausa para permitir el auto-reset de Arduino
                self.ser.reset_input_buffer()

                self.conectado = True
                self.t0 = time.time()
                self.limpiar_datos()

                # Actualizar botón e indicadores
                self.btn_conectar.config(text="Desconectar", bg="#dc3545")
                self.lbl_estado.config(text=f"Conectado ({puerto})", fg="#198754", font=("Segoe UI", 9, "bold"))
                self.combo_puertos.config(state="disabled")
                self.btn_actualizar.config(state="disabled")

                # Iniciar lectura en un hilo en segundo plano (no congela la ventana)
                self.hilo_lectura = threading.Thread(target=self._hilo_recepcion_serial, daemon=True)
                self.hilo_lectura.start()

            except Exception as e:
                messagebox.showerror("Fallo de Conexión", f"No se pudo abrir {puerto}:\n{str(e)}")
                self.conectado = False
        else:
            # --- Proceso de Desconexión ---
            self.desconectar_serial()

    def desconectar_serial(self):
        self.conectado = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

        self.btn_conectar.config(text="Conectar Arduino", bg="#198754")
        self.lbl_estado.config(text="Desconectado", fg="#6c757d", font=("Segoe UI", 9, "italic"))
        self.combo_puertos.config(state="readonly")
        self.btn_actualizar.config(state="normal")

    def _hilo_recepcion_serial(self):
        """Bucle en segundo plano para recibir datos de Arduino."""
        while self.conectado and self.ser and self.ser.is_open:
            try:
                linea = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not linea:
                    continue

                # Formato esperado enviado por Arduino: "Setpoint:1800.0,RPM:1627.1,PWM:138"
                if "Setpoint:" in linea and "RPM:" in linea and "PWM:" in linea:
                    partes = linea.split(',')
                    sp_val = float(partes[0].split(':')[1])
                    rpm_val = float(partes[1].split(':')[1])
                    pwm_val = float(partes[2].split(':')[1])
                    t_val = time.time() - self.t0

                    # Guardar en buffer
                    self.hist_t.append(t_val)
                    self.hist_sp.append(sp_val)
                    self.hist_y.append(rpm_val)
                    self.hist_u.append(pwm_val)

                    # Enviar actualización gráfica al hilo principal de Tkinter
                    self.root.after(0, self._refrescar_grafica_serial, sp_val, rpm_val, pwm_val)

            except Exception:
                break

    def _refrescar_grafica_serial(self, sp, rpm, pwm):
        if not self.hist_t:
            return

        t_vec = np.array(self.hist_t)
        sp_vec = np.array(self.hist_sp)
        y_vec = np.array(self.hist_y)
        u_vec = np.array(self.hist_u)

        # Actualizar datos de las líneas
        self.line_sp.set_data(t_vec, sp_vec)
        self.line_y.set_data(t_vec, y_vec)
        self.line_u.set_data(t_vec, u_vec)

        # Ajuste dinámico de los ejes
        t_min = t_vec[0]
        t_max = max(t_vec[-1], t_min + 1.0)
        self.ax_vel.set_xlim(t_min, t_max)
        self.ax_u.set_xlim(t_min, t_max)

        max_vel = max(np.max(sp_vec), np.max(y_vec))
        self.ax_vel.set_ylim(0, max_vel * 1.18 if max_vel > 0 else 100)

        # Actualizar indicadores numéricos
        self.lbl_rpm_act.config(text=f"RPM: {rpm:.1f}")
        self.lbl_pwm_act.config(text=f"PWM: {int(pwm)}")

        self.canvas.draw_idle()

    def ejecutar_simulacion(self):
        """Modo simulación analítica con diferencias finitas (Euler)."""
        if self.conectado:
            messagebox.showinfo("Aviso", "Desconecta el Arduino antes de ejecutar la simulación teórica.")
            return

        try:
            r = float(self.txt_sp.get())
            Kp = float(self.txt_kp.get())
        except ValueError:
            messagebox.showerror("Error", "Setpoint y Kp deben ser números válidos.")
            return

        pasos = int(self.t_final / self.dt)
        t = np.linspace(0, self.t_final, pasos)
        y = np.zeros(pasos)
        u = np.zeros(pasos)

        # Bucle de Euler con saturación física PWM [0, 255]
        for k in range(1, pasos):
            e_k = r - y[k - 1]
            u[k] = np.clip(Kp * e_k, 0.0, 255.0)
            dy = (self.K * u[k] - y[k - 1]) / self.tau
            y[k] = y[k - 1] + dy * self.dt

        # Graficar respuesta calculada
        self.line_sp.set_data(t, np.full_like(t, r))
        self.line_y.set_data(t, y)
        self.line_u.set_data(t, u)

        self.ax_vel.set_xlim(0, self.t_final)
        self.ax_vel.set_ylim(0, max(r, np.max(y)) * 1.18)
        self.ax_u.set_xlim(0, self.t_final)

        self.lbl_rpm_act.config(text=f"RPM: {y[-1]:.1f}")
        self.lbl_pwm_act.config(text=f"PWM: {int(u[-1])}")

        self.canvas.draw()

    def limpiar_datos(self):
        self.hist_t.clear()
        self.hist_sp.clear()
        self.hist_y.clear()
        self.hist_u.clear()
        self.line_sp.set_data([], [])
        self.line_y.set_data([], [])
        self.line_u.set_data([], [])
        self.canvas.draw()

    def _cerrar_aplicacion(self):
        self.desconectar_serial()
        self.root.destroy()

if __name__ == "__main__":
    ventana = tk.Tk()
    app = ControlMotorGUI(ventana)
    ventana.mainloop()
