"""
Interfaz grafica de la Practica 2 de Instrumentacion Virtual.

Integra el controlador PID (pid.py) y el enlace serie (serial_link.py) en una
unica ventana que permite:
  - Conectarse al ESP32 por puerto serie.
  - Elegir el subsistema a controlar (temperatura o velocidad).
  - Fijar la referencia, las ganancias Kp/Ki/Kd y el periodo de muestreo Ts.
  - Ver en tiempo real cuatro graficas: medida vs referencia, ciclo de trabajo
    PWM, error de seguimiento y componentes P/I/D del controlador.
  - Iniciar, detener, hacer parada de emergencia, resetear y exportar a CSV.

El bucle de control vive en _paso_control(): se ejecuta cada Ts mediante un
QTimer, lee la ultima medida del ESP32, calcula el PID y devuelve el ciclo de
trabajo. Este es el punto de entrada del proyecto: se ejecuta con 'python gui.py'.
"""

import sys
import csv
import time
import numpy as np

# Widgets y clases de PyQt6. En PyQt6 los enumerados estan "encapsulados"
# (por ejemplo Qt.PenStyle.DashLine), a diferencia de PyQt5.
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QFormLayout, QGroupBox, QLabel, QPushButton, QComboBox, QRadioButton,
    QButtonGroup, QDoubleSpinBox, QSpinBox, QFileDialog, QMessageBox
)
from PyQt6.QtCore import Qt, QTimer

import pyqtgraph as pg   # Libreria de graficos en tiempo real

# Modulos propios del proyecto (deben estar en la misma carpeta que este archivo)
from pid import PID
from serial_link import SerialLink, listar_puertos


class VentanaPrincipal(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Practica 2 - Control PID (temperatura / velocidad)")
        self.resize(1150, 680)

        # ---- Modelo: controlador PID y enlace serie ----
        # Ganancias iniciales para la planta termica (segun el bosquejo del preinforme)
        self.pid = PID(kp=18.0, ki=1.5, kd=0.5, setpoint=32.5,
                       out_min=0.0, out_max=100.0)
        self.enlace = SerialLink()

        # ---- Estado de la aplicacion ----
        self.modo = "T"           # "T" = temperatura, "V" = velocidad
        self.corriendo = False    # Indica si el bucle de control esta activo

        # ---- Historicos de datos (para graficar y exportar) ----
        self.PLOT_WINDOW = 1500   # Numero maximo de puntos visibles en las graficas
        self._limpiar_historicos()

        # ---- Variables de temporizacion ----
        self._t0 = 0.0            # Instante de inicio de la corrida (perf_counter)
        self._t_prev_paso = 0.0   # Instante del paso de control anterior
        self._rate_ema = 0.0      # Media movil de la tasa de muestreo efectiva (Hz)

        # ---- Construccion de la interfaz ----
        self._construir_ui()

        # ---- Temporizadores ----
        # Bucle de control: se dispara cada Ts y ejecuta el PID
        self.timer_control = QTimer(self)
        self.timer_control.timeout.connect(self._paso_control)
        # Monitor: refresca las etiquetas de valores actuales aunque no se controle
        self.timer_monitor = QTimer(self)
        self.timer_monitor.timeout.connect(self._paso_monitor)

    # ======================================================================
    #  Construccion de la interfaz
    # ======================================================================
    def _construir_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout_principal = QHBoxLayout(central)

        # ---------- Panel izquierdo: controles ----------
        panel_izq = QVBoxLayout()

        # Grupo: conexion serie
        g_con = QGroupBox("Conexion")
        lay_con = QVBoxLayout(g_con)
        fila_p = QHBoxLayout()
        self.cmb_puertos = QComboBox()
        self.cmb_puertos.addItems(listar_puertos())   # Rellena con los puertos detectados
        btn_ref = QPushButton("Refrescar")
        btn_ref.clicked.connect(self._on_refrescar)
        fila_p.addWidget(self.cmb_puertos)
        fila_p.addWidget(btn_ref)
        lay_con.addLayout(fila_p)
        self.btn_conectar = QPushButton("Conectar")
        self.btn_conectar.clicked.connect(self._on_conectar)
        lay_con.addWidget(self.btn_conectar)
        panel_izq.addWidget(g_con)

        # Grupo: seleccion del subsistema
        g_sys = QGroupBox("Sistema a controlar")
        lay_sys = QVBoxLayout(g_sys)
        self.rb_temp = QRadioButton("Control de temperatura")
        self.rb_vel = QRadioButton("Control de velocidad")
        self.rb_temp.setChecked(True)
        # Agrupamos los radios para garantizar exclusividad mutua
        self.grupo_sys = QButtonGroup(self)
        self.grupo_sys.addButton(self.rb_temp)
        self.grupo_sys.addButton(self.rb_vel)
        lay_sys.addWidget(self.rb_temp)
        lay_sys.addWidget(self.rb_vel)
        panel_izq.addWidget(g_sys)

        # Grupo: referencia y ganancias PID
        g_cfg = QGroupBox("Referencia y ganancias PID")
        form = QFormLayout(g_cfg)
        # Referencia (rango y unidades para temperatura por defecto)
        self.sp_ref = QDoubleSpinBox()
        self.sp_ref.setRange(30.0, 35.0)
        self.sp_ref.setSingleStep(0.5)
        self.sp_ref.setValue(32.5)
        self.sp_ref.setSuffix(" C")
        self.sp_ref.valueChanged.connect(self._on_setpoint)
        # Ganancias (3 decimales, no negativas)
        self.sp_kp = QDoubleSpinBox(); self.sp_kp.setRange(0.0, 1000.0)
        self.sp_kp.setDecimals(3); self.sp_kp.setValue(18.0)
        self.sp_kp.valueChanged.connect(self._sincronizar_ganancias)
        self.sp_ki = QDoubleSpinBox(); self.sp_ki.setRange(0.0, 1000.0)
        self.sp_ki.setDecimals(3); self.sp_ki.setValue(1.5)
        self.sp_ki.valueChanged.connect(self._sincronizar_ganancias)
        self.sp_kd = QDoubleSpinBox(); self.sp_kd.setRange(0.0, 1000.0)
        self.sp_kd.setDecimals(3); self.sp_kd.setValue(0.5)
        self.sp_kd.valueChanged.connect(self._sincronizar_ganancias)
        # Periodo de muestreo Ts en milisegundos
        self.sp_ts = QSpinBox(); self.sp_ts.setRange(10, 1000)
        self.sp_ts.setValue(200); self.sp_ts.setSuffix(" ms")
        self.sp_ts.valueChanged.connect(self._on_ts)
        form.addRow("Referencia:", self.sp_ref)
        form.addRow("Kp:", self.sp_kp)
        form.addRow("Ki:", self.sp_ki)
        form.addRow("Kd:", self.sp_kd)
        form.addRow("Ts:", self.sp_ts)
        panel_izq.addWidget(g_cfg)

        # Grupo: valores actuales (etiquetas de solo lectura)
        g_val = QGroupBox("Valores actuales")
        form_v = QFormLayout(g_val)
        self.lbl_medido = QLabel("-")
        self.lbl_pwm = QLabel("-")
        self.lbl_error = QLabel("-")
        form_v.addRow("Medido:", self.lbl_medido)
        form_v.addRow("PWM:", self.lbl_pwm)
        form_v.addRow("Error:", self.lbl_error)
        panel_izq.addWidget(g_val)

        # Grupo: acciones (botones)
        g_act = QGroupBox("Acciones")
        lay_act = QVBoxLayout(g_act)
        self.btn_iniciar = QPushButton("Iniciar")
        self.btn_iniciar.clicked.connect(self._on_iniciar)
        self.btn_iniciar.setEnabled(False)   # Solo se habilita al conectar
        self.btn_detener = QPushButton("Detener")
        self.btn_detener.clicked.connect(self._on_detener)
        self.btn_emerg = QPushButton("Parada de emergencia")
        self.btn_emerg.clicked.connect(self._on_emergencia)
        # Resaltamos visualmente la parada de emergencia
        self.btn_emerg.setStyleSheet(
            "background-color: #c0392b; color: white; font-weight: bold;")
        self.btn_reset = QPushButton("Reset")
        self.btn_reset.clicked.connect(self._on_reset)
        self.btn_export = QPushButton("Exportar datos a CSV")
        self.btn_export.clicked.connect(self._on_exportar)
        for b in (self.btn_iniciar, self.btn_detener, self.btn_emerg,
                  self.btn_reset, self.btn_export):
            lay_act.addWidget(b)
        panel_izq.addWidget(g_act)

        panel_izq.addStretch(1)   # Empuja los grupos hacia arriba

        # ---------- Panel derecho: cuatro graficas ----------
        pg.setConfigOptions(antialias=True)   # Suavizado de lineas
        grid = QGridLayout()
        self.plot_medida = pg.PlotWidget()
        self.plot_duty = pg.PlotWidget()
        self.plot_error = pg.PlotWidget()
        self.plot_pid = pg.PlotWidget()
        for p in (self.plot_medida, self.plot_duty, self.plot_error, self.plot_pid):
            p.showGrid(x=True, y=True)
            p.setLabel("bottom", "Tiempo (s)")
        self.plot_medida.setTitle("Medida vs referencia")
        self.plot_medida.setLabel("left", "Temperatura (C)")
        self.plot_duty.setTitle("Ciclo de trabajo PWM aplicado")
        self.plot_duty.setLabel("left", "PWM (%)")
        self.plot_duty.setYRange(0, 100)
        self.plot_error.setTitle("Error de seguimiento")
        self.plot_error.setLabel("left", "Error")
        self.plot_pid.setTitle("Componentes del PID")
        self.plot_pid.setLabel("left", "Magnitud")
        # Las leyendas deben crearse antes de las curvas con nombre
        self.plot_medida.addLegend()
        self.plot_pid.addLegend()
        # Curvas (PlotDataItem que luego actualizamos con setData)
        self.c_medida = self.plot_medida.plot(
            pen=pg.mkPen("#1f77b4", width=2), name="Medida")
        self.c_ref = self.plot_medida.plot(
            pen=pg.mkPen("#d62728", width=2, style=Qt.PenStyle.DashLine),
            name="Referencia")
        self.c_duty = self.plot_duty.plot(pen=pg.mkPen("#2ca02c", width=2))
        self.c_error = self.plot_error.plot(pen=pg.mkPen("#ff7f0e", width=2))
        self.c_p = self.plot_pid.plot(pen=pg.mkPen("#1f77b4", width=2), name="P")
        self.c_i = self.plot_pid.plot(pen=pg.mkPen("#2ca02c", width=2), name="I")
        self.c_d = self.plot_pid.plot(pen=pg.mkPen("#d62728", width=2), name="D")
        grid.addWidget(self.plot_medida, 0, 0)
        grid.addWidget(self.plot_duty, 0, 1)
        grid.addWidget(self.plot_error, 1, 0)
        grid.addWidget(self.plot_pid, 1, 1)

        # ---------- Ensamblar ambos paneles ----------
        cont_izq = QWidget()
        cont_izq.setLayout(panel_izq)
        cont_izq.setMaximumWidth(340)   # Limita el ancho del panel de controles
        layout_principal.addWidget(cont_izq)
        layout_principal.addLayout(grid, stretch=1)

        # ---------- Barra de estado ----------
        self.lbl_estado = QLabel("Desconectado")
        self.lbl_rate = QLabel("Ts efectivo: -")
        self.statusBar().addPermanentWidget(self.lbl_estado)
        self.statusBar().addPermanentWidget(self.lbl_rate)

        # Conectamos el cambio de modo AL FINAL, cuando ya existen todos los
        # widgets y graficas que _cambiar_modo necesita manipular.
        self.rb_temp.toggled.connect(self._cambiar_modo)

    # ======================================================================
    #  Utilidades internas
    # ======================================================================
    def _limpiar_historicos(self):
        """Vacia todas las listas de datos historicos."""
        self.hist_t = []        # Tiempo relativo (s)
        self.hist_medida = []   # Variable medida (C o RPM)
        self.hist_ref = []      # Referencia
        self.hist_duty = []     # Ciclo de trabajo aplicado (%)
        self.hist_error = []    # Error de seguimiento
        self.hist_p = []        # Componente proporcional
        self.hist_i = []        # Componente integral
        self.hist_d = []        # Componente derivativa

    def _sincronizar_ganancias(self):
        """Copia las ganancias de los spinbox al objeto PID (permite tuning en vivo)."""
        self.pid.kp = self.sp_kp.value()
        self.pid.ki = self.sp_ki.value()
        self.pid.kd = self.sp_kd.value()

    def _actualizar_etiquetas(self, medida, duty, error):
        """Refresca las etiquetas de valores actuales."""
        uni = "C" if self.modo == "T" else "RPM"
        self.lbl_medido.setText(f"{medida:.2f} {uni}")
        self.lbl_pwm.setText(f"{duty:.1f} %")
        self.lbl_error.setText(f"{error:+.2f} {uni}")   # El signo '+' muestra el sentido

    def _actualizar_graficas(self):
        """Vuelca los ultimos PLOT_WINDOW puntos del historico a las curvas."""
        N = self.PLOT_WINDOW
        # Convertimos a arrays de numpy (setData es mas eficiente con ellos)
        t = np.array(self.hist_t[-N:])
        self.c_medida.setData(t, np.array(self.hist_medida[-N:]))
        self.c_ref.setData(t, np.array(self.hist_ref[-N:]))
        self.c_duty.setData(t, np.array(self.hist_duty[-N:]))
        self.c_error.setData(t, np.array(self.hist_error[-N:]))
        self.c_p.setData(t, np.array(self.hist_p[-N:]))
        self.c_i.setData(t, np.array(self.hist_i[-N:]))
        self.c_d.setData(t, np.array(self.hist_d[-N:]))

    # ======================================================================
    #  Manejadores de eventos de la interfaz (slots)
    # ======================================================================
    def _on_refrescar(self):
        """Vuelve a enumerar los puertos serie disponibles."""
        self.cmb_puertos.clear()
        self.cmb_puertos.addItems(listar_puertos())

    def _on_conectar(self):
        """Conecta o desconecta el puerto serie segun el estado actual."""
        if self.enlace.conectado():
            # --- Desconectar ---
            self._on_detener()
            self.enlace.desconectar()
            self.timer_monitor.stop()
            self.btn_conectar.setText("Conectar")
            self.lbl_estado.setText("Desconectado")
            self.btn_iniciar.setEnabled(False)
            return
        # --- Conectar ---
        puerto = self.cmb_puertos.currentText()
        if not puerto:
            QMessageBox.warning(self, "Sin puerto", "No hay ningun puerto seleccionado.")
            return
        try:
            self.enlace.conectar(puerto, 115200)
        except Exception as e:
            # Captura amplia: puerto ocupado, permisos, inexistente, etc.
            QMessageBox.critical(self, "Error de conexion", str(e))
            return
        self.enlace.enviar("MODE," + self.modo)   # Informa el modo actual al ESP32
        self.btn_conectar.setText("Desconectar")
        self.lbl_estado.setText(f"Conectado: {puerto}")
        self.btn_iniciar.setEnabled(True)
        self.timer_monitor.start(150)             # Monitor de etiquetas cada 150 ms

    def _cambiar_modo(self):
        """Reconfigura la interfaz al cambiar entre temperatura y velocidad."""
        if self.rb_temp.isChecked():
            self.modo = "T"
            # Rango y unidades de la referencia para la planta termica
            self.sp_ref.setRange(30.0, 35.0)
            self.sp_ref.setSingleStep(0.5)
            self.sp_ref.setSuffix(" C")
            self.sp_ref.setValue(32.5)
            # Ganancias por defecto sugeridas para temperatura
            self.sp_kp.setValue(18.0)
            self.sp_ki.setValue(1.5)
            self.sp_kd.setValue(0.5)
            self.sp_ts.setValue(200)   # La planta termica es lenta: Ts amplio
            self.plot_medida.setLabel("left", "Temperatura (C)")
            self.plot_medida.setTitle("Temperatura medida vs referencia")
        else:
            self.modo = "V"
            # Rango y unidades de la referencia para el motor
            self.sp_ref.setRange(0.0, 250.0)
            self.sp_ref.setSingleStep(5.0)
            self.sp_ref.setSuffix(" RPM")
            self.sp_ref.setValue(120.0)
            # Ganancias por defecto sugeridas para el motor (PUNTO DE PARTIDA; ajustar)
            self.sp_kp.setValue(0.10)
            self.sp_ki.setValue(0.50)
            self.sp_kd.setValue(0.00)
            self.sp_ts.setValue(50)    # El motor responde rapido: Ts pequeno
            self.plot_medida.setLabel("left", "Velocidad (RPM)")
            self.plot_medida.setTitle("Velocidad medida vs referencia")
        # Aplica los cambios al PID y avisa al ESP32
        self.pid.setpoint = self.sp_ref.value()
        self._sincronizar_ganancias()
        if self.enlace.conectado():
            self.enlace.enviar("MODE," + self.modo)
        # Una corrida nueva: reiniciamos el PID y limpiamos las graficas
        self.pid.reset()
        self._limpiar_historicos()
        self._actualizar_graficas()

    def _on_setpoint(self):
        """Actualiza la referencia del PID al cambiar el spinbox (tuning en vivo)."""
        self.pid.setpoint = self.sp_ref.value()

    def _on_ts(self):
        """Aplica el nuevo Ts al temporizador de control si esta corriendo."""
        if self.corriendo:
            self.timer_control.setInterval(int(self.sp_ts.value()))

    def _on_iniciar(self):
        """Arranca el bucle de control."""
        if not self.enlace.conectado():
            QMessageBox.warning(self, "Sin conexion", "Conecta primero el ESP32.")
            return
        # Aseguramos modo, ganancias y referencia sincronizadas con el ESP32 y el PID
        self.enlace.enviar("MODE," + self.modo)
        self._sincronizar_ganancias()
        self.pid.setpoint = self.sp_ref.value()
        self.pid.reset()
        # Empezamos una corrida limpia
        self._limpiar_historicos()
        self._t0 = time.perf_counter()
        self._t_prev_paso = self._t0
        self._rate_ema = 0.0
        self.corriendo = True
        self.timer_control.setInterval(int(self.sp_ts.value()))
        self.timer_control.start()

    def _on_detener(self):
        """Detiene el bucle de control y apaga la salida (pausa)."""
        self.corriendo = False
        self.timer_control.stop()
        self.enlace.enviar("STOP")

    def _on_emergencia(self):
        """Corta la salida de inmediato y reinicia el controlador."""
        self.corriendo = False
        self.timer_control.stop()
        # Enviamos STOP por duplicado para asegurar el corte aunque se pierda un byte
        self.enlace.enviar("STOP")
        self.enlace.enviar("STOP")
        self.pid.reset()

    def _on_reset(self):
        """Reinicia el PID, el contador del encoder y limpia las graficas."""
        self.pid.reset()
        if self.enlace.conectado():
            self.enlace.enviar("RESET")
        self._limpiar_historicos()
        self._actualizar_graficas()

    def _on_exportar(self):
        """Exporta todo el historico de la corrida a un archivo CSV."""
        if not self.hist_t:
            QMessageBox.information(self, "Sin datos", "No hay datos para exportar.")
            return
        # Dialogo de guardado; devuelve (ruta, filtro_seleccionado)
        ruta, _ = QFileDialog.getSaveFileName(
            self, "Guardar CSV", "registro.csv", "CSV (*.csv)")
        if not ruta:
            return
        with open(ruta, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "modo", "referencia", "medida",
                        "error", "duty_pct", "P", "I", "D"])
            for k in range(len(self.hist_t)):
                w.writerow([
                    f"{self.hist_t[k]:.4f}", self.modo,
                    f"{self.hist_ref[k]:.4f}", f"{self.hist_medida[k]:.4f}",
                    f"{self.hist_error[k]:.4f}", f"{self.hist_duty[k]:.4f}",
                    f"{self.hist_p[k]:.4f}", f"{self.hist_i[k]:.4f}",
                    f"{self.hist_d[k]:.4f}",
                ])
        QMessageBox.information(self, "Exportado", f"Datos guardados en:\n{ruta}")

    # ======================================================================
    #  Bucles temporizados
    # ======================================================================
    def _paso_monitor(self):
        """
        Refresca las etiquetas de valores actuales con la ultima telemetria,
        este o no corriendo el control. Permite ver la medida en vivo.
        """
        if not self.enlace.conectado():
            return
        lectura = self.enlace.ultima_lectura()
        if lectura is None:
            return
        tipo, valor, duty_echo, _ = lectura
        if tipo != self.modo:
            return   # Dato del modo anterior: esperamos al del modo actual
        error = self.sp_ref.value() - valor
        self._actualizar_etiquetas(valor, duty_echo, error)

    def _paso_control(self):
        """
        Nucleo del lazo cerrado. Se ejecuta cada Ts:
          1) Toma la ultima medida del ESP32.
          2) Calcula el PID con el dt real transcurrido.
          3) Envia el ciclo de trabajo resultante al ESP32.
          4) Registra los datos y actualiza graficas y etiquetas.
        """
        if not self.enlace.conectado():
            return
        lectura = self.enlace.ultima_lectura()
        if lectura is None:
            return   # Aun no ha llegado ninguna medida
        tipo, valor, _, _ = lectura
        if tipo != self.modo:
            return   # Ignoramos datos del modo que no corresponde

        # dt real entre pasos de control (mas robusto que asumir Ts exacto)
        ahora = time.perf_counter()
        dt = ahora - self._t_prev_paso
        self._t_prev_paso = ahora

        # Calculo del PID; la salida u es directamente el ciclo de trabajo (%)
        u, p, i, d = self.pid.compute(valor, dt)

        # Enviamos el ciclo de trabajo al ESP32
        self.enlace.enviar(f"DUTY,{u:.2f}")

        # Registro de datos
        t_rel = ahora - self._t0
        error = self.pid.setpoint - valor
        self.hist_t.append(t_rel)
        self.hist_medida.append(valor)
        self.hist_ref.append(self.pid.setpoint)
        self.hist_duty.append(u)
        self.hist_error.append(error)
        self.hist_p.append(p)
        self.hist_i.append(i)
        self.hist_d.append(d)

        # Actualizacion visual
        self._actualizar_graficas()
        self._actualizar_etiquetas(valor, u, error)

        # Tasa de muestreo efectiva (media movil exponencial)
        if dt > 0:
            inst = 1.0 / dt
            self._rate_ema = inst if self._rate_ema <= 0 else 0.9 * self._rate_ema + 0.1 * inst
            self.lbl_rate.setText(
                f"Ts efectivo: {1000.0 / self._rate_ema:.0f} ms ({self._rate_ema:.1f} Hz)")

    # ======================================================================
    #  Cierre de la ventana
    # ======================================================================
    def closeEvent(self, event):
        """Al cerrar, detiene el control y deja el hardware en estado seguro."""
        self._on_detener()
        if self.enlace.conectado():
            self.enlace.enviar("STOP")
            self.enlace.desconectar()
        event.accept()


def main():
    app = QApplication(sys.argv)
    ventana = VentanaPrincipal()
    ventana.show()
    # app.exec() inicia el bucle de eventos de Qt (en PyQt6 es exec(), no exec_())
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
