"""
Interfaz grafica para control PID de temperatura y velocidad de motor DC.
Practica 2 - Instrumentacion Virtual.

Esta version genera datos sinteticos para captura de pantallas; no requiere
hardware. Para uso real con un ESP32, reemplazar la clase PIDSimulator por
una capa de comunicacion via pyserial que lea la variable medida del MCU y
envie la senal de control PWM al mismo.

Dependencias en Arch Linux:
    sudo pacman -S python-pyqt6 python-pyqtgraph python-numpy

Ejecucion:
    python gui_pid_practica2.py
"""

# sys: acceso a argumentos de linea de comandos y a sys.exit()
import sys
# collections.deque: buffer circular de tamano fijo para las graficas
from collections import deque
# numpy: manejo eficiente de arreglos numericos
import numpy as np

# Importacion de widgets de PyQt6 utilizados en la interfaz
from PyQt6.QtWidgets import (
    QApplication,      # punto de entrada del ciclo de eventos Qt
    QMainWindow,       # ventana principal con barra de estado, menu, etc.
    QWidget,           # contenedor generico
    QTabWidget,        # widget de pestanas para los dos sub-sistemas
    QVBoxLayout,       # disposicion vertical
    QHBoxLayout,       # disposicion horizontal
    QGridLayout,       # disposicion en cuadricula
    QLabel,            # texto estatico
    QPushButton,       # boton presionable
    QDoubleSpinBox,    # campo numerico con flechas para ajustar valores
    QGroupBox,         # caja con titulo para agrupar widgets
    QStatusBar,        # barra de estado en la parte inferior
    QFileDialog,       # dialogo para seleccionar archivos
    QMessageBox        # ventana de mensaje
)
# QTimer: temporizador periodico no bloqueante (para refrescar las graficas)
# Qt: enumeraciones (alineacion, estilos de linea, etc.)
from PyQt6.QtCore import QTimer, Qt
# QFont: configuracion de fuentes
from PyQt6.QtGui import QFont

# pyqtgraph: biblioteca de graficas en tiempo real basada en Qt y numpy
import pyqtgraph as pg

# ---------- Configuracion global de pyqtgraph ----------
# Fondo blanco con texto oscuro: optimo para captura de pantalla y documentos
pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')
# Antialiasing activado para que las lineas se vean suaves en el screenshot
pg.setConfigOption('antialias', True)


class PIDSimulator:
    """
    Simulador de planta de primer orden con un controlador PID estandar.
    Su unico proposito es generar trayectorias sinteticas que parezcan
    realistas para llenar las graficas durante la captura de pantalla.
    No representa un controlador para uso en hardware.
    """

    def __init__(self, tau, gain, noise=0.0, dt=0.05, initial_value=0.0):
        # tau: constante de tiempo de la planta de primer orden, en segundos
        self.tau = tau
        # gain: ganancia estatica de la planta (unidades de salida por % PWM)
        self.gain = gain
        # noise: desviacion estandar del ruido gaussiano aditivo en la medida
        self.noise = noise
        # dt: periodo de muestreo en segundos (Euler explicito)
        self.dt = dt
        # Estado interno de la planta (variable de proceso real)
        self.y = initial_value
        # Acumulador del termino integral del controlador
        self.integral = 0.0
        # Error en el paso anterior, necesario para el termino derivativo
        self.prev_error = 0.0

    def step(self, setpoint, kp, ki, kd):
        """
        Ejecuta un paso del lazo cerrado. Devuelve la medida ruidosa, la
        senal de control aplicada, el error y las tres componentes del PID
        para poder graficarlas por separado.
        """
        # Error de seguimiento entre referencia y variable medida
        error = setpoint - self.y
        # Termino proporcional
        p_term = kp * error
        # Acumulacion del termino integral por aproximacion rectangular
        self.integral += error * self.dt
        i_term = ki * self.integral
        # Termino derivativo por diferencia finita hacia atras
        derivative = (error - self.prev_error) / self.dt
        d_term = kd * derivative
        # Senal de control sin saturar
        u_unsat = p_term + i_term + d_term
        # Saturacion del actuador entre 0 % y 100 % de ciclo de trabajo PWM
        u = max(0.0, min(100.0, u_unsat))
        # Anti-windup por saturacion condicional: si el actuador esta saturado
        # se descuenta la ultima integracion para evitar acumulacion excesiva
        if u != u_unsat:
            self.integral -= error * self.dt
            i_term = ki * self.integral
        # Actualizacion de la planta de primer orden discretizada (Euler)
        # dy/dt = (-y + gain * u) / tau
        self.y += self.dt * (-self.y + self.gain * u) / self.tau
        # Se devuelve la medida con ruido aditivo de simulacion
        measured = self.y + np.random.randn() * self.noise
        # Persistencia del error para el siguiente paso
        self.prev_error = error
        return measured, u, error, p_term, i_term, d_term


class ControlPanel(QWidget):
    """
    Panel reutilizable para uno de los dos lazos de control. Cada panel
    contiene los controles de ajuste (referencia, ganancias) y cuatro
    graficas: variable medida vs referencia, PWM, error y componentes PID.
    """

    def __init__(self, y_label, y_unit, y_min, y_max,
                 default_setpoint, default_kp, default_ki, default_kd,
                 plant_tau, plant_gain, plant_noise, plant_initial,
                 plot_y_padding):
        super().__init__()

        # ---- Buffers circulares para las graficas ----
        # Capacidad del buffer: ~15 s de datos a 50 ms de muestreo
        self.buffer_size = 300
        # Tiempo, variable medida, referencia, PWM y error
        self.t_buf = deque(maxlen=self.buffer_size)
        self.y_buf = deque(maxlen=self.buffer_size)
        self.sp_buf = deque(maxlen=self.buffer_size)
        self.pwm_buf = deque(maxlen=self.buffer_size)
        self.err_buf = deque(maxlen=self.buffer_size)
        # Componentes individuales del PID
        self.p_buf = deque(maxlen=self.buffer_size)
        self.i_buf = deque(maxlen=self.buffer_size)
        self.d_buf = deque(maxlen=self.buffer_size)
        # Contador de tiempo simulado en segundos
        self.t = 0.0
        # Bandera que indica si el lazo de control esta activo
        self.running = False
        # Instancia del simulador interno
        self.sim = PIDSimulator(plant_tau, plant_gain, plant_noise,
                                dt=0.05, initial_value=plant_initial)

        # ---- Layout principal en dos columnas (controles + graficas) ----
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(10)

        # ============================================================
        # Columna izquierda: configuracion y estado
        # ============================================================
        left_col = QVBoxLayout()
        left_col.setSpacing(10)

        # --- Grupo: referencia y ganancias PID ---
        ref_group = QGroupBox("Referencia y ganancias PID")
        ref_layout = QGridLayout(ref_group)
        # Etiqueta y campo para el setpoint
        ref_layout.addWidget(QLabel(f"Referencia ({y_unit}):"), 0, 0)
        self.setpoint_spin = QDoubleSpinBox()
        self.setpoint_spin.setRange(y_min, y_max)
        self.setpoint_spin.setValue(default_setpoint)
        self.setpoint_spin.setSingleStep(0.5)
        self.setpoint_spin.setDecimals(2)
        ref_layout.addWidget(self.setpoint_spin, 0, 1)
        # Etiqueta y campo para Kp
        ref_layout.addWidget(QLabel("Kp:"), 1, 0)
        self.kp_spin = QDoubleSpinBox()
        self.kp_spin.setRange(0.0, 1000.0)
        self.kp_spin.setValue(default_kp)
        self.kp_spin.setSingleStep(0.1)
        self.kp_spin.setDecimals(3)
        ref_layout.addWidget(self.kp_spin, 1, 1)
        # Etiqueta y campo para Ki
        ref_layout.addWidget(QLabel("Ki:"), 2, 0)
        self.ki_spin = QDoubleSpinBox()
        self.ki_spin.setRange(0.0, 1000.0)
        self.ki_spin.setValue(default_ki)
        self.ki_spin.setSingleStep(0.05)
        self.ki_spin.setDecimals(3)
        ref_layout.addWidget(self.ki_spin, 2, 1)
        # Etiqueta y campo para Kd
        ref_layout.addWidget(QLabel("Kd:"), 3, 0)
        self.kd_spin = QDoubleSpinBox()
        self.kd_spin.setRange(0.0, 1000.0)
        self.kd_spin.setValue(default_kd)
        self.kd_spin.setSingleStep(0.005)
        self.kd_spin.setDecimals(3)
        ref_layout.addWidget(self.kd_spin, 3, 1)
        left_col.addWidget(ref_group)

        # --- Grupo: valores actuales en formato numerico ---
        info_group = QGroupBox("Valores actuales")
        info_layout = QGridLayout(info_group)
        # Fuente en negrita para los valores que se actualizan
        bold = QFont()
        bold.setBold(True)
        bold.setPointSize(11)
        # Etiquetas estaticas y dinamicas para cada magnitud
        info_layout.addWidget(QLabel(f"Medido ({y_unit}):"), 0, 0)
        self.lbl_y = QLabel("---")
        self.lbl_y.setFont(bold)
        info_layout.addWidget(self.lbl_y, 0, 1)
        info_layout.addWidget(QLabel("PWM (%):"), 1, 0)
        self.lbl_pwm = QLabel("---")
        self.lbl_pwm.setFont(bold)
        info_layout.addWidget(self.lbl_pwm, 1, 1)
        info_layout.addWidget(QLabel(f"Error ({y_unit}):"), 2, 0)
        self.lbl_err = QLabel("---")
        self.lbl_err.setFont(bold)
        info_layout.addWidget(self.lbl_err, 2, 1)
        left_col.addWidget(info_group)

        # --- Grupo: botones de control ---
        btn_group = QGroupBox("Acciones")
        btn_layout = QVBoxLayout(btn_group)
        btn_layout.setSpacing(6)
        # Boton de iniciar
        self.btn_start = QPushButton("Iniciar")
        self.btn_start.setStyleSheet(
            "background-color: #27ae60; color: white; "
            "font-weight: bold; padding: 6px;"
        )
        self.btn_start.clicked.connect(self.start)
        btn_layout.addWidget(self.btn_start)
        # Boton de detener
        self.btn_stop = QPushButton("Detener")
        self.btn_stop.setStyleSheet(
            "background-color: #7f8c8d; color: white; "
            "font-weight: bold; padding: 6px;"
        )
        self.btn_stop.clicked.connect(self.stop)
        btn_layout.addWidget(self.btn_stop)
        # Boton de parada de emergencia
        self.btn_emergency = QPushButton("PARADA DE EMERGENCIA")
        self.btn_emergency.setStyleSheet(
            "background-color: #c0392b; color: white; "
            "font-weight: bold; padding: 8px;"
        )
        self.btn_emergency.clicked.connect(self.emergency)
        btn_layout.addWidget(self.btn_emergency)
        # Boton de exportacion de datos a CSV
        self.btn_export = QPushButton("Exportar datos a CSV")
        self.btn_export.clicked.connect(self.export_csv)
        btn_layout.addWidget(self.btn_export)
        left_col.addWidget(btn_group)

        # Espaciador para empujar los grupos hacia arriba
        left_col.addStretch()

        # Envoltura del layout izquierdo en un QWidget para limitar su ancho
        left_widget = QWidget()
        left_widget.setLayout(left_col)
        left_widget.setMaximumWidth(300)
        left_widget.setMinimumWidth(280)
        main_layout.addWidget(left_widget)

        # ============================================================
        # Columna derecha: cuatro graficas en cuadricula 2x2
        # ============================================================
        right_grid = QGridLayout()
        right_grid.setSpacing(8)

        # --- Grafica 1: variable medida vs referencia ---
        self.plot_y = pg.PlotWidget(title=f"{y_label} medida vs referencia")
        self.plot_y.setLabel('left', f"{y_label} ({y_unit})")
        self.plot_y.setLabel('bottom', 'Tiempo', units='s')
        # Margen vertical adicional al rango para mostrar el sobreimpulso
        self.plot_y.setYRange(y_min - plot_y_padding, y_max + plot_y_padding)
        self.plot_y.addLegend(offset=(10, 10))
        self.plot_y.showGrid(x=True, y=True, alpha=0.3)
        # Curva azul para la medida
        self.curve_y = self.plot_y.plot(
            pen=pg.mkPen('#2980b9', width=2), name='Medida'
        )
        # Curva roja punteada para la referencia
        self.curve_sp = self.plot_y.plot(
            pen=pg.mkPen('#e74c3c', width=2, style=Qt.PenStyle.DashLine),
            name='Referencia'
        )
        right_grid.addWidget(self.plot_y, 0, 0)

        # --- Grafica 2: ciclo de trabajo PWM aplicado ---
        self.plot_pwm = pg.PlotWidget(title="Ciclo de trabajo PWM aplicado")
        self.plot_pwm.setLabel('left', 'PWM', units='%')
        self.plot_pwm.setLabel('bottom', 'Tiempo', units='s')
        self.plot_pwm.setYRange(-5, 105)
        self.plot_pwm.showGrid(x=True, y=True, alpha=0.3)
        # Curva verde para el PWM
        self.curve_pwm = self.plot_pwm.plot(
            pen=pg.mkPen('#27ae60', width=2), fillLevel=0,
            brush=pg.mkBrush(39, 174, 96, 50)
        )
        right_grid.addWidget(self.plot_pwm, 0, 1)

        # --- Grafica 3: error de seguimiento ---
        self.plot_err = pg.PlotWidget(title="Error de seguimiento")
        self.plot_err.setLabel('left', f"Error ({y_unit})")
        self.plot_err.setLabel('bottom', 'Tiempo', units='s')
        self.plot_err.showGrid(x=True, y=True, alpha=0.3)
        # Linea horizontal de referencia en cero
        self.plot_err.addLine(
            y=0, pen=pg.mkPen('k', style=Qt.PenStyle.DotLine, width=1)
        )
        # Curva naranja para el error
        self.curve_err = self.plot_err.plot(
            pen=pg.mkPen('#e67e22', width=2)
        )
        right_grid.addWidget(self.plot_err, 1, 0)

        # --- Grafica 4: componentes individuales del PID ---
        self.plot_pid = pg.PlotWidget(title="Componentes del controlador PID")
        self.plot_pid.setLabel('left', 'Magnitud')
        self.plot_pid.setLabel('bottom', 'Tiempo', units='s')
        self.plot_pid.addLegend(offset=(10, 10))
        self.plot_pid.showGrid(x=True, y=True, alpha=0.3)
        # Curvas de las tres componentes con colores distintos
        self.curve_p = self.plot_pid.plot(
            pen=pg.mkPen('#8e44ad', width=2), name='P'
        )
        self.curve_i = self.plot_pid.plot(
            pen=pg.mkPen('#16a085', width=2), name='I'
        )
        self.curve_d = self.plot_pid.plot(
            pen=pg.mkPen('#d35400', width=2), name='D'
        )
        right_grid.addWidget(self.plot_pid, 1, 1)

        # Envoltura del grid derecho en un QWidget
        right_widget = QWidget()
        right_widget.setLayout(right_grid)
        main_layout.addWidget(right_widget, stretch=1)

        # ============================================================
        # Temporizador que actualiza la simulacion y las graficas
        # ============================================================
        # Intervalo de muestreo en milisegundos (debe coincidir con sim.dt)
        self.dt_ms = 50
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_step)
        self.timer.start(self.dt_ms)

    def update_step(self):
        """Avanza un paso de simulacion y refresca todas las graficas."""
        # Si el lazo esta activo se ejecuta el simulador; si no, se mantiene
        # la planta en reposo con la senal de control fijada a cero
        if self.running:
            kp = self.kp_spin.value()
            ki = self.ki_spin.value()
            kd = self.kd_spin.value()
            setpoint = self.setpoint_spin.value()
            y, u, err, p, i, d = self.sim.step(setpoint, kp, ki, kd)
        else:
            y = self.sim.y
            u = 0.0
            err = self.setpoint_spin.value() - y
            p, i, d = 0.0, 0.0, 0.0

        # Avance del reloj de simulacion
        self.t += self.dt_ms / 1000.0
        # Insercion de los nuevos valores en los buffers circulares
        self.t_buf.append(self.t)
        self.y_buf.append(y)
        self.sp_buf.append(self.setpoint_spin.value())
        self.pwm_buf.append(u)
        self.err_buf.append(err)
        self.p_buf.append(p)
        self.i_buf.append(i)
        self.d_buf.append(d)

        # Conversion a arreglos de numpy (pyqtgraph trabaja con arrays)
        t_arr = np.array(self.t_buf)
        # Actualizacion de cada curva con los datos del buffer
        self.curve_y.setData(t_arr, np.array(self.y_buf))
        self.curve_sp.setData(t_arr, np.array(self.sp_buf))
        self.curve_pwm.setData(t_arr, np.array(self.pwm_buf))
        self.curve_err.setData(t_arr, np.array(self.err_buf))
        self.curve_p.setData(t_arr, np.array(self.p_buf))
        self.curve_i.setData(t_arr, np.array(self.i_buf))
        self.curve_d.setData(t_arr, np.array(self.d_buf))

        # Actualizacion de las etiquetas numericas con los ultimos valores
        self.lbl_y.setText(f"{y:.2f}")
        self.lbl_pwm.setText(f"{u:.1f}")
        self.lbl_err.setText(f"{err:+.2f}")

    def start(self):
        """Inicia el lazo de control."""
        self.running = True

    def stop(self):
        """Detiene el lazo de control sin reiniciar el estado."""
        self.running = False

    def emergency(self):
        """Parada de emergencia: detiene el lazo y reinicia el simulador."""
        self.running = False
        # Reinicio del estado interno del simulador
        self.sim.y = 0.0
        self.sim.integral = 0.0
        self.sim.prev_error = 0.0
        # Limpieza de buffers para que las graficas se vacien
        self.t_buf.clear()
        self.y_buf.clear()
        self.sp_buf.clear()
        self.pwm_buf.clear()
        self.err_buf.clear()
        self.p_buf.clear()
        self.i_buf.clear()
        self.d_buf.clear()
        self.t = 0.0

    def export_csv(self):
        """Exporta el contenido actual de los buffers a un archivo CSV."""
        # Apertura del dialogo de seleccion de archivo
        filename, _ = QFileDialog.getSaveFileName(
            self, "Exportar datos", "datos_pid.csv",
            "Archivos CSV (*.csv);;Todos (*)"
        )
        # Si el usuario cancela, no se hace nada
        if not filename:
            return
        # Escritura sencilla columna por columna sin dependencia externa
        try:
            with open(filename, "w", encoding="utf-8") as f:
                # Cabecera del archivo
                f.write("tiempo_s,medida,referencia,pwm_pct,error,P,I,D\n")
                # Filas con los valores actuales de los buffers
                for row in zip(self.t_buf, self.y_buf, self.sp_buf,
                               self.pwm_buf, self.err_buf,
                               self.p_buf, self.i_buf, self.d_buf):
                    f.write(",".join(f"{v:.4f}" for v in row) + "\n")
            QMessageBox.information(
                self, "Exportacion completada",
                f"Datos exportados a:\n{filename}"
            )
        except OSError as e:
            QMessageBox.critical(
                self, "Error al exportar",
                f"No se pudo escribir el archivo:\n{e}"
            )


class MainWindow(QMainWindow):
    """Ventana principal con dos pestanas, una por subsistema controlado."""

    def __init__(self):
        super().__init__()
        # Titulo de la ventana
        self.setWindowTitle(
            "Control PID - Practica 2 Instrumentacion Virtual"
        )
        # Dimensiones iniciales adecuadas para una captura legible
        self.resize(1280, 760)

        # Widget central con pestanas
        self.tabs = QTabWidget()
        # Estilo visual de las pestanas
        self.tabs.setStyleSheet(
            "QTabBar::tab { padding: 8px 16px; font-weight: bold; }"
            "QTabBar::tab:selected { background: #34495e; color: white; }"
        )
        self.setCentralWidget(self.tabs)

        # ---- Pestana 1: control de temperatura ----
        # Parametros de la planta termica: respuesta lenta (tau grande),
        # ganancia tal que a 100% PWM la temperatura tienda a ~50C, asi
        # el controlador puede alcanzar setpoints en el rango 30-35C sin
        # saturar. Inicio a temperatura ambiente.
        self.temp_panel = ControlPanel(
            y_label="Temperatura", y_unit="C",
            y_min=30.0, y_max=35.0,
            default_setpoint=32.5,
            default_kp=18.0, default_ki=1.5, default_kd=0.5,
            plant_tau=5.0, plant_gain=0.50,
            plant_noise=0.04, plant_initial=25.0,
            plot_y_padding=3.0
        )
        self.tabs.addTab(self.temp_panel, "Control de Temperatura")

        # ---- Pestana 2: control de velocidad ----
        # Planta electromecanica: respuesta rapida, ruido mayor por encoder.
        # Ganancia tal que a 100% PWM la velocidad tienda a ~250 RPM.
        self.motor_panel = ControlPanel(
            y_label="Velocidad", y_unit="RPM",
            y_min=0.0, y_max=250.0,
            default_setpoint=180.0,
            default_kp=0.35, default_ki=2.5, default_kd=0.015,
            plant_tau=0.30, plant_gain=2.5,
            plant_noise=2.5, plant_initial=0.0,
            plot_y_padding=20.0
        )
        self.tabs.addTab(self.motor_panel, "Control de Velocidad")

        # Barra de estado en la parte inferior con info auxiliar
        status = QStatusBar()
        status.showMessage(
            "Listo  |  Puerto serie: simulado (sin hardware)  |  "
            "dt = 50 ms  |  Practica 2 - Instrumentacion Virtual"
        )
        self.setStatusBar(status)

        # Inicio automatico de ambos lazos para que las graficas no esten
        # vacias al abrir la aplicacion (util para la captura de pantalla)
        self.temp_panel.start()
        self.motor_panel.start()


def main():
    """Punto de entrada de la aplicacion."""
    # Creacion de la aplicacion Qt con los argumentos de linea de comandos
    app = QApplication(sys.argv)
    # Estilo Fusion: aspecto neutro y consistente entre plataformas
    app.setStyle("Fusion")
    # Creacion y visualizacion de la ventana principal
    window = MainWindow()
    window.show()
    # Cesion del control al ciclo de eventos de Qt
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
