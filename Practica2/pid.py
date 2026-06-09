"""
Controlador PID discreto para la Practica 2 de Instrumentacion Virtual.

Implementa la ley de control del preinforme:
    u(t) = Kp*e(t) + Ki*integral(e) + Kd*de/dt
en su forma discreta, con tres anadidos practicos:
  - Saturacion de la salida al rango del ciclo de trabajo (0..100 %).
  - Anti-windup por integracion condicional (clamping): evita que el termino
    integral siga creciendo cuando la salida ya esta saturada.
  - Filtro pasa-bajos de primer orden sobre el termino derivativo, para
    atenuar la amplificacion de ruido (relevante con la medida de RPM, que
    es ruidosa). El preinforme advierte que Kd puede amplificar el ruido.

La clase no depende de ninguna libreria externa.
"""


class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0,
                 setpoint=0.0, out_min=0.0, out_max=100.0,
                 deriv_tau=0.05):
        # Ganancias del controlador
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # Referencia (consigna) deseada: TR en C o RPMR en RPM
        self.setpoint = setpoint
        # Limites de saturacion de la salida (ciclo de trabajo, en %)
        self.out_min = out_min
        self.out_max = out_max
        # Constante de tiempo del filtro derivativo, en segundos.
        # Cuanto mayor es deriv_tau, mas suave (mas filtrada) es la derivada.
        self.deriv_tau = deriv_tau
        # Inicializa el estado interno
        self.reset()

    def reset(self):
        """Reinicia el estado interno del controlador."""
        self._integral = 0.0         # Acumulador de la accion integral
        self._prev_error = 0.0       # Error de la iteracion anterior
        self._deriv_filtered = 0.0   # Valor filtrado del termino derivativo
        self._first_run = True       # Para no derivar en la primera muestra

    def compute(self, measurement, dt):
        """
        Calcula la senal de control para una nueva muestra.

        Parametros:
          measurement : variable medida (T en C, o velocidad en RPM).
          dt          : tiempo transcurrido desde la llamada anterior, en segundos.

        Devuelve una tupla (u, p, i, d):
          u : salida total ya saturada (ciclo de trabajo en %).
          p, i, d : las tres componentes por separado (utiles para graficar).
        """
        if dt <= 0.0:
            dt = 1e-3   # Proteccion ante un dt invalido (evita dividir por cero)

        # 1) Error actual: diferencia entre la referencia y la medida
        error = self.setpoint - measurement

        # 2) Accion proporcional: reacciona al error presente
        p = self.kp * error

        # 3) Accion integral: acumula el error a lo largo del tiempo
        self._integral += error * dt
        i = self.ki * self._integral

        # 4) Accion derivativa sobre el error, con filtro pasa-bajos
        if self._first_run:
            # En la primera muestra no hay error previo valido: derivada = 0
            raw_deriv = 0.0
            self._first_run = False
        else:
            raw_deriv = (error - self._prev_error) / dt
        # Filtro de primer orden discreto:
        #   alpha = dt / (tau + dt)   (entre 0 y 1)
        #   con tau grande -> alpha pequeno -> mas filtrado (mas suave)
        alpha = dt / (self.deriv_tau + dt)
        self._deriv_filtered += alpha * (raw_deriv - self._deriv_filtered)
        d = self.kd * self._deriv_filtered

        # 5) Salida sin saturar (suma de las tres acciones)
        u = p + i + d

        # 6) Saturacion con anti-windup por integracion condicional.
        #    Si la salida se sale del rango, la recortamos al limite y, si el
        #    error empuja en la misma direccion de la saturacion, deshacemos el
        #    ultimo incremento del integral para que no crezca sin control.
        if u > self.out_max:
            u = self.out_max
            if error > 0.0:
                self._integral -= error * dt
        elif u < self.out_min:
            u = self.out_min
            if error < 0.0:
                self._integral -= error * dt

        # 7) Guardamos el error actual para calcular la derivada en la siguiente llamada
        self._prev_error = error

        return u, p, i, d
