# Especificación de Desarrollo: Interfaz de Control PID (Instrumentación Virtual)

## 1. Contexto del Proyecto
El objetivo es desarrollar una interfaz de usuario en Python para la implementación empírica de dos sistemas de control en lazo cerrado mediante controladores Proporcional-Integral-Derivativo (PID). Las señales de control (PWM) y la adquisición de datos se gestionarán a través de una DAQ o un microcontrolador (por ejemplo, Arduino).

El entorno de ejecución y desarrollo principal es **Arch Linux**. Se deben tener en cuenta las particularidades de este sistema operativo (gestión de permisos de puertos seriales como `/dev/ttyACM0` o `/dev/ttyUSB0`, y el uso de entornos virtuales de Python o dependencias instaladas vía `pacman`).

## 2. Descripción de los Sistemas Físicos

### 2.1. Sistema Térmico (Control de Temperatura)
* **Planta:** Resistencia de potencia calefactora (mayor a 3 W, aprox. 22 ohmios).
* **Sensor:** Sensor de temperatura LM35 (salida lineal $10mV/^{\circ}C$) o Termistor NTC (requiere linealización vía ecuación de Steinhart-Hart).
* **Actuador/Potencia:** Transistor Darlington TIP122 o MOSFET de nivel lógico (ej. IRLZ44N).
* **Rango de Operación:** $30^{\circ}C$ a $35^{\circ}C$.
* **Frecuencia PWM:** Baja (10 Hz a 20 Hz debido a la alta constante de tiempo térmica).

### 2.2. Sistema Electromecánico (Control de Velocidad)
* **Planta:** Micromotor DC N20 (3V, 250 RPM).
* **Sensor:** Encoder de cuadratura integrado en el motor.
* **Actuador/Potencia:** Driver de puente H DRV8833.
* **Rango de Operación:** Velocidad en RPM.
* **Frecuencia PWM:** Alta (1 kHz a 20 kHz para promediar corriente en el devanado y evitar ruido audible).

## 3. Requerimientos de Software (Interfaz Gráfica)

La aplicación en Python debe cumplir rigurosamente con los siguientes requisitos:

1.  **Selección de Sistema:** Permitir al usuario alternar entre el control del sistema térmico y el electromecánico.
2.  **Configuración de Referencias:** Entradas para definir el *Set-Point* (temperatura objetivo o RPM objetivo).
3.  **Sintonización PID:** Entradas numéricas para modificar en tiempo real las constantes $K_p$, $K_i$ y $K_d$.
4.  **Visualización en Tiempo Real:** * Gráfica 1: Señal medida frente a la señal de referencia a lo largo del tiempo.
    * Gráfica 2: Ciclo de trabajo (Duty Cycle) de la señal PWM generada por el controlador.
5.  **Adquisición y Control:** Comunicación eficiente y sin bloqueos (non-blocking) a través del puerto serie o la API de la DAQ.
6.  **Registro de Datos (Datalogging):** Capacidad para exportar los datos experimentales (tiempo, referencia, medición, acción de control) en formato CSV para análisis posterior (tiempo de respuesta, sobreimpulso, error en estado estacionario).
7.  **Seguridad:** Botones de inicio, parada, y una parada de emergencia que envíe inmediatamente un ciclo de trabajo de 0% a los actuadores.

## 4. Instrucciones de Actuación para el Agente de Código

Al procesar este documento y generar el código asociado, debes adherirte a las siguientes directrices:

* **Lenguaje Formal y Técnico:** Utiliza un tono serio, prescinde de emojis y evita cualquier lenguaje condescendiente. 
* **Comentarios de Código:** El código generado debe estar documentado extensamente de manera interna. Debes explicar la lógica subyacente y la función de las bibliotecas o métodos específicos utilizados (ej. manejo de colas para comunicación serial en hilos separados, actualización de gráficos con PyQtGraph o Matplotlib).
* **Modularidad:** Divide el proyecto en componentes lógicos (comunicación serial/DAQ, lógica PID, interfaz gráfica).
* **Consideraciones Arch Linux:** Asegúrate de sugerir la adición del usuario al grupo `uucp` o `dialout` (según corresponda, típicamente `uucp` en Arch) para evitar problemas de permisos con el hardware serie. Proporciona comandos pertinentes si es necesario, priorizando la instalación de dependencias vía `pacman` cuando estén disponibles en los repositorios oficiales, o indicando la creación estricta de un entorno virtual (`python -m venv`).

## 5. Próximo Paso
Procede únicamente con la estructura inicial del proyecto y la clase encargada de la comunicación serial/adquisición de datos. No generes toda la interfaz gráfica de una vez; avanza de manera iterativa tras la validación de la capa de acceso a hardware.
