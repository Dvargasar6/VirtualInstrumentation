# Contexto del proyecto — Práctica 2 de Instrumentación Virtual

Documento de traspaso para continuar el desarrollo en una nueva conversación.
Adjunta también los dos PDF iniciales: la guía del profesor (`Practica2_VIR.pdf`)
y el preinforme (`Practica_2_Instrumentacion_Virtual.pdf`).

## Resumen del proyecto

Se implementan dos lazos de control PID con una misma interfaz: control de
**temperatura** de una resistencia de potencia (rango 30–35 °C, sensor LM35) y
control de **velocidad** de un micromotor DC N20 con encoder de cuadratura
(driver DRV8833). El sistema a controlar se elige desde la GUI. Solo se opera
un subsistema a la vez.

## Arquitectura del software

El PID y la interfaz corren en el PC; el ESP32 actúa únicamente como capa de
entrada/salida (genera PWM, lee LM35 por ADC, cuenta flancos del encoder).
Comunicación PC ↔ ESP32 por USB/serie a 115200 baudios, protocolo de texto.

Cuatro piezas de software:

- **`firmware_pid_esp32/firmware_pid_esp32.ino`**: firmware del ESP32, núcleo
  arduino-esp32 3.x (API LEDC nueva: `ledcAttach` + `ledcWrite(pin, cuentas)`).
  Genera PWM (15 Hz para temperatura, 10 kHz para motor), lee LM35 con
  `analogReadMilliVolts` y promediado de 16 muestras, cuenta flancos del canal A
  del encoder por interrupción y emite telemetría cada 20 ms.
- **`pid.py`**: clase `PID` discreta con saturación a 0–100 %, anti-windup por
  integración condicional y filtro pasa-bajos de primer orden sobre la derivada.
  Sin dependencias externas.
- **`serial_link.py`**: enlace serie con pyserial. Hilo lector en segundo plano
  que parsea la telemetría y guarda la última lectura de forma segura entre
  hilos. Función `listar_puertos` para enumerar puertos.
- **`gui.py`**: interfaz PyQt6 + PyQtGraph (punto de entrada: `python gui.py`).
  Contiene el bucle de control: cada Ts toma la última medida del enlace, la
  pasa al PID y envía el ciclo de trabajo al ESP32. Cuatro gráficas en vivo
  (medida vs referencia, PWM, error, componentes P/I/D), exportación a CSV.

### Protocolo serie (115200 baudios)

PC → ESP32, terminado en `\n`:
```
MODE,T | MODE,V | DUTY,xx | STOP | RESET | PING
```
ESP32 → PC:
```
T,<°C>,<duty> | V,<rpm>,<duty> | READY | PONG
```

## Hardware y asignación de pines del ESP32

| Pin     | Conexión                                       | Subsistema  |
|---------|------------------------------------------------|-------------|
| GPIO34  | Salida del LM35 (ADC)                          | Temperatura |
| GPIO25  | Resistencia 1 kΩ → Gate del MOSFET             | Temperatura |
| GPIO26  | IN1 del DRV8833 (PWM motor)                    | Motor       |
| GPIO27  | IN2 del DRV8833 (dirección)                    | Motor       |
| GPIO32  | Canal A del encoder (interrupción, INPUT_PULLUP) | Motor     |
| GPIO33  | Canal B del encoder (reservado, sin uso)       | Motor       |
| 5V      | LM35 (positivo)                                | Temperatura |
| 3.3V    | (libre, candidato para encoder)                | Motor       |

## Estado actual del trabajo

### Subsistema de temperatura: COMPLETO y funcionando

- **MOSFET**: IRLZ44N (canal N de nivel lógico) en lugar del TIP122 original.
  Funciona como low-side switch. Gate (pin 1) por R = 1 kΩ a GPIO25.
  Drain (pin 2) a un extremo de la resistencia. Source (pin 3) a tierra común.
  Pull-down de 10 kΩ entre Gate y Source.
- **Resistencia de potencia**: 10 Ω, 5 W (en lugar de los 22 Ω/3 W del preinforme).
- **Alimentación V2**: 4 pilas AA en serie, ≈ 6,4 V circuito abierto, ≈ 5,46 V
  bajo carga a D = 100 % (R_int ≈ 1,5 Ω). Potencia útil ≈ 2,98 W a D = 100 %.
- **LM35**: alimentado desde el pin 5 V del ESP32, Vout a GPIO34, GND común.
  Lecturas correctas verificadas con multímetro y monitor serie.
- **R_DS,on real medido del MOSFET**: ≈ 21 mΩ (consistente con la hoja de datos).
  Hubo un episodio inicial de caídas parásitas de ~1 V en cables/protoboard
  que se resolvió usando cable más grueso directo a los terminales.
- **Verificación en lazo abierto**:
  - D = 50 %: subió de 29 °C a 51 °C en 6 minutos sin estabilizarse (planta
    sobredimensionada para el rango objetivo).
  - Conclusión: el ciclo de trabajo estacionario que mantendrá la temperatura
    en 30–35 °C estará entre 5 % y 15 %. Falta caracterizar formalmente.
- **GUI ejecutándose con sobreimpulso**: usuario lo dejará para sintonización al final.

### Subsistema de velocidad: POR MONTAR (siguiente paso)

Pendiente todo el montaje físico y verificación. El firmware ya soporta el
modo motor sin cambios necesarios (excepto calibración de `PULSES_PER_REV`).

## Sintonización del PID: APLAZADA al final

Acordado con el usuario que la sintonización (tanto de temperatura como de
motor) se hará al final del proyecto. Plan documentado:

- Caracterizar la planta en lazo abierto (K, τ, L con escalón a ciclo bajo).
- Aplicar Ziegler-Nichols por curva de reacción (PI recomendado, Kd ≈ 0 en
  sistema térmico).
- Dividir K_p resultante entre 2 para reducir sobreimpulso (regla práctica).
- Refinamiento manual con las gráficas P/I/D de la GUI.
- Para el motor: PULSES_PER_REV en el firmware debe calibrarse experimentalmente
  girando el eje exactamente 10 vueltas y contando pulsos. El valor por defecto
  (350) es solo un punto de partida típico para un N20 de 250 RPM.

## Decisiones de alimentación acordadas

| Subsistema     | Alimentación                                            |
|----------------|---------------------------------------------------------|
| ESP32          | USB del PC                                              |
| LM35           | Pin 5 V del ESP32                                       |
| Resistencia    | 4 pilas AA en serie (V2 ≈ 6,4 V)                        |
| Motor + encoder| **Por decidir** (ver sección siguiente)                 |

**Fuente de protoboard no usable**: la salida de "5 V" entrega ≈ 9 V (regulador
dañado). La salida de 3,3 V debe verificarse con multímetro antes de usarla;
si tampoco está bien, no usar la fuente de protoboard para nada.

## Próximo paso: montaje del circuito del motor

Esto es lo que se debe retomar al iniciar el nuevo chat.

### Datos del motor (del fabricante)

Motor N20 con encoder, 3 V, 250 RPM. **Atención**: el código de colores no es
el estándar común. La info del fabricante manda:

| Color   | Función                       | Conecta a              |
|---------|-------------------------------|------------------------|
| Rojo    | Motor +                       | OUT1 del DRV8833       |
| Blanco  | Motor −                       | OUT2 del DRV8833       |
| Negro   | Encoder VCC (+)               | 3,3 V del ESP32        |
| Azul    | Encoder GND (−)               | GND común              |
| Amarillo| Encoder canal A               | GPIO32 del ESP32       |
| Verde   | Encoder canal B               | GPIO33 del ESP32       |

El encoder entrega **7 señales por vuelta del eje del motor** (antes de la
reducción). Asumiendo relación de reducción típica ≈ 50:1 para 250 RPM, salen
≈ 350 pulsos por vuelta del eje de salida. Calibrar experimentalmente.

**Advertencia crítica del fabricante**: el cable Negro NO es el negativo del
motor, es la alimentación positiva del encoder. Confundirlo destruye el
encoder. Verificar con multímetro la continuidad entre Negro/Azul y el cuerpo
del encoder antes de energizar.

### Decisión pendiente de alimentación del motor

El usuario tiene dos opciones razonables, debe elegir al inicio del nuevo chat:

1. **Pilas AA dedicadas para el motor** (recomendado): otro juego de 2 pilas AA
   (≈ 3 V, nominal del N20) o reutilizar el juego de 4 pilas (6,4 V, pero
   limitando `out_max` del PID del motor a 50 % para no superar 3,2 V efectivos).
2. **Pin 5 V del ESP32 compartido con el LM35**: viable porque solo se opera
   un subsistema a la vez. Exige condensador de 100 µF entre VM y GND del
   DRV8833 (ya planeado). Ruido potencial sobre el LM35 cuando el motor está
   activo, pero no es crítico porque no se controlan ambos a la vez.

### Pasos a seguir en el nuevo chat

1. **Confirmar alimentación del motor** (preguntar al usuario qué pilas tiene
   disponibles y verificar con multímetro la salida de 3,3 V de la fuente de
   protoboard, por si se puede recuperar).
2. **Cableado del DRV8833**:
   - VM (o VCC) → positivo de la fuente del motor.
   - GND → tierra común (ESP32 + fuente del motor + encoder).
   - IN1 → GPIO26; IN2 → GPIO27.
   - OUT1 → cable rojo del motor; OUT2 → cable blanco.
   - Condensador de 100 µF electrolítico entre VM y GND, lo más cerca del IC.
   - Si la placa trae pin EEP/SLEEP/STBY, conectarlo a 3,3 V (HIGH = activo).
3. **Cableado del encoder**:
   - Negro → 3,3 V del ESP32 (probar primero con 3,3 V; subir a 5 V solo si
     no entrega pulsos limpios).
   - Azul → GND común.
   - Amarillo → GPIO32 (canal A).
   - Verde → GPIO33 (canal B, reservado por ahora).
4. **Verificación en frío** (ESP32 desconectado, fuente del motor apagada):
   - GND común entre todos los nodos: continuidad ≈ 0 Ω.
   - OUT1–OUT2 sin cortocircuito a GND.
   - No hay cortos entre pistas.
5. **Verificación del encoder solo**, sin alimentar el motor: ESP32 + encoder
   energizados, motor sin alimentación. Girar el eje a mano y verificar con
   multímetro o con el monitor serie en modo `MODE,V` que el contador responde.
6. **Prueba en bucle abierto del motor**: `MODE,V` + `DUTY,30` desde el monitor
   serie. El motor debe arrancar. Verificar que la telemetría cambia a formato
   `V,<rpm>,<duty>`.
7. **Calibración de PULSES_PER_REV**: con el motor sin alimentar, girar el
   eje de salida exactamente 10 vueltas a mano contando pulsos reportados.
   Dividir entre 10. Actualizar `PULSES_PER_REV` en el firmware y recompilar.
8. **Pruebas en lazo abierto a varios ciclos de trabajo** para caracterizar el
   rango útil del motor.
9. Cuando el motor esté validado en lazo abierto, queda pendiente solo la
   **sintonización del PID** (tanto motor como temperatura).

## Estilo de respuesta esperado por el usuario

- Lenguaje formal, serio, técnico, sin condescendencia. Sin emojis.
- Si el usuario se equivoca, decirlo claramente sin suavizarlo.
- Sistema operativo Arch Linux. Comandos y particularidades enfocadas a Arch
  (PEP 668 / entornos virtuales, grupo `uucp` para puertos serie, módulos
  `cp210x`/`ch341` del kernel, AUR para arduino-cli).
- Procesos multi-paso: explicar y entregar solo el primer paso, avanzar al
  siguiente cuando el usuario confirme que el anterior funcionó.
- Código siempre con comentarios línea por línea, sin emojis. El usuario es
  estudiante y aprende con los comentarios.

## Archivos del proyecto en el sistema del usuario

```
Practica2/
├── firmware_pid_esp32/
│   └── firmware_pid_esp32.ino
├── pid.py
├── serial_link.py
├── gui.py
├── requirements.txt
└── README.md
```

Entorno virtual de Python: `pid_env` (visto en el prompt del usuario:
`(pid_env) daniel68@Tuxif:~/Development/VirtualInstrumentation/Practica2$`).
Puerto serie del ESP32: `/dev/ttyUSB0`.
