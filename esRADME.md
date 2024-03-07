# Péndulo Invertido
Este proyecto consiste en un sistema de control de péndulo invertido utilizando un sensor MPU6050, un microcontrolador ESP32 y dos motores DC. El objetivo es mantener el péndulo en posición vertical mediante el control de la velocidad de los motores.

[Vídeo1](https://youtu.be/fJx0cJZ3p6E)
[Vídeo2](https://youtu.be/VGf259euCRA)

## Funcionamiento
El código controla el péndulo invertido de la siguiente manera:

1. **Inicialización del Sensor y Motores**: En la función `setup()`, se inicializa la comunicación serial, el sensor MPU6050 y se configuran los pines de los motores.

2. **Lectura de Datos del MPU6050**: En el bucle `loop()`, se leen los datos del acelerómetro y el giroscopio del MPU6050 para calcular los ángulos de inclinación del péndulo tanto en el eje X como en el eje Y.

3. **Control del Péndulo Invertido**: Se calcula el error entre el ángulo deseado y el ángulo actual del péndulo. Luego, se utiliza un controlador PID (Proporcional-Integral-Derivativo) para determinar la velocidad de los motores necesaria para corregir este error.

4. **Control de los Motores**: Se ajusta la velocidad de los motores según la salida del controlador PID para mantener el péndulo en posición vertical.

5. **Visualización de Datos**: Se imprimen los datos relevantes, como la rotación en el eje X, el error y la velocidad de los motores, a través de la comunicación serial para su visualización en un monitor serial.

## BOM
Tabla de materiales utilizados en el proyecto.
| Componente | Cantidad |
|------------|----------|
| ESP32 Dev Board | 1 |
| Sensor MPU6050 | 1 |
| Motor DC | 2 |
| TB6612FNG Motor Driver | 1 |
| Switch de encendido | 2 |
| Condensador electrolítico 330uF 16V | 1 |
| Condensador cerámico 100nF | 2 |
| TO-220 L7805CV Regulador de voltaje | 1 |
| Conector JST-HX hembra 2 pines | 2 |
| Conector JST-HX macho 2 pines | 2 |
| Cables de conexión | - |
| Ruedas para motores DC | 2 |
| Lipo Battery 11.1V 1500mAh | 1 |
| Conector XT60 hembra | 1 |
| PCB | 1 |
| Chasis | 1 |
| Tornillos m3 8mm | 4 |
| Tornillos m3 25mm | 4 |
| Cintas sujección LiPo | 2 |

## Ajustes y Personalización
- Es importante calibrar el sensor MPU6050 para obtener mediciones precisas. La explicación detallada de cómo calibrar el sensor se puede encontrar en el siguiente [enlace](https://naylampmechatronics.com/blog/45_tutorial-mpu6050-acelerometro-y-giroscopio.html).
- Los parámetros del controlador PID (Kp, Ki, Kd) se deben ajustar para que el robot se mantenga estable, en el siguiente [enlace](https://www.luisllamas.es/como-ajustar-un-controlador-pid-en-arduino/) se puede encontrar una guía para ajustar los parámetros del controlador PID.

## PCB
El diseño de la PCB se realizó en KiCad. Los archivos de fabricación de la PCB se encuentran en la carpeta `PCB`. Se utilizó el servicio de JLCPCB para la fabricación de la PCB.