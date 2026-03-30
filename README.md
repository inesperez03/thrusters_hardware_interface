# catamaran_hardware_interface

## Descripción general

`catamaran_hardware_interface` implementa la interfaz hardware del catamarán usando `ros2_control`.

Su función es recibir comandos de fuerza para los dos thrusters del robot a través de las interfaces de comando de `ros2_control` y traducirlos a la salida correspondiente según el entorno de ejecución:

- **modo real**: conversión de fuerza a **PWM**
- **modo simulación**: conversión de fuerza a **valores float para Stonefish**

La interfaz hardware está implementada como un plugin de tipo `SystemInterface`.

---

## Objetivo del paquete

Este paquete actúa como la capa que conecta el sistema de control con los actuadores del catamarán.

La entrada al hardware es una referencia de **fuerza en newtons** para cada thruster. Esa fuerza se recibe a través de las interfaces:

- `left_thruster_joint/effort`
- `right_thruster_joint/effort`

---

## Conversión de fuerza

La conversión entre fuerza y salida del actuador se realiza mediante una tabla de calibración almacenada en:

`config/thrusters_lookup.csv`

Esta tabla contiene la relación entre:

- fuerza en newtons
- PWM
- valor equivalente para Stonefish

Los valores de referencia utilizados para construir esta tabla se han tomado de la caracterización del **thruster T500 de Blue Robotics** a **22 V**. :contentReference[oaicite:1]{index=1}

La clase `ThrusterMapper` se encarga de:

- cargar el CSV,
- ordenar las muestras,
- interpolar entre puntos,
- devolver:
  - PWM en modo real,
  - float Stonefish en modo simulación.

## Arquitectura

La arquitectura actual del hardware interface es la siguiente:

```text
controller_manager
    ↓
controlador
    ↓
left_thruster_joint/effort
right_thruster_joint/effort
    ↓
CatamaranSystem
    ├── entorno real -> fuerza a PWM
    └── entorno sim  -> fuerza a valores Stonefish


NOTA: Actualmente solo funciona en simulador, aun no publica valores PWM