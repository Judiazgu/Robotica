# 🐢 Trazado de Letras con Turtlesim y ROS 2

## 📋 Descripción del Proyecto

Este proyecto permite controlar una tortuga virtual del simulador `turtlesim` en ROS 2 para dibujar letras en la pantalla. Usando comandos desde el teclado, el usuario puede ordenar al sistema que trace las letras **J**, **D** y **G** mediante movimientos programados. El nodo se ejecuta en ROS 2 utilizando `rclpy` (Python) y hace uso de publicaciones y servicios disponibles en el entorno de `turtlesim`.

---

## 🎯 Objetivos

- Aplicar los conceptos de publicación y servicios en ROS 2 usando `rclpy`.
- Utilizar `Twist` para enviar comandos de velocidad a la tortuga.
- Emplear los servicios `TeleportAbsolute` y `TeleportRelative` para ubicar la tortuga en posiciones y orientaciones específicas.
- Implementar control por teclado para activar el trazado de diferentes letras.

---

## ⚙️ Procedimiento Realizado

### Inicialización:

- Se crea un nodo `move_turtle` que publica en `/turtle1/cmd_vel`.
- Se definen clientes para los servicios:
  - `/clear` (limpiar pantalla)
  - `/turtle1/teleport_absolute` (teletransportación absoluta)
  - `/turtle1/teleport_relative` (teletransportación relativa)
- Se suscribe también al tópico `/turtle1/pose` para obtener la posición actual de la tortuga.

### Control por teclado:

- Se inicia un hilo que escucha continuamente la entrada del teclado.
- Al presionar:
  - `j`: se traza la letra **J**.
  - `d`: se traza la letra **D**.
  - `g`: se traza la letra **G**.

### Letras implementadas:

- **Letra J**:
  - Teletransportación inicial a una posición centrada.
  - Movimiento hacia abajo (línea vertical) mediante `teleport_relative`.
  - Giro para formar un arco hacia la izquierda simulando la curvatura de la "J".

- **Letra D**:
  - Movimiento vertical hacia la derecha usando `teleport_relative`.
  - Rotación usando `teleport_absolute` manteniendo la posición.
  - Movimiento horizontal (simulando la línea recta).
  - Arco derecho para cerrar la forma de "D".

- **Letra G**:
  - Arco izquierdo (semicírculo).
  - Línea vertical con orientación hacia arriba.
  - Línea horizontal final con orientación hacia la izquierda.

---

## 🚀 Cómo Ejecutar

1. Ejecuta el simulador turtlesim:

```bash
ros2 run turtlesim turtlesim_node
