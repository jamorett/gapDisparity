# gapDisparity



# F1Tenth - Gap Triangle Navigation Node



Este repositorio contiene un nodo de navegación autónoma en ROS2 basado en el algoritmo de Gap Triangle (una mejora del enfoque Disparity Extender), adaptado específicamente para el simulador F1Tenth.



## 🚗 Descripción del enfoque utilizado



El nodo implementa una estrategia reactiva de conducción autónoma usando datos LiDAR. El flujo general es el siguiente:



1. Extensión de disparidades (Disparity Extender): crea burbujas de seguridad alrededor de obstáculos.

2. Selección de dirección óptima: se aplica un sesgo frontal y se elige el ángulo con mayor visibilidad.

3. Corrección de colisiones laterales: se ajusta el giro si se detectan objetos cercanos a los lados.

4. Cálculo de velocidad dinámica: la velocidad se ajusta en tiempo real según el entorno visible y el ángulo de giro.

5. Conteo de vueltas y cronómetro: se registra cada vuelta completada usando `/ego_racecar/odom`, con tiempos publicados en consola.



## 📁 Estructura del código



- `gap_triangle.py`: nodo ROS2 que realiza:

- Suscripción a `/scan` (sensor LiDAR)

- Suscripción a `/ego_racecar/odom` (posición actual del vehículo)

- Publicación en `/drive` (comandos de dirección y velocidad)

- Lógica de navegación, seguridad, y registro de vueltas



## ▶️ Instrucciones de ejecución



### Requisitos



- ROS 2 (recomendado Humble o Foxy)

- Simulador F1Tenth corriendo

- Dependencias:

  - `numpy`

  - `rclpy`

  - `ackermann_msgs`

  - `sensor_msgs`

  - `nav_msgs`



### Ejecución del nodo



1. Clona este repositorio en tu workspace de ROS2:




cd ~/ros2_ws/src

git clone https://github.com/jamorett/gapDisparity.git

cd ~/ros2_ws

colcon build

source install/setup.bash

## 📖 Explicación del código (gap_triangle.py)



El nodo gap_triangle.py implementa un controlador de navegación autónoma para F1Tenth basado en una extensión del algoritmo Disparity Extender. A continuación, se explica su estructura y lógica:

## 🧩 Estructura general



   Se suscribe a:



      /scan: datos del LiDAR.



       /ego_racecar/odom: posición del vehículo para contar vueltas.


   Publica comandos en /drive (velocidad y ángulo de dirección).



   Utiliza lógica de evasión de obstáculos, selección de ruta óptima, y control dinámico de velocidad.



## ⚙️ Parámetros ajustables



Están definidos al inicio e incluyen:



   Ancho del vehículo (CAR_WIDTH)



   Margen de seguridad alrededor del vehículo (SAFETY_MARGIN)



   Límites de velocidad, aceleración, frenado



   Resolución y campo de visión del LiDAR



   Umbral para detectar disparidades (DISPARITY_THRESHOLD)



   Constantes para suavizar el giro y detectar vueltas



## 🔄 scan_callback()



Función que se ejecuta en cada ciclo del LiDAR:



   Limpieza de datos: convierte a numpy y reemplaza inf por el rango máximo.



   Extensión de obstáculos: mediante disparity_extender(), se expande la zona de seguridad alrededor de cambios bruscos de distancia (bordes de obstáculos).



   Selección del objetivo: elige el ángulo con mayor apertura, priorizando los que están frente al auto usando un sesgo cos(θ).



   Corrección lateral: avoid_side_collision() reduce el giro si hay obstáculos muy cercanos a los lados.



   Suavizado: se aplica un filtro exponencial para evitar cambios bruscos en la dirección.



   Cálculo de velocidad: compute\_speed() ajusta la velocidad considerando:



       Distancia libre al frente



       Magnitud del giro



       Distancia necesaria para frenar



   Publicación: se envía un mensaje AckermannDriveStamped con la dirección y velocidad resultantes.



## 📍 odom_callback()



Detecta cuando el vehículo completa una vuelta:



   Registra la posición inicial (track_origin) al comenzar.



   Calcula la distancia del vehículo a esa posición.



   Si vuelve a acercarse (dentro del radio LAP_DETECTION_RADIUS) y ya ha salido una vez, cuenta una nueva vuelta.



   Publica en consola el número de vuelta y su tiempo.



   La primera vuelta no se cuenta para evitar falsos positivos tras iniciar el nodo.



## 📐 disparity_extender(ranges)



   Detecta bordes (disparidades) en las lecturas del LiDAR.



   Extiende el obstáculo hacia ambos lados en un rango proporcional al ángulo necesario para evitarlo.



   Aumenta la seguridad en curvas cerradas o zonas estrechas.



🧮 compute_speed(ranges, steer)



   Calcula la velocidad objetivo en función de:



       La distancia al obstáculo directamente al frente.



       El ángulo de giro (menor velocidad en curvas).



       La distancia requerida para frenar en seco (v <= sqrt(2·a·d)).



   Devuelve el valor mínimo entre lo deseado y lo seguro.



## 🚧 avoid_side_collision(ranges, angle)



   Evalúa obstáculos a 90° a cada lado.



Si el vehículo intenta girar hacia un lado donde hay peligro, reduce el ángulo para evitar la colisión.



## 📐 get_max_steering_angle()

   Retorna el ángulo máximo que cubre el LiDAR, usado para normalizar cálculos de giro.


