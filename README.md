# gapDisparity



\# F1Tenth - Gap Triangle Navigation Node



Este repositorio contiene un nodo de navegación autónoma en ROS2 basado en el algoritmo de \*\*Gap Triangle\*\* (una mejora del enfoque Disparity Extender), adaptado específicamente para el simulador \*\*F1Tenth\*\*.



\## 🚗 Descripción del enfoque utilizado



El nodo implementa una estrategia reactiva de conducción autónoma usando datos LiDAR. El flujo general es el siguiente:



1\. \*\*Extensión de disparidades (Disparity Extender):\*\* crea burbujas de seguridad alrededor de obstáculos.

2\. \*\*Selección de dirección óptima:\*\* se aplica un sesgo frontal y se elige el ángulo con mayor visibilidad.

3\. \*\*Corrección de colisiones laterales:\*\* se ajusta el giro si se detectan objetos cercanos a los lados.

4\. \*\*Cálculo de velocidad dinámica:\*\* la velocidad se ajusta en tiempo real según el entorno visible y el ángulo de giro.

5\. \*\*Conteo de vueltas y cronómetro:\*\* se registra cada vuelta completada usando `/ego\\\_racecar/odom`, con tiempos publicados en consola.



\## 📁 Estructura del código



\- `gap\\\_triangle.py`: nodo ROS2 que realiza:

  - Suscripción a `/scan` (sensor LiDAR)

  - Suscripción a `/ego\\\_racecar/odom` (posición actual del vehículo)

  - Publicación en `/drive` (comandos de dirección y velocidad)

  - Lógica de navegación, seguridad, y registro de vueltas



\## ▶️ Instrucciones de ejecución



\### Requisitos



\- ROS 2 (recomendado Humble o Foxy)

\- Simulador F1Tenth corriendo

\- Dependencias:

  - `numpy`

  - `rclpy`

  - `ackermann\\\_msgs`

  - `sensor\\\_msgs`

  - `nav\\\_msgs`



\### Ejecución del nodo



1\. Clona este repositorio en tu workspace de ROS2:



```bash

cd ~/ros2\\\_ws/src

git clone https://github.com/jamorett/gapDisparity.git

cd ~/ros2\\\_ws

colcon build

source install/setup.bash

📖 Explicación del código (gap\_triangle.py)



El nodo gap\_triangle.py implementa un controlador de navegación autónoma para F1Tenth basado en una extensión del algoritmo Disparity Extender. A continuación, se explica su estructura y lógica:

🧩 Estructura general



&nbsp;   Se suscribe a:



&nbsp;       /scan: datos del LiDAR.



&nbsp;       /ego\_racecar/odom: posición del vehículo para contar vueltas.



&nbsp;   Publica comandos en /drive (velocidad y ángulo de dirección).



&nbsp;   Utiliza lógica de evasión de obstáculos, selección de ruta óptima, y control dinámico de velocidad.



⚙️ Parámetros ajustables



Están definidos al inicio e incluyen:



&nbsp;   Ancho del vehículo (CAR\_WIDTH)



&nbsp;   Margen de seguridad alrededor del vehículo (SAFETY\_MARGIN)



&nbsp;   Límites de velocidad, aceleración, frenado



&nbsp;   Resolución y campo de visión del LiDAR



&nbsp;   Umbral para detectar disparidades (DISPARITY\_THRESHOLD)



&nbsp;   Constantes para suavizar el giro y detectar vueltas



🔄 scan\_callback()



Función que se ejecuta en cada ciclo del LiDAR:



&nbsp;   Limpieza de datos: convierte a numpy y reemplaza inf por el rango máximo.



&nbsp;   Extensión de obstáculos: mediante disparity\_extender(), se expande la zona de seguridad alrededor de cambios bruscos de distancia (bordes de obstáculos).



&nbsp;   Selección del objetivo: elige el ángulo con mayor apertura, priorizando los que están frente al auto usando un sesgo cos(θ).



&nbsp;   Corrección lateral: avoid\_side\_collision() reduce el giro si hay obstáculos muy cercanos a los lados.



&nbsp;   Suavizado: se aplica un filtro exponencial para evitar cambios bruscos en la dirección.



&nbsp;   Cálculo de velocidad: compute\_speed() ajusta la velocidad considerando:



&nbsp;       Distancia libre al frente



&nbsp;       Magnitud del giro



&nbsp;       Distancia necesaria para frenar



&nbsp;   Publicación: se envía un mensaje AckermannDriveStamped con la dirección y velocidad resultantes.



📍 odom\_callback()



Detecta cuando el vehículo completa una vuelta:



&nbsp;   Registra la posición inicial (track\_origin) al comenzar.



&nbsp;   Calcula la distancia del vehículo a esa posición.



&nbsp;   Si vuelve a acercarse (dentro del radio LAP\_DETECTION\_RADIUS) y ya ha salido una vez, cuenta una nueva vuelta.



&nbsp;   Publica en consola el número de vuelta y su tiempo.



&nbsp;   La primera vuelta no se cuenta para evitar falsos positivos tras iniciar el nodo.



📐 disparity\_extender(ranges)



&nbsp;   Detecta bordes (disparidades) en las lecturas del LiDAR.



&nbsp;   Extiende el obstáculo hacia ambos lados en un rango proporcional al ángulo necesario para evitarlo.



&nbsp;   Aumenta la seguridad en curvas cerradas o zonas estrechas.



🧮 compute\_speed(ranges, steer)



&nbsp;   Calcula la velocidad objetivo en función de:



&nbsp;       La distancia al obstáculo directamente al frente.



&nbsp;       El ángulo de giro (menor velocidad en curvas).



&nbsp;       La distancia requerida para frenar en seco (v <= sqrt(2·a·d)).



&nbsp;   Devuelve el valor mínimo entre lo deseado y lo seguro.



🚧 avoid\_side\_collision(ranges, angle)



&nbsp;   Evalúa obstáculos a 90° a cada lado.



&nbsp;   Si el vehículo intenta girar hacia un lado donde hay peligro, reduce el ángulo para evitar la colisión.



📐 get\_max\_steering\_angle()



&nbsp;   Retorna el ángulo máximo que cubre el LiDAR, usado para normalizar cálculos de giro.


