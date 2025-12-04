## README – Examen Práctico Robótica IMT-342

Este repositorio contiene los desarrollos correspondientes al examen práctico del curso de Robótica IMT-342.
Incluye la implementación de navegación autónoma mediante Deep Q-Network (DQN), control visual mediante gestos corporales usando ROS2 + micro-ROS, y un espacio reservado para el ejercicio extra de procesamiento de imágenes de profundidad.
Los ejercicios deben ser ejecutados en ROS2 Humble y Gazebo.

===>
MIEMBROS
========

```
• Adrian Fuentes  
• Alejandro Miranda  
• Paolo Quisbert  
• Luz Soria  
```

===>
EJERCICIO 1 – Navegación Autónoma con DQN
=========================================

Componentes del proyecto
El workspace contiene:
- dqn_agent.py
- environment.py
- state_processor.py
- train_node.py
- test_node.py
- trained_model.pkl

Preparación previa
Instalar dependencias:
pip3 install scikit-learn numpy

Compilación del workspace
Ruta recomendada: ~/dqn_ws

```
	cd ~/dqn_ws  
	colcon build  
	source install/setup.bash  
```

Entrenamiento (Training)

```
a) Limpieza de procesos previos:  
	killall -9 gzserver gzclient rosmaster  

b) Lanzar simulación (Terminal 1):  
	export TURTLEBOT3_MODEL=burger  
	ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py  

c) Nodo de entrenamiento (Terminal 2):  
	cd ~/dqn_ws  
	source install/setup.bash  
	ros2 run tb3_dqn train  

El entrenamiento debe ejecutarse por al menos 200 episodios.
```

Evaluación (Testing)

```
a) Cerrar Gazebo y limpiar:  
	killall -9 gzserver gzclient  

b) Lanzar simulación limpia (Terminal 1):  
	export TURTLEBOT3_MODEL=burger  
	ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py  

c) Nodo de evaluación (Terminal 2):  
	cd ~/dqn_ws  
	source install/setup.bash  
	ros2 run tb3_dqn test  

El sistema ejecutará 20 pruebas consecutivas y mostrará la tasa de éxito.
```

===>
EJERCICIO 2 – Control Gestual con ROS2, Kinect y micro-ROS
==========================================================

Descarga y preparación de archivos

```
a) Código para ESP32  
	Abrir ~/…/test2/test_ej2 y cargar el archivo en la placa.  
	Las conexiones están descritas dentro del código.

b) Datos necesarios  
	Descargar desde:  
	https://drive.google.com/drive/folders/1W4o1lLWoxtzip3PO2Qi9y1yUasqX7DO7?usp=sharing

	Colocar en la ruta:  
		ej2/kinect_data2

	Nombres exactos:  
		- kinect_data2_0.db3  
		- metadata.yaml  

	Si no coinciden, renombrarlos.
```

Ejecución del sistema (cuatro terminales)

```
Terminal 1 – Nodo de gestos  
	Ruta: ~/…/test2/ej2/ros2_ws  
		colcon build --packages-select gesture_control  
		source install/setup.bash  
		ros2 run gesture_control gestures_node  

Terminal 2 – Reproducción del rosbag  
		source /opt/ros/humble/setup.bash  
		ros2 bag play kinect_data2  

Terminal 3 – micro-ROS Agent  
	Si la placa no es reconocida, desconectarla 3 segundos y reconectar.  
		docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 115200 -v6  

Terminal 4 – Gazebo  
		ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py  
```

===>
EJERCICIO EXTRA – Procesamiento de Imagen de Profundidad
========================================================

```
(Espacio reservado para implementación futura.)
```

---


