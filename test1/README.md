Navegación Autónoma de TurtleBot3 mediante Deep Q-Network (DQN)

Este repositorio contiene la implementación de un sistema de navegación autónoma para el robot TurtleBot3 utilizando algoritmos de Aprendizaje por Refuerzo (DQN). El sistema permite al robot aprender a navegar hacia un objetivo en un entorno simulado de Gazebo, procesando datos de sensores y tomando decisiones de movimiento en tiempo real.
Descripción de los Componentes del Proyecto

El proyecto se estructura en los siguientes módulos clave:
○ dqn_agent.py - Implementación del Agente DQN

Define el "cerebro" del robot. Contiene la clase DQNAgent, la red neuronal (Perceptrón Multicapa) y la lógica de aprendizaje (memoria de repetición y política Epsilon-Greedy).
○ environment.py - Wrapper del Entorno ROS2

Actúa como interfaz entre el algoritmo y el simulador Gazebo. Gestiona la suscripción a sensores (/scan, /odom), publica comandos de velocidad (/cmd_vel) y calcula las recompensas basadas en el progreso de distancia.
○ state_processor.py - Procesamiento de LiDAR

Se encarga de preprocesar los datos crudos del láser. Discretiza los 360 grados del escáner en sectores reducidos y normaliza las distancias para facilitar el aprendizaje de la red neuronal.
○ train_node.py - Nodo de Entrenamiento

Script principal para la fase de aprendizaje. Ejecuta un bucle de episodios (por defecto 300-500), gestiona el reinicio del entorno y coordina el guardado periódico del modelo.
○ test_node.py - Nodo de Evaluación

Script para validar el rendimiento. Carga el modelo entrenado, desactiva la exploración aleatoria y ejecuta una serie de pruebas para medir la tasa de éxito al llegar a la meta.
○ Modelo entrenado (trained_model.pkl)

Archivo binario que contiene los pesos y la configuración de la red neuronal una vez finalizado el entrenamiento. Es indispensable para ejecutar el nodo de evaluación.
Instrucciones de Ejecución Paso a Paso

Siga estos pasos en orden estricto para garantizar el funcionamiento correcto del sistema.
Requisitos Previos

    Tener instalado ROS 2 Humble y Gazebo.

    Instalar librerías de Python necesarias:
    Bash

    pip3 install scikit-learn numpy

PASO 1: Compilación del Proyecto

Antes de ejecutar nada, debe compilar el paquete para que ROS2 reconozca los scripts.

    Abra una terminal en la raíz de su espacio de trabajo.

    Ejecute los siguientes comandos:
    Bash

    cd ~/dqn_ws
    colcon build
    source install/setup.bash

PASO 2: Fase de Entrenamiento (Training)

En esta fase, el robot aprenderá desde cero. Nota: Si existe un archivo trained_model.pkl antiguo, se recomienda borrarlo antes de empezar para evitar conflictos.

    Limpiar procesos en segundo plano: Ejecute este comando para asegurar que no haya simulaciones antiguas consumiendo memoria.
    Bash

killall -9 gzserver gzclient rosmaster

Lanzar la simulación (Terminal 1):
Bash

export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

Ejecutar el nodo de entrenamiento (Terminal 2): Abra una nueva terminal y ejecute:
Bash

    cd ~/dqn_ws
    source install/setup.bash
    ros2 run tb3_dqn train

    Deje correr el entrenamiento hasta que se completen los episodios o deténgalo manualmente con Ctrl+C cuando el robot muestre un buen comportamiento.

PASO 3: Fase de Evaluación (Test)

Una vez generado el archivo trained_model.pkl, proceda a evaluar el rendimiento.

    Reiniciar el entorno: Cierre todas las terminales y ventanas de Gazebo, y ejecute nuevamente la limpieza:
    Bash

killall -9 gzserver gzclient

Lanzar la simulación limpia (Terminal 1):
Bash

export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

Ejecutar el nodo de prueba (Terminal 2):
Bash

    cd ~/dqn_ws
    source install/setup.bash
    ros2 run tb3_dqn test

Resultados Esperados en el Test

El sistema ejecutará 20 pruebas consecutivas hacia objetivos predefinidos.

    Éxito: El robot llega al objetivo (distancia < 0.50m).

    Fallo: El robot choca o se agota el tiempo (timeout).

    Al finalizar, se mostrará en consola la Tasa de Éxito (%).
