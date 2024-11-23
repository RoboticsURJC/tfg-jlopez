# Versión en español

## Introducción 
Bienvenido/a al repo del TFG (Trabajo Fin de Carrera) de Julia López Augusto. En él podrás encontrar todo el código y ficheros correspondientes y podrás ver un seguimiento continuo en el apartado de [Wiki](https://github.com/RoboticsURJC/tfg-jlopez/wiki). Si deseas replicar este proyecto, te recomiendo seguir los siguientes pasos: 

### Construcción del robot
Para poder construir este robot, se han proporcionado una serie de ficheros .stl  que se pueden imprimir en cualquier impresora 3D convencional. Para ello, es necesario imprimir una pieza de cada una de las siguientes: 

- Chasis
- Soporte de la cámara
- Carcasa
- Sujección trasera

Estas son las características de impresión: 


Esta es la tornillería usada: 


Para el ensamblaje, se ha creado este [fichero]() que muestra paso a paso el ensamblaje completo del robot. Finalmente el robot tiene que quedar así: 

(Incluir imagen)

### Soporte software 
A continuación, se van a explicar la implementación software desarrollada para el robot: 

#### Simulación 

Primero de todo, hay que instalar los siguientes programas:
```bash
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-rviz2
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-xacro ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
```
Una vez instalados, únicamente hay que ejecutar el programa que permita que el robot en simulación se inicialice. Para ello, únicamente hay que escribir los siguientes comandos:

```bash
colcon build --packages-select pibotj_r2c# compila los paquetes
source ./install/setup.bash# configura variables
ros2 launch pibotj_r2c launch_sim.launch.py
```

Si la primera vez que se lance el robot ocurre algún error, puede ser normal; se vuelve a repetir el proceso de nuevo.

**Si quieres ver al robot en funcionamiento** tienes que escribir este comando por la terminal:

```bash
ros2 launch pibotj_r2c launch_sim.launch.py
```
Este [vídeo](https://www.youtube.com/watch?v=A0yi7YlLpq0) muestra una demo completa del robot ejecutado en simulación.

#### Robot físico





--------------------------
# English version

## Introduction 
