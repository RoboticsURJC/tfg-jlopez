
# Introducción 
Bienvenido/a al repo del TFG (Trabajo Fin de Carrera) de Julia López Augusto. En él podrás encontrar todo el código y ficheros correspondientes y podrás ver un seguimiento continuo en el apartado de [Wiki](https://github.com/RoboticsURJC/tfg-jlopez/wiki). Si deseas replicar este proyecto, te recomiendo seguir los siguientes pasos: 

## Construcción del robot
Para poder construir este robot, se han proporcionado una serie de ficheros .stl  que se pueden imprimir en cualquier impresora 3D convencional. Para ello, es necesario imprimir una pieza de cada una de las siguientes: 

- [Chasis](https://github.com/RoboticsURJC/tfg-jlopez/blob/main/design/base.stl)
- [Soporte de la cámara](https://github.com/RoboticsURJC/tfg-jlopez/blob/main/design/camara.stl)
- [Carcasa](https://github.com/RoboticsURJC/tfg-jlopez/blob/main/design/parte-superior.stl)
- [Sujección trasera](https://github.com/RoboticsURJC/tfg-jlopez/blob/main/design/sujeccion-trasera.stl)

Estas son las características de impresión: 


Esta es la tornillería usada: 


Para el ensamblaje, se ha creado este [fichero]() que muestra paso a paso el ensamblaje completo del robot. Finalmente el robot tiene que quedar así: 

(Incluir imagen)

## Soporte software 
A continuación, se van a explicar la implementación software desarrollada para el robot: 

### Simulación 

Primero de todo, hay que instalar los siguientes programas:
```bash
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-rviz2
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-xacro ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
```
Una vez instalados, únicamente hay que **ejecutar el programa que permita que el robot en simulación se inicialice**. Para ello, únicamente hay que escribir los siguientes comandos:

```bash
colcon build --packages-select pibotj_r2c# compila los paquetes
source ./install/setup.bash# configura variables
ros2 launch pibotj_r2c launch_sim.launch.py
```

Si la primera vez que se lance el robot ocurre algún error, puede ser normal; se vuelve a repetir el proceso de nuevo.

El vídeo de acontinuación, muestra una demo completa del robot ejecutado en simulación.

### Robot físico


```bash
ros2 launch pibotj_rr robot_teleop.launch.py
```

```bash
http://<ip robot>:8000/index_teleop.html
```
```bash
ros2 launch pibotj_rr robot_vff.launch.py
```

```bash
http://<ip robot>:8000/index_vff.html
```






