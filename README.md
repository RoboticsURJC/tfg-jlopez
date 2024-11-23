
# Introducción 
Bienvenido/a al repo del TFG (Trabajo Fin de Carrera) de Julia López Augusto. En él podrás encontrar todo el código y ficheros correspondientes y podrás ver un seguimiento continuo en el apartado de [Wiki](https://github.com/RoboticsURJC/tfg-jlopez/wiki). Si deseas replicar este proyecto, te recomiendo seguir los siguientes pasos: 

## Construcción del robot
Para poder construir este robot, se han proporcionado una serie de ficheros .stl  que se pueden imprimir en cualquier impresora 3D convencional. Para ello, es necesario imprimir una pieza de cada una de las siguientes: 

- [Chasis](https://github.com/RoboticsURJC/tfg-jlopez/blob/main/design/base.stl)
- [Soporte de la cámara](https://github.com/RoboticsURJC/tfg-jlopez/blob/main/design/camara.stl)
- [Carcasa](https://github.com/RoboticsURJC/tfg-jlopez/blob/main/design/parte-superior.stl)
- [Sujección trasera](https://github.com/RoboticsURJC/tfg-jlopez/blob/main/design/sujeccion-trasera.stl)

Estas son las características de impresión: 
<table>
  
  <tr>
    <th>Características</th>
    <th>Parámetros</th>
  </tr>
  <tr>
    <td rowspan="2"><b>Calidad</b></td>
    <td>Altura de capa: 0,2 mm</td>
  </tr>
  <tr>
    <td>Ancho de línea: 0,4 mm</td>
  </tr>
  <tr>
    <td rowspan="4"><b>Paredes</b></td>
    <td>Grosor de pared: 0,8 mm</td>
  </tr>
  <tr>
    <td>Cantidad de líneas de pared: 2</td>
  </tr>
  <tr>
    <td>Alineación de costura en Z: Esquina más afilada</td>
  </tr>
  <tr>
    <td>Preferencia de costura en esquina: Ocultación inteligente</td>
  </tr>
  <tr>
    <td rowspan="2"><b>Relleno</b></td>
    <td>Densidad de relleno: 15%</td>
  </tr>
  <tr>
    <td>Patrón de relleno: Gyroid</td>
  </tr>
  <tr>
    <td rowspan="2"><b>Velocidad</b></td>
    <td>Velocidad de impresión: 50 mm/s</td>
  </tr>
  <tr>
    <td>Velocidad de la primera capa: 20 mm/s</td>
  </tr>
</table>


Esta es la tornillería usada: 

| **Componente**         | **Tornillos**  | **Tuercas**| **Arandelas**| **Hama beads blancas**|
|------------------------|----------------|------------|--------------|-----------------------|
| Motores                | 12 M2 10mm     |   12       |      24      |                       |
| PiCamera(base)         |  2 M2 10mm     |    2       |       4      |                       |
| PiCamera(cámara)       |  2 M2 12mm     |    2       |      12      |                       |
| Raspberry Pi           |  4 M2 12mm     |    4       |              |         4             |
| Placa GPS              |  4 M2 16mm     |    4       |       8      |         4             |
| Antena GPS             |  4 M2 16mm     |    4       |              |         4             |
| Sujección entre placas |  4 M2 16mm     |    4       |       8      |                       |
| Sujección trasera      |  1 M2 16mm     |    1       |       2      |                       |




Para el ensamblaje, se ha creado este [fichero](https://github.com/RoboticsURJC/tfg-jlopez/blob/main/design/robotcompleto.FCStd) que muestra paso a paso el ensamblaje completo del robot. Finalmente el robot tiene que quedar así: 

<p align="center">
<img src="https://github.com/RoboticsURJC/tfg-jlopez/blob/main/memoria/figs/cap5/completo3.png" width="40%" height="40%">
</p>

<p align="center">
<img src="https://github.com/RoboticsURJC/tfg-jlopez/blob/main/memoria/figs/cap5/ab.jpeg" width="20%" height="20%">
</p>


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

<a href="https://www.youtube.com/watch?v=A0yi7YlLpq0">
    <img src="https://img.youtube.com/vi/A0yi7YlLpq0/mqdefault.jpg" alt="">
</a>

### Robot físico


```bash
ros2 launch pibotj_rr robot_teleop.launch.py
```

```bash
http://<ip robot>:8000/index_teleop.html
```

<a href="https://www.youtube.com/watch?v=qGbJ7IGwjWk">
    <img src="https://img.youtube.com/vi/qGbJ7IGwjWk/mqdefault.jpg" alt="">
</a>

```bash
ros2 launch pibotj_rr robot_vff.launch.py
```

```bash
http://<ip robot>:8000/index_vff.html
```

<a href="https://www.youtube.com/watch?v=zQudXBXHVaY">
    <img src="https://img.youtube.com/vi/zQudXBXHVaY/mqdefault.jpg" alt="">
</a>





