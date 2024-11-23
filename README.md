
# Introducción 
Bienvenido/a al repo del TFG (Trabajo Fin de Carrera) de Julia López Augusto. En él podrás encontrar todo el código y ficheros correspondientes y podrás ver un seguimiento continuo en el apartado de [Wiki](https://github.com/RoboticsURJC/tfg-jlopez/wiki). Si se desea replicar este proyecto, es recomendable seguir los siguientes pasos: 

# Construcción del robot
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


# Soporte software 
A continuación, se va a explicar la implementación software desarrollada para el robot: 

## Simulación 

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

## Robot físico

Hay que seguir los siguientes pasos:
1. Instalar Raspberry Pi Imager : ``sudo apt install rpi-imager``.
2. Descargar la [imagen de Ubuntu 20.04 Server](https://releases.ubuntu.com/20.04.6/).
3. Abrir rpi-imager y seguir los pasos que aparecen por pantalla.
4. Conectarse al robot usando ``ssh usuario@ip_dispositivo`` con la contraseña introducida en el Paso 3.
5. Una vez conectado con el robot, ya se puede operar con él.
6. Instalar [ROS 2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).
7. Configurar cada uno de los componentes del robot.
   
Al no tener disponible interfaz gráfica, la única forma de comunicarse con el robot es a través de ssh y variantes de esta, como scp, para copiar directorios entre el robot y el ordenador local. Otra opción para poder desarrollar código es usar un plugin de VSCode (desde el ordenador local) que permite usar [VSCode conectándose al robot usando ssh](https://code.visualstudio.com/docs/remote/ssh-tutorial).


### Cámara
La cámara está conectada al puerto CSI y, para hacerla funcionar, se necesitan instalar los siguientes programas:
```bash
sudo apt-get install gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly
```
Una vez instalados, hay que seguir los siguientes pasos:

1. Editar el fichero ``/boot/firmware/config.txt`` añadiendo ``start_x=1`` al final.
2. Reiniciar la Raspberry: ``sudo reboot``.
3. Comprobar que la configuración es correcta: ``v4l2-ctl --list-devices``.
   
En la siguiente figura se puede ver el resultado del Paso 3 y muestra que la cámara se encontraba conectada en el dispositivo ``/dev/video0``.

<p align="center">
<img src="https://github.com/RoboticsURJC/tfg-jlopez/blob/main/memoria/figs/cap6/vl.png" width="40%" height="40%">
</p>

### Google Coral
Para su configuración hay que seguir los siguientes pasos:
1.
```bash
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
```
2.
```bash
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
```
3. ``sudo apt-get update``.
4. ``sudo apt-get install libedgetpu1-std``.
5. Conecta el USB en uno de los puertos 3.0.
6. ``sudo apt-get install python3-pycoral``.

Finalmente, el proceso se deberı́a haber completado de forma exitosa cuando la salida del último comando se deberı́a ver como muestra la sigueiente figura:

<p align="center">
<img src="https://github.com/RoboticsURJC/tfg-jlopez/blob/main/memoria/figs/cap6/pycoralinstalled.png" width="40%" height="40%">
</p>

### Módulo GPS
Hay que seguir los siguientes pasos:
1. Crear el fichero ``/etc/udev/rules.d/99-ttyAMA0.rules``.
2. Añadir: ``KERNEL=="ttyAMA0", MODE="0666", GROUP="dialout"``.
3. ``sudo udevadm control --reload-rules``.
4. ``sudo udevadm trigger``.
5. Las reglas se habrán actualizado.
6. [Instalar raspi-config](https://github.com/EmilGus/install_raspi-config/tree/master).
7. Ejecutar raspi-config usando ``sudo raspi-config``
8. Dentro de raspi-config hay que desplazarse hasta Interfacing options y serial.
9. Deshabilitar serial login shell y habilitar serial interface.
10. Reiniciar la Raspberry: ``sudo reboot``.

Finalmente, los siguientes ficheros tienen que contener la siguiente información:

```bash
cat /boot/firmware/config.txt
```
```bash
# Please DO NOT modify this file; if you need to modify the boot config,
# the "usercfg.txt" file is the place to include user changes. Please
# refer to the README file for a description of the various configuration
# files on the boot partition.
# The unusual ordering below is deliberate; older firmwares (in particular
# the version initially shipped with bionic) don’t understand the
conditional
# [sections] below and simply ignore them. The Pi4 doesn’t boot at all
# with firmwares this old so it’s safe to place at the top. Of the Pi2 and
# Pi3, the Pi3 uboot happens to work happily on the Pi2, so it needs to go
# at the bottom to support old firmwares.
[pi4]
kernel=uboot_rpi_4.bin
[pi2]
kernel=uboot_rpi_2.bin
[pi3]
kernel=uboot_rpi_3.bin
[pi0]
kernel=uboot_rpi_3.bin
[all]
device_tree_address=0x03000000
[pi4]
max_framebuffers=2
arm_boost=1
[all]
# Enable the audio output, I2C and SPI interfaces on the GPIO header. As
these
# parameters related to the base device-tree they must appear *before* any
# other dtoverlay= specification
dtparam=audio=on
dtparam=i2c_arm=on
dtparam=spi=on
# Comment out the following line if the edges of the desktop appear
outside
# the edges of your display
disable_overscan=1
# If you have issues with audio, you may try uncommenting the following
line
# which forces the HDMI output into HDMI mode instead of DVI (which doesn’t
#  support audio output)
#hdmi_drive=2
# Config settings specific to arm64
arm_64bit=1
dtoverlay=dwc2
[cm4]
# Enable the USB2 outputs on the IO board (assuming your CM4 is plugged
into
# such a board)
dtoverlay=dwc2,dr_mode=host
[all]
# The following settings are "defaults" expected to be overridden by the
# included configuration. The only reason they are included is, again, to
# support old firmwares which don’t understand the "include" command.
enable_uart=1
cmdline=cmdline.txt
include syscfg.txt
include usercfg.txt
start_x=1
```
```bash
cat /boot/firmware/cmdline.txt
```
```bash
elevator=deadline net.ifnames=0 dwc_otg.lpm_enable=0 root=LABEL=writable rootfstype=ext4 rootwait fixrtc quiet splash cfg80211.ieee80211_regdom=GB
```
```bash
cat /boot/firmware/usercfg.txt
```
```bash
# Place "config.txt" changes (dtparam, dtoverlay, disable_overscan, etc.)
in
# this file. Please refer to the README file for a description of the
various
# configuration files on the boot partition.
dtoverlay=pi3-disable-bt
```
```bash
cat /boot/firmware/syscfg.txt
```
```bash
# This file is intended to be modified by the pibootctl utility. User
# configuration changes should be placed in "usercfg.txt". Please refer
to the
# README file for a description of the various configuration files on the
boot
# partition.

enable_uart=1
dtparam=audio=on
dtparam=i2c_arm=on
dtparam=spi=on
cmdline=cmdline.txt
```
Debido a la distribución de Ubuntu usada, hay problemas al encender la Raspberry Pi si tiene algo a través del [puerto serie](https://wiki.ubuntu.com/EoanErmine/ReleaseNotes#Raspberry_Pi), pero las sugerencias de otros usuarios no conseguı́an hacer funcionar bien el módulo; por lo tanto, la única solución encontrada es desconectar el módulo GPS hasta que se inicia sesión a través de ssh y luego conectar el módulo.

El módulo GPS realmente capta información válida cuando se enciende el LED que tiene integrado. Una forma fiable para que el módulo reciba todo el rato señal correcta es dejarle en el exterior hasta que se encienda el LED y luego poder operar con el módulo en interior o exterior.

### Servomotores
Hay que seguir los siguientes pasos:
1. ``sudo apt-get update``.
2. ``sudo apt-get install rpi.gpio-common python3-pigpio``
3. ``sudo apt-get install python3-gpiozero python3-rpi.gpio``.
4. Hay que añadir ``dialout`` a ``groups``.
5. Los motores ya estarán listos para usarse importando la librerı́a ``RPi.GPIO``.


### Aplicaciones completas 

Para ejecutar a PiBotJ en modo **teleoperado**, es necesario escribir los siguientes comandos: 

- En la terminal: ``ros2 launch pibotj_rr robot_teleop.launch.py``

- En un navegador: ``http://<ip robot>:8000/index_teleop.html``

A continuación, se muestra un vídeo mostrando a PiBotJ de forma teleoperada:

<a href="https://www.youtube.com/watch?v=qGbJ7IGwjWk">
    <img src="https://img.youtube.com/vi/qGbJ7IGwjWk/mqdefault.jpg" alt="">
</a>

Para ejecutar a PiBotJ en modo **autónomo**, es necesario escribir los siguientes comandos: 

- En la terminal: ``ros2 launch pibotj_rr robot_vff.launch.py ``

- En un navegador: ``http://<ip robot>:8000/index_vff.html``

A continuación, se muestra un vídeo mostrando a PiBotJ de forma autónoma:

<a href="https://www.youtube.com/watch?v=zQudXBXHVaY">
    <img src="https://img.youtube.com/vi/zQudXBXHVaY/mqdefault.jpg" alt="">
</a>





