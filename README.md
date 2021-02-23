# Curso Robots Móviles 2021-2 FI, UNAM

Material para el curso de Robots Móviles de la Facultad de Ingeniería, UNAM, Semestre 2021-2

## Requerimientos

* Ubuntu 18.04
* ROS Melodic http://wiki.ros.org/melodic/Installation/Ubuntu

## Instalación

Nota: se asume que ya se tiene instalado Ubuntu y ROS.

* $ cd
* $ git clone https://github.com/mnegretev/Mobile-Robots-2021-1
* $ cd Mobile-Robots-2021-1
* $ ./Setup.sh
* $ cd catkin_ws
* $ catkin_make -j2 -l2

Para probar que todo se instaló y compiló correctamente:

* $ cd 
* $ export GAZEBO_MODEL_PATH=~/Mobile-Robots-2021-1/catkin_ws/src/bring_up/models/
* $ source Mobile-Robots-2021-1/catkin_ws/devel/setup.bash
* $ roslaunch bring_up justina_simple.launch

Nota: Se asume que el repositorio se descargó en la carpeta home. Si se descargó en otra ruta, es necesario realizar los cambios correspondientes en los comandos. 

Si todo se instaló y compiló correctamente, se debería ver un Gazebo como el siguiente:<br>
<img src="https://github.com/mnegretev/Mobile-Robots-2021-1/blob/master/Media/gazebo.png" alt="Star Gazer App" width="700"/>
<br><br>
Y un RViz como el siguiente:<br>
<img src="https://github.com/mnegretev/Mobile-Robots-2021-1/blob/master/Media/rviz.png" alt="Star Gazer App" width="700"/>

## Contacto
Dr. Marco Negrete<br>
Profesor Asociado C<br>
Departamento de Procesamiento de Señales<br>
Facultad de Ingeniería, UNAM <br>
[mnegretev.info](http://mnegretev.info)<br>
contact@mnegretev.info<br>
marco.negrete@ingenieria.unam.edu<br>
