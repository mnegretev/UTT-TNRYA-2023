# Introducción a los Robots de Servicio Doméstico
Taller para el Torneo Nacional de Robótica y Aeronáutica, UTT, 2023.


## Requerimientos

* Ubuntu 20.04
* ROS Noetic ( http://wiki.ros.org/noetic/Installation/Ubuntu )

## Instalación

Nota: se asume que ya se tiene instalado Ubuntu y ROS.

* $ cd
* $ git clone https://github.com/mnegretev/UTT-TNRYA-2023
* $ cd UTT-TNRYA-2023
* $ ./Setup.sh
* $ cd catkin_ws
* $ catkin_make -j2 -l2

## Pruebas

Para probar que todo se instaló y compiló correctamente:

* $ cd 
* $ source UTT-TNRYA-2023/catkin_ws/devel/setup.bash
* $ roslaunch bring_up path_planning.launch

Si todo se instaló y compiló correctamente, se debería ver un visualizador RViz como el siguiente:

<img src="https://github.com/mnegretev/UTT-TNRYA-2023/blob/master/Media/rviz.png" alt="RViz" width="639"/>

Un ambiente simulado como el siguiente:

<img src="https://github.com/mnegretev/UTT-TNRYA-2023/blob/master/Media/gazebo.png" alt="Gazebo" width="631"/>

Y una GUI como la siguiente:

<img src="https://github.com/mnegretev/UTT-TNRYA-2023/blob/master/Media/gui.png" alt="GUI" width="454"/>

## Máquina virtual

Se puede descargar una máquina virtual para [VirtualBox](https://www.virtualbox.org/wiki/Downloads) con Ubuntu y ROs ya instalado de [esta dirección.](https://drive.google.com/drive/folders/1IqEFug0CiOSdiaw3HvmrAM3h2_HQhKpx?usp=share_link) <br>
Sólo es necesario  descomprimir el archivo y seguir las instrucciones del video que está en esa misma carpeta. La máquina virtual ya tiene todo instalado por lo que se puede pasar directo a la sección de "pruebas".<br> 
Se recomienda configurar la máquina virtual con 4 CPUs y 4GB de RAM.<br>
Usuario: student <br>
Contraseña: utt

## Contacto
Dr. Marco Negrete<br>
Profesor Asociado C<br>
Departamento de Procesamiento de Señales<br>
Facultad de Ingeniería, UNAM <br>
marco.negrete@ingenieria.unam.edu<br>
