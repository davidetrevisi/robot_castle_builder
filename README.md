# Robot Castle Builder

Repository del progetto di tirocinio e tesi di laurea del corso di Ingegneria Informatica, delle Comunicazioni ed Elettronica, anno 2021-2022

# Progetto

Lo scopo di questo progetto è posizionare dei blocchi da costruzione, dati in ordine sparso e in posizioni casuali, per costruire una struttura. Questa parte si focalizza prevalentemente sul planning e sul movimento del robot.

Il robot utilizzato è un UR5e con un gripper a 2 dita azionato ad aria compressa, la struttura di supporto del robot è mappata sul tavolo di lavoro presente in laboratorio.

### Video

[![Demo Costruzione Gazebo](https://user-images.githubusercontent.com/89746496/200953324-7f3220e2-f50d-412e-9f93-33a03dd09014.jpg)](https://youtu.be/9SK5fYOJhfQ)

https://user-images.githubusercontent.com/89746496/202859210-4fdf2a31-a90c-4ac1-a155-c9a835840753.mp4

### Struttura del progetto

I pacchetti da cui dipende la repository sono:

- **Universal_Robot** e relativi: contengono tutte le definizioni e file relativi al robot UR5e in uso
- **Universal_Robot_ROS_Driver** e relativi: contengono tutti i file necessari all'utilizzo del robot nell'ambiente reale
- **gazebo_ros_link_attacher**: necessario nell'ambiente simulato poiché permette di unire due oggetti in Gazebo tramite link a runtime

I pacchetti presenti nella repository sono:

- **soft_robotics_description**: contiene i file di descrizione del gripper
- **soft_robotics_gazebo**: contiene i file per unire il gripper al robot senza usare il file `urdf`
- **integration_package**: contiene tutti i file di definizione dell'UR5e + il gripper
- **integration_package_moveit**: contiene i file di configurazione per MoveIt dell'UR5e + gripper
- **robot_commands**: contiene i programmi per il movimento del robot

### Utilizzo del progetto

Clonare la repository nella cartella `src` di un ambiente catkin già inizializzato. Controllare che le dipendenze del progetto siano soddisfatte utilizzando `rosdep`

# Comandi

### Ambiente simulato

Lancio **Gazebo** con l'ambiente di simulazione necessario, in pausa per evitare problemi ai joint del robot

```
roslaunch integration_package ur5e_gripper_bringup_gazebo.launch paused:=true
```

Lancio il pacchetto di integrazione con **MoveIt** per il robot, con le necessarie configurazioni e plugin, in ambiente simulato

```
roslaunch integration_package_moveit moveit_planning_execution.launch sim:=true
```

Lancio il nodo che aggiunge le collisioni virtuali al robot, per evitare pose e collisioni indesiderate

```
rosrun robot_commands add_collisions
```

Lancio **Rviz** per la visualizzazione del planning e delle collisioni (se necessario)

```
roslaunch integration_package_moveit moveit_rviz.launch
```

Lancio la demo del movimento del robot

```
rosrun robot_commands demo
```

### Ambiente reale

Lancio la calibrazione del braccio (necessaria solo la prima volta)

```
roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.0.100 target_filename:=$(rospack find integration_package)/calibration_files/ex-ur5e_calibration.yaml
```

Lancio il robot con i parametri corretti

```
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.0.100 kinematics_config:=$(rospack find integration_package)/calibration_files/ex-ur5e_calibration.yaml robot_description_file:=$(rospack find integration_package)/launch/load_ur5e_gripper.launch headless_mode:=true
```

Lancio il pacchetto di integrazione con **MoveIt** per il robot

```
roslaunch integration_package_moveit moveit_planning_execution.launch
```

Lancio il nodo che aggiunge le collisioni virtuali al robot, per evitare pose e collisioni indesiderate

```
rosrun robot_commands add_collisions
```

Lancio il nodo per l'invio dei comandi al gripper:

```
rosrun soft_robotics_description gripper_controller.py
```

Lancio **Rviz** per la visualizzazione del planning e delle collisioni (se necessario)

```
roslaunch integration_package_moveit moveit_rviz.launch
```
