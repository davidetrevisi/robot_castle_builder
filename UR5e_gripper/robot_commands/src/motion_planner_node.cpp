/**
 * @file motion_planner_node.cpp
 * @author Davide Trevisi
 * @brief ROS Node che esegue il movimento completo del braccio completo leggendo i valori dai messaggi dei topic specifici
 * @date 2022-07-17
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <moveit_msgs/OrientationConstraint.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit_msgs/AttachedCollisionObject.h>

#include <gazebo_ros_link_attacher/Attach.h>
#include <gazebo_ros_link_attacher/AttachRequest.h>
#include <gazebo_ros_link_attacher/AttachResponse.h>

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <pcl_action/custom.h>

#include <ros/ros.h>

#include <signal.h>

// Definisco le costanti necessarie (utile averle anche se non usate)

#define SIMULATION true

#define MAX_VELOCITY_SCALING_FACTOR 0.35
#define MAX_ACCELLERATION_SCALING_FACTOR 0.35

#define CARTESIAN_MAX_VELOCITY_SCALING_FACTOR 0.05
#define CARTESIAN_MAX_ACCELLERATION_SCALING_FACTOR 0.05

#define Z_BASE_LINK 1.791
#define Z_DESK 0.8675
#define Z_MIN 1.02
#define Z_INCREMENT 0.05

#define BUFFER_CUBE_ROWS 2
#define BUFFER_PARA_ROWS 1
#define BUFFER_CUBE_PER_ROW 10
#define BUFFER_PARA_PER_ROW 7
#define BUFFER_CUBE_SPACE 0.045
#define BUFFER_PARA_SPACE 0.07
#define BUFFER_ROW_SPACE 0.08

#define CUBE_MEASURE 0.025
#define PARALLELEPIPED_MEASURE 0.05

static geometry_msgs::Pose cube_buffer_pose, parallelepiped_buffer_pose;
static const char delim = '_';
static const std::string PLANNING_GROUP_ARM = "manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "endeffector";

// Definisco le variabili necessarie

moveit::planning_interface::MoveGroupInterface *arm_group;
moveit::planning_interface::MoveGroupInterface::Plan *arm_motion_plan;
moveit::planning_interface::MoveGroupInterface *gripper_group;
moveit::planning_interface::MoveGroupInterface::Plan *gripper_motion_plan;

moveit_msgs::Constraints lab_gripper;

moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;

shape_msgs::SolidPrimitive cube_primitive, parallelepiped_primitive;
moveit_msgs::AttachedCollisionObject block_collision, detach_object;
moveit_msgs::CollisionObject remove_object;

moveit::core::RobotStatePtr current_state;
std::vector<double> joint_group_positions;
const moveit::core::JointModelGroup *joint_model_group;

trajectory_processing::TimeOptimalTrajectoryGeneration *totg;
robot_trajectory::RobotTrajectory *robot_traj;

int cube_index = 0;
int parallelepiped_index = 0;
int buffer_counter = 0;

bool TARGET_BUFFER_FLAG = false;
bool TARGET_CASTLE_FLAG = false;

pcl_action::custom pos_msg;
geometry_msgs::PoseStamped action_msg;

bool iscube = false;

bool position_flag = false;
bool pickup_flag = false;
bool stack_flag = false;
bool place_flag = false;

/**
 * @brief Funzone che inizializza le variabili globali e i puntatori
 *
 */

void setup()
{
    arm_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM);
    arm_group->setPlanningTime(15.0);
    arm_group->setMaxVelocityScalingFactor(MAX_VELOCITY_SCALING_FACTOR);
    arm_group->setMaxAccelerationScalingFactor(MAX_ACCELLERATION_SCALING_FACTOR);

    arm_motion_plan = new moveit::planning_interface::MoveGroupInterface::Plan();

    totg = new trajectory_processing::TimeOptimalTrajectoryGeneration();
    robot_traj = new robot_trajectory::RobotTrajectory(arm_group->getRobotModel());

    // Inzializzo le variabili del gripper solo se siamo in ambiente simulato

    if (SIMULATION)
    {
        gripper_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GRIPPER);
        gripper_group->setPlanningTime(5.0);
        gripper_group->setMaxVelocityScalingFactor(0.1);
        gripper_group->setMaxAccelerationScalingFactor(0.1);

        gripper_motion_plan = new moveit::planning_interface::MoveGroupInterface::Plan();
    }

    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();

    // Definisco la collisione del cubo
    cube_primitive.type = cube_primitive.BOX;
    cube_primitive.dimensions.resize(3);
    cube_primitive.dimensions[0] = CUBE_MEASURE;
    cube_primitive.dimensions[1] = CUBE_MEASURE;
    cube_primitive.dimensions[2] = CUBE_MEASURE;

    // Definisco la collisione del parallelepipedo
    parallelepiped_primitive.type = parallelepiped_primitive.BOX;
    parallelepiped_primitive.dimensions.resize(3);
    parallelepiped_primitive.dimensions[0] = CUBE_MEASURE;
    parallelepiped_primitive.dimensions[1] = PARALLELEPIPED_MEASURE;
    parallelepiped_primitive.dimensions[2] = CUBE_MEASURE;

    // Definisco l'oggetto di collisione
    block_collision.link_name = "soft_robotics_gripper_base_link";
    block_collision.object.header.frame_id = arm_group->getPlanningFrame();
    block_collision.touch_links = std::vector<std::string>{"soft_robotics_gripper_base_link",
                                                           "soft_robotics_right_finger_link1",
                                                           "soft_robotics_left_finger_link1",
                                                           "soft_robotics_right_finger_link2",
                                                           "soft_robotics_left_finger_link2"};

    // Definisco la posizione del primo cubo nel buffer (in basso a sinistra, vedi figura)
    tf2::Quaternion cube_orientation(-1.0, 0.0, 0.0, 0.0);
    cube_orientation.normalize();

    cube_buffer_pose.position.x = 0.45;
    cube_buffer_pose.position.y = 0.4;
    cube_buffer_pose.position.z = Z_MIN;

    cube_buffer_pose.orientation = tf2::toMsg(cube_orientation);

    // Definisco la posizione del primo parallelepipedo nel buffer (in basso a sinistra, vedi figura)
    tf2::Quaternion parallelepiped_orientation(-0.7, 0.7, 0.0, 0.0);
    parallelepiped_orientation.normalize();

    parallelepiped_buffer_pose.position.x = 0.29;
    parallelepiped_buffer_pose.position.y = 0.395;
    parallelepiped_buffer_pose.position.z = Z_MIN;

    parallelepiped_buffer_pose.orientation = tf2::toMsg(parallelepiped_orientation);
}

/**
 * @brief Funzione che inizializza le costrizioni dei vari valori dei joint e orientazione dell'end-effector del braccio,
 *        valori testati in modo da limitare il braccio all'area di lavoro per favorire soluzioni 'ottimali' dell'inverse
 *        kinematics
 *
 */

void robot_constraints()
{
    // Inizializzo le variabili necessarie
    moveit_msgs::OrientationConstraint wrist_3_ocm;
    moveit_msgs::JointConstraint shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2;

    // 'shoulder_pan' impedisce la rotazione completa attorno all'asse
    shoulder_pan.joint_name = "shoulder_pan_joint";
    shoulder_pan.position = -1.5708;
    shoulder_pan.tolerance_below = 2.1708;
    shoulder_pan.tolerance_above = 2.1708;
    shoulder_pan.weight = 1.0;

    // 'shoulder_lift' limita il movimento del braccio
    shoulder_lift.joint_name = "shoulder_lift_joint";
    shoulder_lift.position = -1.62076;
    shoulder_lift.tolerance_below = 1.5208;
    shoulder_lift.tolerance_above = 1.5208;
    shoulder_lift.weight = 1.0;

    // 'elbow' limita il movimento del braccio
    elbow.joint_name = "elbow_joint";
    elbow.position = -1.62076;
    elbow.tolerance_below = 1.5208;
    elbow.tolerance_above = 1.5208;
    elbow.weight = 1.0;

    // 'wrist_1' limita il movimento del polso
    wrist_1.joint_name = "wrist_1_joint";
    wrist_1.position = -2.07076;
    wrist_1.tolerance_below = 1.0708;
    wrist_1.tolerance_above = 1.0708;
    wrist_1.weight = 1.0;

    // 'wrist_2' limita il movimento del polso ad una sola posizione
    wrist_2.joint_name = "wrist_2_joint";
    wrist_2.position = -1.5708;
    wrist_2.tolerance_below = 0.1;
    wrist_2.tolerance_above = 0.1;
    wrist_2.weight = 1.0;

    // 'wrist_3' fa in modo che il gripper sia sempre rivolto con il tubo
    // dell'aria verso il muro posteriore, impedendo situazioni pericolose
    // pur mantenendo un margine di movimento per la posa dei blocchi
    wrist_3_ocm.link_name = "tool0";
    wrist_3_ocm.header.frame_id = "base_link";
    wrist_3_ocm.orientation.z = -1;
    wrist_3_ocm.absolute_x_axis_tolerance = 0.01;
    wrist_3_ocm.absolute_y_axis_tolerance = 0.01;
    wrist_3_ocm.absolute_z_axis_tolerance = 1.59;
    wrist_3_ocm.weight = 1.0;

    // Aggiungo e applico le costrizioni
    lab_gripper.joint_constraints.push_back(shoulder_pan);
    lab_gripper.joint_constraints.push_back(shoulder_lift);
    lab_gripper.joint_constraints.push_back(elbow);
    lab_gripper.joint_constraints.push_back(wrist_1);
    lab_gripper.joint_constraints.push_back(wrist_2);
    lab_gripper.orientation_constraints.push_back(wrist_3_ocm);

    arm_group->clearPathConstraints();
    arm_group->setPathConstraints(lab_gripper);
}

/**
 * @brief Funzione che aggiunge l'oggetto di collisione 'cube_collision_ \e index ' alla scena MoveIt!
 *
 * @param pose posa (X, Y, Z, x, y, z, w) dell'oggetto da aggiungere alle collisioni
 * @param index indice dell'oggetto da aggiungere alle collisioni
 */

void add_cube_collision(geometry_msgs::Pose pose, int index)
{
    geometry_msgs::Pose local = pose;
    local.position.z = Z_DESK + (CUBE_MEASURE / 2.0);

    block_collision.object.id = "cube_collision_" + std::to_string(index);
    block_collision.object.primitives.clear();
    block_collision.object.primitive_poses.clear();
    block_collision.object.primitives.push_back(cube_primitive);
    block_collision.object.primitive_poses.push_back(local);
    block_collision.object.operation = block_collision.object.ADD;

    planning_scene_interface->applyCollisionObject(block_collision.object);
}

/**
 * @brief Funzione che aggiunge l'oggetto di collisione 'parallelepiped_collision_ \e index ' alla scena MoveIt!
 *
 * @param pose posa (X, Y, Z, x, y, z, w) dell'oggetto da aggiungere alle collisioni
 * @param index indice dell'oggetto da aggiungere alle collisioni
 */

void add_parallelepiped_collision(geometry_msgs::Pose pose, int index)
{
    geometry_msgs::Pose local = pose;
    local.position.z = Z_DESK + CUBE_MEASURE / 2.0;

    block_collision.object.id = "parallelepiped_collision_" + std::to_string(index);
    block_collision.object.primitives.clear();
    block_collision.object.primitive_poses.clear();
    block_collision.object.primitives.push_back(parallelepiped_primitive);
    block_collision.object.primitive_poses.push_back(local);
    block_collision.object.operation = block_collision.object.ADD;

    planning_scene_interface->applyCollisionObject(block_collision.object);
}

/**
 * @brief Funzione che rimuove le collisioni dalla scena MoveIt!
 *
 * @param sig numero del segnale
 */

void removeCollision(int sig)
{
    std::vector<std::string> object_ids;

    std::cout << std::endl
              << "Rimuovo le collisioni"
              << std::endl;

    // Rimuovo le collisioni e termino il programma

    for (int i = 1; i <= cube_index; i++)
    {
        object_ids.push_back("cube_collision_" + std::to_string(i));
    }

    for (int i = 1; i <= parallelepiped_index; i++)
    {
        object_ids.push_back("parallelepiped_collision_" + std::to_string(i));
    }

    planning_scene_interface->removeCollisionObjects(object_ids);
    ros::shutdown();
}

/**
 * @brief Funzione che calcola la posa dell'end-effector per posizionare il blocco
 *        nel buffer tramite l'indice 'counter'. La funzione calcola la posa
 *        in base alle costanti presenti e alle pose di default
 *
 * @param counter variabile che tiene conto del numero dei blocchi inseriti (0, n-1)
 *
 * @return posa target dell'end-effector per il posizionamento del blocco nel buffer,
 *         vuota in caso di buffer pieno
 */

geometry_msgs::Pose get_buffer_target(int counter)
{
    geometry_msgs::Pose return_pose;

    // Controllo se buffer pieno
    if (counter > (BUFFER_CUBE_ROWS * BUFFER_CUBE_PER_ROW + BUFFER_PARA_ROWS * BUFFER_PARA_PER_ROW))
    {
        std::cout << "ERRORE: Buffer pieno!" << std::endl
                  << std::endl;
        return_pose.position.x = return_pose.position.y = return_pose.position.z = 0;
        return_pose.orientation.x = return_pose.orientation.y = return_pose.orientation.z = return_pose.orientation.w = 0;
        return return_pose;
    }

    // Riga dei parallelepipedi: primo parallelepipedo della riga del cambio di oggetti
    if (counter == ((BUFFER_CUBE_ROWS * BUFFER_CUBE_PER_ROW) - 1))
    {
        return_pose = parallelepiped_buffer_pose;
        return return_pose;
    }

    // Riga dei parallelepipedi: considero la variabile 'counter' minore del massimo dei blocchi,
    // perché la funzione sarebbe uscita sopra in caso contrario. I cubi vengono prima dei
    // parallelepipedi.
    if (counter > ((BUFFER_CUBE_ROWS * BUFFER_CUBE_PER_ROW) - 1))
    {
        // Controllo se ho finito i blocchi nella riga in cui mi trovo
        if (counter % BUFFER_PARA_PER_ROW == (BUFFER_PARA_PER_ROW - 1))
        {
            // Passo alla riga successiva
            return_pose = parallelepiped_buffer_pose;

            return_pose.position.x = return_pose.position.x - BUFFER_ROW_SPACE;
            return return_pose;
        }
        else
        {
            // Passo al blocco successivo nella riga
            return_pose = parallelepiped_buffer_pose;

            return_pose.position.y = return_pose.position.y - (BUFFER_PARA_SPACE * (counter % BUFFER_PARA_PER_ROW));
            return return_pose;
        }
    }

    // Righe dei cubi: controllo se ho finito i blocchi nella riga in cui mi trovo
    if (counter % BUFFER_CUBE_PER_ROW == (BUFFER_CUBE_PER_ROW - 1))
    {
        // Passo alla riga successiva
        return_pose = cube_buffer_pose;

        return_pose.position.x = return_pose.position.x - BUFFER_ROW_SPACE;
        return return_pose;
    }
    else
    {
        // Passo al blocco successivo nella riga
        return_pose = cube_buffer_pose;

        return_pose.position.y = return_pose.position.y - (BUFFER_CUBE_SPACE * (counter % BUFFER_CUBE_PER_ROW));
        return return_pose;
    }
}

/**
 * @brief Funzione che trova l'indice tra le collisioni del blocco con la posa specificata e lo ritorna
 *
 * @param target posa (X, Y, Z, x, y, z, w) dell'oggetto da trovare nelle collisioni
 *
 * @return int contenente l'indice della collision, 0 in caso di errore
 */

int get_index_from_buffer_pose(geometry_msgs::Pose target)
{
    // Inizializzo le variabili necessarie (mappa e iteratore, legati alla funzione di MoveIt!)
    std::map<std::string, geometry_msgs::Pose> object_poses;
    std::map<std::string, geometry_msgs::Pose>::iterator i;
    geometry_msgs::Pose local = target;

    std::cout << "Posizione cercata: " << local.position.x << std::endl;
    std::cout << "Posizione cercata: " << local.position.y << std::endl;
    std::cout << "Posizione cercata: " << local.position.z << std::endl
              << std::endl;

    // Salvo la lista con le pose degli oggetti di collisione nella scena MoveIt!
    object_poses = planning_scene_interface->getObjectPoses(planning_scene_interface->getKnownObjectNames());

    // Ciclo nella mappa cercando un oggetto con la posa desiderata
    for (i = object_poses.begin(); i != object_poses.end(); i++)
    {
        std::cout << (i->second.position == local.position) << std::endl;
        std::cout << "Posizione trovata: " << i->second.position.x << std::endl;
        std::cout << "Posizione trovata: " << i->second.position.y << std::endl;
        std::cout << "Posizione trovata: " << i->second.position.z << std::endl
                  << std::endl;

        if (std::abs(i->second.position.x - local.position.x) <= 0.001 &&
            std::abs(i->second.position.y - local.position.y) <= 0.001 &&
            std::abs(i->second.position.z - local.position.z) <= 0.001)
        {
            std::cout << "OK1" << std::endl;
            // Ho trovato l'oggetto, estraggo l'id dal nome
            std::string s = i->first;

            std::stringstream s_stream(s);
            std::string segment;

            // Divido la stringa in sottostringhe "tagliando" in presenza del delimitatore '_'
            while (std::getline(s_stream, segment, delim))
            {
                char *n;

                // Provo a convertire la stringa in numero
                std::strtol(segment.c_str(), &n, 10);
                std::cout << "OK2" << std::endl;
                if (!*n)
                {
                    std::cout << "OK3" << std::endl;
                    // Conversione riuscita, ritorno il valore
                    return std::stoi(segment);
                }
            }
        }
    }
    return -1;
}

/**
 * @brief Funzione che esegue l'apertura del gripper in ambiente reale e simulato
 *
 * @param pub publisher ROS utlilzzato per l'invio di messaggi nel topic 'gripper_controller_cmd'. Usato solo in ambiente reale
 * @param client client ROS utilizzato per riconnettere il robot dopo l'invio del messaggio al gripper (ambiente reale)
 *               oppure client ROS per l'apertura del gripper (ambiente simulato)
 * @param model nome del modello di Gazebo da rilasciare dopo l'apertura ( \e cube-parallelepiped , usato soltanto in Gazebo)
 */

void open_gripper(ros::Publisher pub, ros::ServiceClient client, std::string model)
{
    if (SIMULATION)
    {
        // Ambiente simulato
        gripper_group->setJointValueTarget(gripper_group->getNamedTargetValues("open"));

        // Pianifico ed eseguo l'apertura del gripper
        bool success = (gripper_group->plan(*gripper_motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
        {
            std::cout << "Apertura gripper in corso..." << std::endl
                      << std::endl;
            gripper_group->move();
        }
        else
        {
            std::cout << "ERRORE: Planning fallito!" << std::endl
                      << std::endl;
            exit(1);
        }

        // Stacco il blocco
        gazebo_ros_link_attacher::Attach detach;

        detach.request.link_name_1 = "link";
        detach.request.model_name_1 = model;
        detach.request.link_name_2 = "wrist_3_link";
        detach.request.model_name_2 = "robot";

        client.call(detach);
    }
    else
    {
        // Ambiente reale, invio il messaggio di apertura al controller
        std_msgs::String msg;
        std::stringstream ss;

        ss << "open";
        msg.data = ss.str();
        std::cout << "Invio del messaggio al nodo di comando del gripper: " << msg.data.c_str() << std::endl;
        pub.publish(msg);
        ros::Duration(1.3).sleep();
        std_srvs::Trigger srv;
        client.call(srv);
    }
}

/**
 * @brief Funzione che esegue la chiusura del gripper in ambiente reale e simulato
 *
 * @param pub publisher ROS utlilzzato per l'invio di messaggi nel topic 'gripper_controller_cmd'. Usato solo in ambiente reale
 * @param client client ROS utilizzato per riconnettere il robot dopo l'invio del messaggio al gripper (ambiente reale)
 *               oppure client ROS per l'apertura del gripper (ambiente simulato)
 * @param model nome del modello di Gazebo da prendere dopo l'apertura ( \e cube-parallelepiped , usato soltanto in Gazebo)
 */

void close_gripper(ros::Publisher pub, ros::ServiceClient client, std::string model)
{
    if (SIMULATION)
    {
        // Ambiente simulato
        gripper_group->setJointValueTarget(gripper_group->getNamedTargetValues("close"));

        // Pianifico ed eseguo la chiusura del gripper
        bool success = (gripper_group->plan(*gripper_motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
        {
            std::cout << "Chiusura gripper in corso..." << std::endl
                      << std::endl;
            gripper_group->move();
        }
        else
        {
            std::cout << "ERRORE: Planning fallito!" << std::endl
                      << std::endl;
            exit(1);
        }

        // Attacco il blocco
        gazebo_ros_link_attacher::Attach attach;

        attach.request.link_name_1 = "link";
        attach.request.model_name_1 = model;
        attach.request.link_name_2 = "wrist_3_link";
        attach.request.model_name_2 = "robot";

        client.call(attach);
    }
    else
    {
        // Ambiente reale, invio il messaggio di chiusura al controller
        std_msgs::String msg;
        std::stringstream ss;

        ss << "close";
        msg.data = ss.str();
        std::cout << "Invio del messaggio al nodo di comando del gripper: " << msg.data.c_str() << std::endl;
        pub.publish(msg);
        ros::Duration(1.3).sleep();
        std_srvs::Trigger srv;
        client.call(srv);
    }
}

/**
 * @brief Funzione che esegue il planning ed il movimento del braccio linearmente verso una posa dell'end-effector passata come parametro
 *
 * @param target posa (X, Y, Z, x, y, z, w) che dovrà avere l'end-effector del braccio al termine del movimento
 */

void execute_Cartesian_Path(geometry_msgs::Pose target)
{
    // Inizializzo le variabili necessarie
    std::vector<geometry_msgs::Pose> waypoints;
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = 0;
    int counter = 0;

    // Stampa di cortesia della posizione in input
    std::cout << "Destinazione:\n"
              << std::endl;
    std::cout << "x: " << target.position.x << std::endl;
    std::cout << "y: " << target.position.y << std::endl;
    std::cout << "z: " << target.position.z << std::endl
              << std::endl;

    // Aggiungo la posa 'target' alla lista dei waypoint per il calcolo della traiettoria
    waypoints.push_back(target);

    // Inizializzo l'interfaccia per il movimento eliminando eventuali rimanenze da codice precedente
    arm_group->clearPoseTargets();
    arm_group->setStartStateToCurrentState();

    // Inizio un ciclo do-while calcolando la traiettoria finché non è completa oppure se non supero 3 tentativi
    do
    {
        // Se supero la prima iterazione aspetto un secondo
        if (counter > 0)
        {
            ros::Duration(1).sleep();
        }

        // Calcolo la traiettoria e salvo il risultato nella variabile 'fraction',
        // che indica la frazione di traiettoria eseguita (1 = 100%). I parametri per la funzione sono:
        //      lista di punti (solo uno in questo caso),
        //      end-effector resolution (precisione nella posizione, 4mm),
        //      jump threshold (valore soglia per evitare i 'salti', movimenti bruschi, dei joint: 5 dai test risulta buono)
        //      path constraints (costrizioni sulla posizione dei joint, passati quelli definiti a priori)
        fraction = arm_group->computeCartesianPath(waypoints, 0.004, 5.0, trajectory, robot_constraints);

        // Se la traiettoria è stata calcolata completa la assegno, altrimenti ciclo di nuovo
        // incrementando la variabile 'counter'
        if (fraction == 1)
        {
            robot_state::RobotStatePtr robot_stat = arm_group->getCurrentState();

            // Inizializzo la traiettoria per il ricalcolo dei tempi
            robot_traj->clear();
            robot_traj->setRobotTrajectoryMsg(*robot_stat, trajectory);
            robot_traj->setGroupName(PLANNING_GROUP_ARM);

            // Ricalcolo i tempi di ogni punto della traiettoria finale per eventuali errori e per limitare la velocità
            totg->computeTimeStamps(*robot_traj, CARTESIAN_MAX_VELOCITY_SCALING_FACTOR, CARTESIAN_MAX_ACCELLERATION_SCALING_FACTOR);

            // Assegno la traiettoria calcolata e la eseguo
            robot_traj->getRobotTrajectoryMsg(trajectory);
            arm_group->execute(trajectory);
        }

        counter++;
    } while (fraction < 1 && counter < 3);

    // Se non è stata calcolata nessuna traiettoria completa stampo un messaggio di errore e termino l'esecuzione
    if (fraction < 1)
    {
        std::cout << "ERRORE: Planning cartesiano fallito!" << std::endl
                  << std::endl;
        exit(1);
    }
}

/**
 * @brief Funzione che esegue il planning ed il movimento del braccio verso una posa dell'end-effector passata come parametro
 *
 * @param target posa (X, Y, Z, x, y, z, w) che dovrà avere l'end-effector del braccio al termine del movimento
 */

void motion_plan(geometry_msgs::Pose target)
{
    // Stampa di cortesia della posizione in input
    std::cout << "Destinazione:\n"
              << std::endl;
    std::cout << "x: " << target.position.x << std::endl;
    std::cout << "y: " << target.position.y << std::endl;
    std::cout << "z: " << target.position.z << std::endl
              << std::endl;

    // Inizializzo l'interfaccia per il movimento con la posa in input, eliminando eventuali rimanenze da codice precedente
    arm_group->clearPoseTargets();
    arm_group->setStartStateToCurrentState();
    arm_group->setPoseTarget(target);

    // Eseguo il planning e lo salvo nella variabile 'arm_motion_plan'
    bool success = (arm_group->plan(*arm_motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Se il planning è andato a buon fine eseguo il movimento, altrimenti ritorno un errore e termino l'esecuzione
    if (success)
    {
        std::cout << "Movimento in esecuzione" << std::endl
                  << std::endl;
        arm_group->execute(*arm_motion_plan);
    }
    else
    {
        std::cout << "ERRORE: Planning fallito!" << std::endl
                  << std::endl;
        exit(1);
    }
}

/**
 * @brief Funzione che esegue il planning ed il movimento del braccio verso una posa dell'end-effector data come parametro,
 *        passando per un punto intermedio definito a priori come posa (nel file .srdf di MoveIt!). Il movimento è l'unione
 *        di due traiettorie pianificate separatamente e con il timing ricalcolato
 *
 * @param target posa (X, Y, Z, x, y, z, w) che dovrà avere l'end-effector del braccio al termine del movimento
 */

void motion_plan_middle(geometry_msgs::Pose target)
{
    // Inizializzo le variabili necessarie
    robot_state::RobotStatePtr robot_stat;

    // Stampa di cortesia della posizione in input
    std::cout
        << "Destinazione (passando per 'middle_point'):\n"
        << std::endl;
    std::cout << "x: " << target.position.x << std::endl;
    std::cout << "y: " << target.position.y << std::endl;
    std::cout << "z: " << target.position.z << std::endl
              << std::endl;

    // Inizializzo l'interfaccia per il movimento con i valori dei joint del waypoint intermedio, eliminando eventuali rimanenze da codice precedente
    arm_group->clearPoseTargets();
    arm_group->setStartStateToCurrentState();
    arm_group->setJointValueTarget(arm_group->getNamedTargetValues("middle_point"));

    // Eseguo il planning fino al punto medio e lo salvo nella variabile 'arm_motion_plan'
    bool success = (arm_group->plan(*arm_motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Se il planning è andato a buon fine proseguo, altrimenti ritorno un errore e termino l'esecuzione
    if (success)
    {
        // Inizializzo la traiettoria con la parte appena trovata
        robot_stat = arm_group->getCurrentState();
        robot_traj->clear();
        robot_traj->setRobotTrajectoryMsg(*robot_stat, arm_motion_plan->trajectory_);

        // Inizializzo l'interfaccia per il movimento con la posa in input, iniziando dall'ultimo punto della traiettoria precedente
        // (il valore dei joint dalla posa 'middle_point') ed eliminando eventuali rimanenze da codice precedente
        arm_group->clearPoseTargets();
        arm_group->setStartState(robot_traj->getLastWayPoint());
        arm_group->setPoseTarget(target);

        // Eseguo il planning dal waypoint alla destinazione in input e lo salvo nella variabile 'arm_motion_plan'
        bool success = (arm_group->plan(*arm_motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // Se il planning è andato a buon fine proseguo, altrimenti ritorno un errore e termino l'esecuzione
        if (success)
        {
            // Inizializzo la seconda traiettoria tramite alcune funzioni particolari che mi permettono
            // di accodare la prima metà alla seconda
            robot_stat = arm_group->getCurrentState();
            robot_trajectory::RobotTrajectory second_traj = robot_trajectory::RobotTrajectory(arm_group->getRobotModel());

            second_traj.setRobotTrajectoryMsg(*robot_stat, arm_motion_plan->trajectory_);

            robot_traj->append(second_traj, 0.02);
            robot_traj->setGroupName(PLANNING_GROUP_ARM);

            // Ricalcolo i tempi di ogni punto della traiettoria finale per l'esecuzione
            totg->computeTimeStamps(*robot_traj, MAX_VELOCITY_SCALING_FACTOR, MAX_ACCELLERATION_SCALING_FACTOR);
            robot_traj->getRobotTrajectoryMsg(arm_motion_plan->trajectory_);

            // Eseguo la traiettoria finale
            std::cout << "Movimento in esecuzione" << std::endl
                      << std::endl;
            arm_group->execute(*arm_motion_plan);
        }
        else
        {
            std::cout << "ERRORE: Planning del target fallito!" << std::endl
                      << std::endl;
            exit(1);
        }
    }
    else
    {
        std::cout << "ERRORE: Planning del waypoint fallito!" << std::endl
                  << std::endl;
        exit(1);
    }
}

/**
 * @brief Funzione che sceglie se far passare il braccio per il punto medio oppure no.
 *        Il passaggio avviene supponendo di divirere il tavolo di lavoro in due metà
 *        rispetto all'origine del robot: se origine e destinazione si trovano sulla stessa metà
 *        allora non serve passare per il waypoint
 *
 * @param target posa (X, Y, Z, x, y, z, w) che dovrà avere l'end-effector del braccio al termine del movimento
 */

void select_arm_motion_plan(geometry_msgs::Pose target)
{
    // Breve controllo per vedere se passare o meno per il punto medio:
    // Se la posa corrente e la posa target sono opposte rispetto alla rotazione del robot
    // (x positiva o negativa) allora passo per il punto medio, viceversa se hanno lo stesso
    // segno eseguo il movimento diretto
    if (signbit(arm_group->getCurrentPose().pose.position.x))
    {
        // Posa corrente negativa
        if (signbit(target.position.x))
        {
            // Posa target negativa
            motion_plan(target);
        }
        else
        {
            // Posa target positiva
            motion_plan_middle(target);
        }
        ros::Duration(0.5).sleep();
    }
    else
    {
        // Posa corrente positiva
        if (signbit(target.position.x))
        {
            // Posa target negativa
            motion_plan_middle(target);
        }
        else
        {
            // Posa target positiva
            motion_plan(target);
        }
        ros::Duration(0.5).sleep();
    }
}

/**
 * @brief Funzione che assegna ad un determinato joint un valore passato come parametro
 *        (consultare il file URDF del robot per maggiori informazioni, numerazione crescente)
 *
 * @param joint_number il numero del joint del robot su cui agire
 * @param amount angolo che avrà \e joint_number dopo il movimento
 *
 */

void rotate_end_effector(int joint_number, double amount)
{
    // Brevi linee di codice per ottenere una rotazione RPY da una rotazione in quaternioni di una posa, nel caso possa tornare utile

    /*tf2::Quaternion q(
        target.orientation.x,
        target.orientation.y,
        target.orientation.z,
        target.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);*/

    // Inizializzo le variabili necessarie
    const moveit::core::JointModelGroup *joint_model_group = arm_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
    std::vector<double> joint_group_positions;

    // Salvo i valori attuali dei joint per la modifica
    current_state = arm_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // Controllo (basilare) del valore in input
    if (joint_number < 0)
    {
        std::cout << "ERRORE: Nessun joint valido specificato!" << std::endl
                  << std::endl;
        exit(1);
    }

    // Modifico l'angolo del joint inserito con il valore dal parametro
    joint_group_positions[joint_number] = amount;

    // Inizializzo l'interfaccia per il movimento con i nuovi valori dei joint, eliminando eventuali rimanenze da codice precedente
    arm_group->clearPoseTargets();
    arm_group->setStartStateToCurrentState();
    arm_group->setJointValueTarget(joint_group_positions);

    // Eseguo il planning e lo salvo nella variabile 'arm_motion_plan'
    bool success = (arm_group->plan(*arm_motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Se il planning è andato a buon fine eseguo il movimento, altrimenti ritorno un errore e termino l'esecuzione
    if (success)
    {
        std::cout << "Rotazione end-effector in esecuzione" << std::endl
                  << std::endl;
        arm_group->execute(*arm_motion_plan);
    }
    else
    {
        std::cout << "ERRORE: Planning fallito!" << std::endl
                  << std::endl;
        exit(1);
    }
}

/**
 * @brief Funzione che esegue il movimento in sequenza del braccio, chiamando le apposite funzioni, per la presa
 *        dei blocchi (con collisioni)
 *
 * @param demo_client_attach client ROS utilizzato per riconnettere il robot dopo l'invio del messaggio al gripper (ambiente reale)
 *               oppure client ROS per l'apertura del gripper (ambiente simulato)
 * @param demo_client_detach client ROS utilizzato per la chiusura del gripper (ambiente simulato)
 * @param gripper_pub publisher ROS utlilzzato per l'invio di messaggi nel topic 'gripper_controller_cmd'. Usato solo in ambiente reale
 */

void pickup(ros::ServiceClient demo_client_attach, ros::ServiceClient demo_client_detach, ros::Publisher gripper_pub)
{
    // Inizializzo le variabili necessarie
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
    planning_scene_monitor::LockedPlanningSceneRW ls(planning_scene_monitor);
    collision_detection::AllowedCollisionMatrix &acm = ls->getAllowedCollisionMatrixNonConst();
    moveit_msgs::PlanningScene diff_scene;

    geometry_msgs::Pose point, start;
    tf2::Quaternion target_q;

    // Imposto la rotazione di default (gripper con tubo dell'aria verso il muro posteriore)
    target_q.setX(-1);
    target_q.setY(0);
    target_q.setZ(0);
    target_q.setW(0);
    target_q.normalize();

    // Inizializzo le variabili locali in base al messaggio ricevuto
    if (TARGET_BUFFER_FLAG)
    {
        start.position.x = pos_msg.cx;
        start.position.y = pos_msg.cy;
        start.position.z = pos_msg.cz;
        iscube = pos_msg.cube;
        tf2::Quaternion o;

        if (iscube)
        {
            if (pos_msg.y > M_PI_4)
            {
                o.setRPY(pos_msg.r, pos_msg.p, pos_msg.y - M_PI_2);
            }
            else if (pos_msg.y < -M_PI_4)
            {
                o.setRPY(pos_msg.r, pos_msg.p, pos_msg.y + M_PI_2);
            }
            else
            {
                o.setRPY(pos_msg.r, pos_msg.p, pos_msg.y);
            }
        }
        else
        {
            o.setRPY(pos_msg.r, pos_msg.p, pos_msg.y);
        }
        o.normalize();
        start.orientation = tf2::toMsg(o);
    }
    else if (TARGET_CASTLE_FLAG)
    {
        start = action_msg.pose;
    }

    // Stampa di cortesia delle posizioni
    std::cout << "Destinazione iniziale:\n"
              << std::endl;
    std::cout << "x: " << start.position.x << std::endl;
    std::cout << "y: " << start.position.y << std::endl;
    std::cout << "z: " << start.position.z << std::endl;
    std::cout << "rot x: " << start.orientation.x << std::endl;
    std::cout << "rot y: " << start.orientation.y << std::endl;
    std::cout << "rot z: " << start.orientation.z << std::endl;
    std::cout << "rot w: " << start.orientation.w << std::endl
              << std::endl;

    // Inizializzo la posa del blocco da prendere
    point = start;

    // Aggiungo l'oggetto solo se lo metto nel buffer (non è presente nella scena)
    if (TARGET_BUFFER_FLAG)
    {
        // Aggiungo l'oggetto di collisione all'ambiente
        if (iscube)
        {
            cube_index++;
            add_cube_collision(point, cube_index);
        }
        else
        {
            parallelepiped_index++;
            add_parallelepiped_collision(point, parallelepiped_index);
        }
    }
    else if (TARGET_CASTLE_FLAG)
    {
        geometry_msgs::Pose local = point;
        local.position.z = Z_DESK + (CUBE_MEASURE / 2.0);

        block_collision.object.primitive_poses.clear();
        block_collision.object.primitive_poses.push_back(local);

        buffer_counter = get_index_from_buffer_pose(local);

        // Riga dei parallelepipedi: considero la variabile 'counter' minore del massimo dei blocchi,
        // perché la funzione sarebbe uscita in caso contrario. I cubi vengono prima dei
        // parallelepipedi.
        if (buffer_counter > ((BUFFER_CUBE_ROWS * BUFFER_CUBE_PER_ROW) - 1))
        {
            block_collision.object.id = "parallelepiped_collision_" + std::to_string(buffer_counter);
            block_collision.object.primitives.push_back(parallelepiped_primitive);
            iscube = false;
        }
        else
        {
            block_collision.object.id = "cube_collision_" + std::to_string(buffer_counter);
            block_collision.object.primitives.push_back(cube_primitive);
            iscube = true;
        }
    }

    // Sistemo la posa per il robot
    point.orientation = tf2::toMsg(target_q);
    point.position = start.position;
    point.position.z = start.position.z + Z_INCREMENT;

    select_arm_motion_plan(point);

    // Ruoto il gripper se la rotazione dell'end-effector della posizione desiderata è diversa da quella di default
    if (point.orientation != start.orientation)
    {
        point.orientation = start.orientation;
        motion_plan(point);
        // rotate_end_effector(5, angolo); può essere usata, ma è più semplice delegare il calcolo dell'angolo a MoveIt!
        ros::Duration(0.5).sleep();
    }

    // Eseguo il planning cartesiano verso la posizione di pick
    point.position.z = start.position.z;
    execute_Cartesian_Path(point);
    ros::Duration(0.5).sleep();

    // Aggiorno la matrice delle collisioni per rimuovere le collisioni del blocco con il gripper

    if (iscube)
    {
        acm.setEntry("cube_collision_" + std::to_string(cube_index), "soft_robotics_right_finger_link1", true);
        acm.setEntry("cube_collision_" + std::to_string(cube_index), "soft_robotics_left_finger_link1", true);
        acm.setEntry("cube_collision_" + std::to_string(cube_index), "soft_robotics_right_finger_link2", true);
        acm.setEntry("cube_collision_" + std::to_string(cube_index), "soft_robotics_left_finger_link2", true);
        acm.setEntry("cube_collision_" + std::to_string(cube_index), "desk", true);
    }
    else
    {
        acm.setEntry("parallelepiped_collision_" + std::to_string(parallelepiped_index), "soft_robotics_right_finger_link1", true);
        acm.setEntry("parallelepiped_collision_" + std::to_string(parallelepiped_index), "soft_robotics_left_finger_link1", true);
        acm.setEntry("parallelepiped_collision_" + std::to_string(parallelepiped_index), "soft_robotics_right_finger_link2", true);
        acm.setEntry("parallelepiped_collision_" + std::to_string(parallelepiped_index), "soft_robotics_left_finger_link2", true);
        acm.setEntry("parallelepiped_collision_" + std::to_string(parallelepiped_index), "desk", true);
    }

    ls->getPlanningSceneDiffMsg(diff_scene);

    // Aggiorno la scena in MoveIt!
    planning_scene_interface->applyPlanningScene(diff_scene);

    // Chiudo il gripper e aggancio l'oggetto
    if (iscube)
    {
        close_gripper(gripper_pub, demo_client_attach, "cubo_" + std::to_string(cube_index));
    }
    else
    {
        close_gripper(gripper_pub, demo_client_attach, "parallelepipedo_" + std::to_string(parallelepiped_index));
    }
    ros::Duration(0.5).sleep();

    // Modifico le collisioni del blocco, rimuovendolo dall'ambiente e attaccandolo al gripper
    block_collision.object.operation = block_collision.object.REMOVE;
    planning_scene_interface->applyCollisionObject(block_collision.object);

    block_collision.object.operation = block_collision.object.ADD;
    planning_scene_interface->applyAttachedCollisionObject(block_collision);

    // Eseguo il planning cartesiano verso la posizione precedente
    point.position.z = start.position.z + Z_INCREMENT;
    execute_Cartesian_Path(point);
    ros::Duration(0.5).sleep();

    // Ruoto il gripper in posizione di default se necessario
    if (point.orientation != tf2::toMsg(target_q))
    {
        point.orientation = tf2::toMsg(target_q);
        motion_plan(point);
        // rotate_end_effector(5, angolo); può essere usata, ma è più semplice delegare il calcolo dell'angolo a MoveIt!
        ros::Duration(0.5).sleep();
    }
}

/**
 * @brief Funzione che esegue il movimento in sequenza del braccio, chiamando le apposite funzioni, per il posizionamento
 *        dei blocchi (con collisioni)
 *
 * @param demo_client_attach client ROS utilizzato per riconnettere il robot dopo l'invio del messaggio al gripper (ambiente reale)
 *               oppure client ROS per l'apertura del gripper (ambiente simulato)
 * @param demo_client_detach client ROS utilizzato per la chiusura del gripper (ambiente simulato)
 * @param gripper_pub publisher ROS utlilzzato per l'invio di messaggi nel topic 'gripper_controller_cmd'. Usato solo in ambiente reale
 */

void place(ros::ServiceClient demo_client_attach, ros::ServiceClient demo_client_detach, ros::Publisher gripper_pub)
{
    // Inizializzo le variabili necessarie
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
    planning_scene_monitor::LockedPlanningSceneRW ls(planning_scene_monitor);
    collision_detection::AllowedCollisionMatrix &acm = ls->getAllowedCollisionMatrixNonConst();
    moveit_msgs::PlanningScene diff_scene;

    geometry_msgs::Pose point, target;
    tf2::Quaternion target_q;

    // Imposto la rotazione di default (gripper con tubo dell'aria verso il muro posteriore)
    target_q.setX(-1);
    target_q.setY(0);
    target_q.setZ(0);
    target_q.setW(0);
    target_q.normalize();

    // Inizializzo le variabili locali in base al messaggio ricevuto
    if (TARGET_BUFFER_FLAG)
    {
        target = get_buffer_target(buffer_counter);
    }
    else if (TARGET_CASTLE_FLAG)
    {
        target = action_msg.pose;
    }

    // Stampa di cortesia delle posizioni
    std::cout << "Destinazione finale:\n"
              << std::endl;
    std::cout << "x: " << target.position.x << std::endl;
    std::cout << "y: " << target.position.y << std::endl;
    std::cout << "z: " << target.position.z << std::endl;
    std::cout << "rot x: " << target.orientation.x << std::endl;
    std::cout << "rot y: " << target.orientation.y << std::endl;
    std::cout << "rot z: " << target.orientation.z << std::endl;
    std::cout << "rot w: " << target.orientation.w << std::endl
              << std::endl;

    // Eseguo il planning verso la posa finale
    // con orientazione di default (da prima) e altezza prefissata
    point.position = target.position;
    point.orientation = tf2::toMsg(target_q);
    point.position.z = target.position.z + Z_INCREMENT;

    select_arm_motion_plan(point);

    // Ruoto il gripper se la rotazione dell'end-effector della posizione desiderata è diversa da quella di default
    if (point.orientation != target.orientation)
    {
        point.orientation = target.orientation;
        motion_plan(point);
        // rotate_end_effector(5, angolo); può essere usata, ma è più semplice delegare il calcolo dell'angolo a MoveIt!
        ros::Duration(0.5).sleep();
    }

    // Eseguo il planning cartesiano verso la posizione di place
    point.position.z = target.position.z;
    execute_Cartesian_Path(point);
    ros::Duration(0.5).sleep();

    // Apro il gripper e sgancio l'oggetto
    if (iscube)
    {
        open_gripper(gripper_pub, demo_client_attach, "cubo_" + std::to_string(cube_index));
    }
    else
    {
        open_gripper(gripper_pub, demo_client_attach, "parallelepipedo_" + std::to_string(parallelepiped_index));
    }
    ros::Duration(0.5).sleep();

    // Modifico le collisioni del blocco, aggiungendolo all'ambiente e rimuovendolo dal gripper
    block_collision.object.operation = block_collision.object.REMOVE;
    planning_scene_interface->applyAttachedCollisionObject(block_collision);

    geometry_msgs::Pose local = point;
    local.position.z = Z_DESK + (CUBE_MEASURE / 2.0);

    block_collision.object.primitives.clear();
    block_collision.object.primitive_poses.clear();
    block_collision.object.primitives.push_back(cube_primitive);
    block_collision.object.primitive_poses.push_back(local);
    block_collision.object.operation = block_collision.object.ADD;

    planning_scene_interface->applyCollisionObject(block_collision.object);

    // Aggiorno la matrice delle collisioni per aggiungere le collisioni del blocco con il gripper

    if (iscube)
    {
        acm.setEntry("cube_collision_" + std::to_string(cube_index), "soft_robotics_right_finger_link1", false);
        acm.setEntry("cube_collision_" + std::to_string(cube_index), "soft_robotics_left_finger_link1", false);
        acm.setEntry("cube_collision_" + std::to_string(cube_index), "soft_robotics_right_finger_link2", false);
        acm.setEntry("cube_collision_" + std::to_string(cube_index), "soft_robotics_left_finger_link2", false);
        acm.setEntry("cube_collision_" + std::to_string(cube_index), "desk", false);
    }
    else
    {
        acm.setEntry("parallelepiped_collision_" + std::to_string(parallelepiped_index), "soft_robotics_right_finger_link1", false);
        acm.setEntry("parallelepiped_collision_" + std::to_string(parallelepiped_index), "soft_robotics_left_finger_link1", false);
        acm.setEntry("parallelepiped_collision_" + std::to_string(parallelepiped_index), "soft_robotics_right_finger_link2", false);
        acm.setEntry("parallelepiped_collision_" + std::to_string(parallelepiped_index), "soft_robotics_left_finger_link2", false);
        acm.setEntry("parallelepiped_collision_" + std::to_string(parallelepiped_index), "desk", false);
    }

    ls->getPlanningSceneDiffMsg(diff_scene);

    // Aggiorno la scena in MoveIt!
    planning_scene_interface->applyPlanningScene(diff_scene);

    // Eseguo il planning cartesiano verso la posizione precedente
    point.position.z = target.position.z + Z_INCREMENT;
    execute_Cartesian_Path(point);
    ros::Duration(0.5).sleep();

    // Ruoto il gripper in posizione di default se necessario
    if (point.orientation != tf2::toMsg(target_q))
    {
        point.orientation = tf2::toMsg(target_q);
        motion_plan(point);
        // rotate_end_effector(5, angolo); può essere usata, ma è più semplice delegare il calcolo dell'angolo a MoveIt!
        ros::Duration(0.5).sleep();
    }
}

/**
 * @brief Funzione che esegue il movimento in sequenza del braccio, chiamando le apposite funzioni, per il posizionamento
 *        di default del manipolatore
 *
 * @param demo_client_attach client ROS utilizzato per riconnettere il robot dopo l'invio del messaggio al gripper (ambiente reale)
 *               oppure client ROS per l'apertura del gripper (ambiente simulato)
 * @param demo_client_detach client ROS utilizzato per la chiusura del gripper (ambiente simulato)
 */

void reset(ros::ServiceClient demo_client_attach, ros::ServiceClient demo_client_detach)
{
    // Breve controllo per vedere se passare o meno per il punto medio:
    // Se la posa corrente e la posa target sono opposte rispetto alla rotazione del robot
    // (x positiva o negativa) allora passo per il punto medio, viceversa se hanno lo stesso
    // segno eseguo il movimento diretto (so a priori che la posa iniziale ha x positiva)
    if (signbit(arm_group->getCurrentPose().pose.position.x))
    {
        // Posa corrente negativa, eseguo il planning verso il punto medio
        arm_group->clearPoseTargets();
        arm_group->setStartStateToCurrentState();
        arm_group->setJointValueTarget(arm_group->getNamedTargetValues("middle"));
        arm_group->move();
        ros::Duration(0.5).sleep();
    }

    // Eseguo il planning verso il la posa iniziale per la successiva chiamata
    arm_group->clearPoseTargets();
    arm_group->setStartStateToCurrentState();
    arm_group->setJointValueTarget(arm_group->getNamedTargetValues("home"));
    arm_group->move();
    ros::Duration(0.5).sleep();

    // Porto il counter a puntare al blocco successivo
    buffer_counter++;

    // Resetto le flag per la successiva iterazione
    TARGET_BUFFER_FLAG = false;
    TARGET_CASTLE_FLAG = false;
}

/**
 * @brief Funzione che invia il messaggio di conferma del movimento al nodo del planning
 *
 * @param confirmation_pub publisher ROS utlilzzato per l'invio di messaggi nel topic 'confirmation_topic'
 */

void confirmation_msgs(ros::Publisher confirmation_pub)
{
    // Invio il messaggio di fine movimento al nodo del planning PlanSys2
    std_msgs::String msg;
    std::stringstream ss;

    ss << "SUCCESS";
    msg.data = ss.str();
    std::cout << "Invio del messaggio di conferma nel topic 'confirmation_topic'" << std::endl;
    confirmation_pub.publish(msg);
}

/**
 * @brief Funzione callback chiamata dopo la ricezione del messaggio dal nodo
 * della camera, inizializza la posa iniziale e il tipo di blocco in base ai valori del messaggio.
 * In aggiunta setta la variabile flag 'position_flag' per segnalare al loop principale di eseguire
 * il movimento
 *
 * @param msg messaggio proveniente dal nodo 'pcl_action'
 */

void buffer_callback(const pcl_action::custom::ConstPtr &msg)
{
    pos_msg.cx = msg->cx;
    pos_msg.cy = msg->cy;
    pos_msg.cz = msg->cz;
    pos_msg.r = -3.1415927;
    pos_msg.p = 0.0;
    pos_msg.y = msg->y;

    pos_msg.cube = msg->cube;

    position_flag = true;
    TARGET_BUFFER_FLAG = true;
}

/**
 * @brief Funzione callback chiamata dopo la ricezione del messaggio dal nodo
 * del planning, inizializza la posa finale in base ai valori del messaggio.
 * In aggiunta setta la variabile flag 'pickup_flag' per segnalare al loop principale di eseguire
 * il movimento
 *
 * @param msg messaggio proveniente dal nodo 'action_controller_node'
 */

void pickup_callback(const geometry_msgs::PoseStampedPtr &msg)
{
    action_msg.pose = msg->pose;

    pickup_flag = true;
    TARGET_CASTLE_FLAG = true;
}

/**
 * @brief Funzione callback chiamata dopo la ricezione del messaggio dal nodo
 * del planning, inizializza la posa finale in base ai valori del messaggio.
 * In aggiunta setta la variabile flag 'stack_flag' per segnalare al loop principale di eseguire
 * il movimento
 *
 * @param msg messaggio proveniente dal nodo 'action_controller_node'
 */

void stack_callback(const geometry_msgs::PoseStampedPtr &msg)
{
    action_msg.pose = msg->pose;

    stack_flag = true;
    TARGET_CASTLE_FLAG = true;
}

/**
 * @brief Funzione callback chiamata dopo la ricezione del messaggio dal nodo
 * del planning, inizializza la posa finale in base ai valori del messaggio.
 * In aggiunta setta la variabile flag 'place_flag' per segnalare al loop principale di eseguire
 * il movimento
 *
 * @param msg messaggio proveniente dal nodo 'action_controller_node'
 */

void place_callback(const geometry_msgs::PoseStampedPtr &msg)
{
    action_msg.pose = msg->pose;

    place_flag = true;
    TARGET_CASTLE_FLAG = true;
}

///////////////////////////////////////////////////////////////////////////////

int main(int argc, char **args)
{
    // Inizializzo il nodo
    ros::init(argc, args, "motion_plan_node");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);

    // Inizializzo i publisher di ROS per i messaggi al controller del gripper e per la conferma del movimento
    ros::Publisher gripper_pub = n.advertise<std_msgs::String>("gripper_controller_cmd", 10);
    ros::Publisher confirmation_pub = n.advertise<std_msgs::String>("confirmation_topic", 10);

    // Inizializzo i subscriber di ROS per leggere i messaggi dai nodi esterni
    ros::Subscriber sub = n.subscribe("/custom_msg", 1, buffer_callback);
    ros::Subscriber pickup_sub = n.subscribe("/points/pickup", 1, pickup_callback);
    ros::Subscriber stack_sub = n.subscribe("/points/stack", 1, stack_callback);
    ros::Subscriber place_sub = n.subscribe("/points/place", 1, place_callback);

    spinner.start();
    setup();

    ros::ServiceClient attach_srv, detach_srv, client;

    if (SIMULATION)
    {
        attach_srv = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
        detach_srv = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

        // Eseguo il planning alla posizione iniziale del braccio (usato solo per Gazebo)
        arm_group->setJointValueTarget(arm_group->getNamedTargetValues("home"));
        arm_group->move();
    }
    else
    {
        client = n.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/resend_robot_program");
    }

    // Aggiungo le costrizioni al robot
    robot_constraints();

    // Gestisco la rimozione delle collision all'uscita del programma
    signal(SIGINT, removeCollision);

    std::cout << "Robot pronto a ricevere comandi" << std::endl
              << std::endl;

    // Loop infinito
    while (ros::ok())
    {
        // Se il flag è stato settato dalla funzione callback del messaggio eseguo il movimento
        if (position_flag)
        {
            if (SIMULATION)
            {
                // In ambiente simulato ho bisogno di tutti e 4 i parametri per il gripping virtuale degli oggetti
                pickup(attach_srv, detach_srv, gripper_pub);
                place(attach_srv, detach_srv, gripper_pub);
                reset(attach_srv, detach_srv);
                std::cout << "Movimento terminato correttamente" << std::endl
                          << std::endl;
            }
            else
            {
                // In ambiente reale il secondo parametro non viene usato
                pickup(client, detach_srv, gripper_pub);
                place(client, detach_srv, gripper_pub);
                reset(client, detach_srv);
                std::cout << "Movimento terminato correttamente" << std::endl
                          << std::endl;
            }

            // Resetto il flag per la lettura successiva
            position_flag = false;
        }

        if (pickup_flag)
        {
            if (SIMULATION)
            {
                // In ambiente simulato ho bisogno di tutti e 4 i parametri per il gripping virtuale degli oggetti
                pickup(attach_srv, detach_srv, gripper_pub);

                // Invio il messaggio di conferma del movimento
                confirmation_msgs(confirmation_pub);
            }
            else
            {
                // In ambiente reale il secondo parametro non viene usato
                pickup(client, detach_srv, gripper_pub);

                // Invio il messaggio di conferma del movimento
                confirmation_msgs(confirmation_pub);
            }

            // Resetto il flag per la lettura successiva
            pickup_flag = false;
        }

        if (stack_flag)
        {
            if (SIMULATION)
            {
                // In ambiente simulato ho bisogno di tutti e 4 i parametri per il gripping virtuale degli oggetti
                place(attach_srv, detach_srv, gripper_pub);
                reset(attach_srv, detach_srv);

                // Invio il messaggio di conferma del movimento
                confirmation_msgs(confirmation_pub);
                std::cout << "Movimento terminato correttamente" << std::endl
                          << std::endl;
            }
            else
            {
                // In ambiente reale il secondo parametro non viene usato
                place(client, detach_srv, gripper_pub);
                reset(client, detach_srv);

                // Invio il messaggio di conferma del movimento
                confirmation_msgs(confirmation_pub);
                std::cout << "Movimento terminato correttamente" << std::endl
                          << std::endl;
            }

            // Resetto il flag per la lettura successiva
            stack_flag = false;
        }

        if (place_flag)
        {
            if (SIMULATION)
            {
                // In ambiente simulato ho bisogno di tutti e 4 i parametri per il gripping virtuale degli oggetti
                place(attach_srv, detach_srv, gripper_pub);
                reset(attach_srv, detach_srv);

                // Invio il messaggio di conferma del movimento
                confirmation_msgs(confirmation_pub);
                std::cout << "Movimento terminato correttamente" << std::endl
                          << std::endl;
            }
            else
            {
                // In ambiente reale il secondo parametro non viene usato
                place(client, detach_srv, gripper_pub);
                reset(client, detach_srv);

                // Invio il messaggio di conferma del movimento
                confirmation_msgs(confirmation_pub);
                std::cout << "Movimento terminato correttamente" << std::endl
                          << std::endl;
            }

            // Resetto il flag per la lettura successiva
            place_flag = false;
        }
    }
}