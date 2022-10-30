/**
 * @file ik_testing.cpp
 * @author Davide Trevisi
 * @brief ROS Node che esegue il movimento del braccio da posizioni inserite da terminale
 * @version 0.2
 * @date 2022-07-21
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <moveit_msgs/OrientationConstraint.h>
#include <moveit_msgs/JointConstraint.h>

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <ros/ros.h>

#include <chrono>

// Definisco le costanti necessarie (utile averle anche se non usate)

#define MAX_VELOCITY_SCALING_FACTOR 0.05
#define MAX_ACCELLERATION_SCALING_FACTOR 0.05

#define Z_BASE_LINK 1.791
#define Z_DESK 0.8675
#define Z_MIN 1.02
#define Z_INCREMENT 0.05

static const std::string PLANNING_GROUP_ARM = "manipulator";

// Definisco le variabili necessarie

moveit::planning_interface::MoveGroupInterface *arm_group;
moveit::planning_interface::MoveGroupInterface::Plan *arm_motion_plan;

moveit_msgs::Constraints lab_gripper;

moveit::core::RobotStatePtr current_state;
std::vector<double> joint_group_positions;
const moveit::core::JointModelGroup *joint_model_group;

trajectory_processing::TimeOptimalTrajectoryGeneration *totg;
robot_trajectory::RobotTrajectory *robot_traj;

/**
 * @brief Funzione che inizializza le variabili globali e i puntatori
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
        //      end-effector resolution (precisione nella posizione, 5mm),
        //      jump threshold (valore soglia per evitare i 'salti', movimenti bruschi, dei joint: 5 dai test risulta buono)
        //      path constraints (costrizioni sulla posizione dei joint, passati quelli definiti a priori)
        fraction = arm_group->computeCartesianPath(waypoints, 0.005, 5.0, trajectory, robot_constraints);

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
            totg->computeTimeStamps(*robot_traj, MAX_VELOCITY_SCALING_FACTOR, MAX_ACCELLERATION_SCALING_FACTOR);

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
 * @brief Funzione che esegue il movimento in sequenza del braccio, chiamando le apposite funzioni, per il pick-place
 *        dei blocchi (senza collisioni)
 *
 * @param start posa (X, Y, Z, x, y, z, w) del blocco da prendere (solo x,y e rotazione usati)
 * @param target posa (X, Y, Z, x, y, z, w) in cui rilasciare il blocco (solo x,y e rotazione usati)
 */

void pick_place_simple(geometry_msgs::Pose start, geometry_msgs::Pose target)
{
    // Inizializzo le variabili necessarie
    geometry_msgs::Pose point;
    tf2::Quaternion target_q;

    // Imposto la rotazione di default (gripper con tubo verso il muro posteriore)
    target_q.setX(-1);
    target_q.setY(0);
    target_q.setZ(0);
    target_q.setW(0);
    target_q.normalize();

    // Stampa di cortesia delle posizioni in input
    std::cout << "Destinazione iniziale:\n"
              << std::endl;
    std::cout << "x: " << start.position.x << std::endl;
    std::cout << "y: " << start.position.y << std::endl;
    std::cout << "z: " << start.position.z << std::endl;
    std::cout << "rot x: " << start.orientation.x << std::endl;
    std::cout << "rot y: " << start.orientation.y << std::endl;
    std::cout << "rot z: " << start.orientation.x << std::endl;
    std::cout << "rot w: " << start.orientation.w << std::endl
              << std::endl;

    std::cout << "Destinazione finale:\n"
              << std::endl;
    std::cout << "x: " << target.position.x << std::endl;
    std::cout << "y: " << target.position.y << std::endl;
    std::cout << "z: " << target.position.z << std::endl;
    std::cout << "rot x: " << target.orientation.x << std::endl;
    std::cout << "rot y: " << target.orientation.y << std::endl;
    std::cout << "rot z: " << target.orientation.x << std::endl;
    std::cout << "rot w: " << target.orientation.w << std::endl
              << std::endl;

    // Eseguo il planning verso la posa iniziale
    // con orientazione di default e altezza prefissata
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

    // Aspetto per simulare il gripper
    ros::Duration(3).sleep();

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

    // Eseguo il planning verso la posa finale
    // con orientazione di default (da prima) e altezza prefissata
    point.position = target.position;
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

    // Aspetto per simulare il gripper
    ros::Duration(3).sleep();

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
}

void test(geometry_msgs::Pose target)
{
    arm_group->clearPoseTargets();
    arm_group->setStartStateToCurrentState();
    arm_group->setPoseTarget(target);
    ros::Duration(3).sleep();
    auto start = std::chrono::high_resolution_clock::now();
    bool success = (arm_group->plan(*arm_motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        std::cout << "SUCCESS" << std::endl;
    }
    else
    {
        std::cout << "ERRORE: Planning fallito!" << std::endl;
    }
    auto stop = std::chrono::high_resolution_clock::now();
    ros::Duration(3).sleep();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Tempo di planning: "
              << duration.count() << " millisecondi" << std::endl
              << std::endl;
}

///////////////////////////////////////////////////////////////////////////////

int main(int argc, char **args)
{
    // Inizializzo il nodo
    ros::init(argc, args, "ik_testing");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    setup();

    // Eseguo il planning alla posizione iniziale del braccio (usato solo per Gazebo)
    arm_group->setJointValueTarget(arm_group->getNamedTargetValues("middle_point"));
    arm_group->move();

    // Aggiungo le costrizioni al robot
    robot_constraints();

    std::cout << "Robot pronto a ricevere posizioni" << std::endl
              << std::endl;

    geometry_msgs::Pose start, target;
    tf2::Quaternion q_orientation1, q_orientation2;

    while (ros::ok())
    {
        std::cout << "Inserire la posa di partenza: (decimali delimitati da '.')" << std::endl;
        std::cout << std::endl
                  << "x: ";
        std::cin >> start.position.x;
        std::cout << std::endl
                  << "y: ";
        std::cin >> start.position.y;
        std::cout << std::endl
                  << "z: ";
        std::cin >> start.position.z;
        std::cout << std::endl
                  << "rot x: ";
        std::cin >> start.orientation.x;
        std::cout << std::endl
                  << "rot y: ";
        std::cin >> start.orientation.y;
        std::cout << std::endl
                  << "rot z: ";
        std::cin >> start.orientation.z;
        std::cout << std::endl
                  << "rot w: ";
        std::cin >> start.orientation.w;
        std::cout << std::endl;

        q_orientation1.setX(start.orientation.x);
        q_orientation1.setY(start.orientation.y);
        q_orientation1.setZ(start.orientation.z);
        q_orientation1.setW(start.orientation.w);
        q_orientation1.normalize();

        start.orientation = tf2::toMsg(q_orientation1);

        std::cout << "Inserire la posa di arrivo: (decimali delimitati da '.')" << std::endl;
        std::cout << std::endl
                  << "x: ";
        std::cin >> target.position.x;
        std::cout << std::endl
                  << "y: ";
        std::cin >> target.position.y;
        std::cout << std::endl
                  << "z: ";
        std::cin >> target.position.z;
        std::cout << std::endl
                  << "rot x: ";
        std::cin >> target.orientation.x;
        std::cout << std::endl
                  << "rot y: ";
        std::cin >> target.orientation.y;
        std::cout << std::endl
                  << "rot z: ";
        std::cin >> target.orientation.z;
        std::cout << std::endl
                  << "rot w: ";
        std::cin >> target.orientation.w;
        std::cout << std::endl;

        q_orientation1.setX(target.orientation.x);
        q_orientation1.setY(target.orientation.y);
        q_orientation1.setZ(target.orientation.z);
        q_orientation1.setW(target.orientation.w);
        q_orientation1.normalize();

        target.orientation = tf2::toMsg(q_orientation1);

        std::cout << "Esecuzione in corso..." << std::endl
                  << std::endl;

        pick_place_simple(start, target);
    }

    /*q_orientation1.setX(-1.0);
    q_orientation1.setY(0);
    q_orientation1.setZ(0);
    q_orientation1.setW(0);
    q_orientation1.normalize();

    q_orientation2.setX(-0.7);
    q_orientation2.setY(0.7);
    q_orientation2.setZ(0);
    q_orientation2.setW(0);
    q_orientation2.normalize();

    start.position.x = 0.45;
    start.position.y = 0.40;
    start.position.z = Z_MIN;
    start.orientation = tf2::toMsg(q_orientation1);

    test(start);

    start.position.x = 0.45;
    start.position.y = 0.355;
    start.position.z = Z_MIN;
    start.orientation = tf2::toMsg(q_orientation1);

    test(start);

    start.position.x = 0.45;
    start.position.y = 0.31;
    start.position.z = Z_MIN;
    start.orientation = tf2::toMsg(q_orientation1);

    test(start);

    start.position.x = 0.45;
    start.position.y = 0.265;
    start.position.z = Z_MIN;
    start.orientation = tf2::toMsg(q_orientation1);

    test(start);

    start.position.x = 0.45;
    start.position.y = 0.22;
    start.position.z = Z_MIN;
    start.orientation = tf2::toMsg(q_orientation1);

    test(start);

    start.position.x = 0.45;
    start.position.y = 0.175;
    start.position.z = Z_MIN;
    start.orientation = tf2::toMsg(q_orientation1);

    test(start);

    start.position.x = 0.45;
    start.position.y = 0.13;
    start.position.z = Z_MIN;
    start.orientation = tf2::toMsg(q_orientation1);

    test(start);

    start.position.x = 0.45;
    start.position.y = 0.085;
    start.position.z = Z_MIN;
    start.orientation = tf2::toMsg(q_orientation1);

    test(start);

    start.position.x = 0.45;
    start.position.y = 0.04;
    start.position.z = Z_MIN;
    start.orientation = tf2::toMsg(q_orientation1);

    test(start);

    start.position.x = 0.45;
    start.position.y = -0.005;
    start.position.z = Z_MIN;
    start.orientation = tf2::toMsg(q_orientation1);

    test(start);

    start.position.x = 0.37;
    start.position.y = 0.40;
    start.position.z = Z_MIN;
    start.orientation = tf2::toMsg(q_orientation1);

    test(start);

    start.position.x = 0.37;
    start.position.y = 0.355;
    start.position.z = Z_MIN;
    start.orientation = tf2::toMsg(q_orientation1);

    test(start);

    start.position.x = 0.37;
    start.position.y = 0.31;
    start.position.z = Z_MIN;
    start.orientation = tf2::toMsg(q_orientation1);

    test(start);

    start.position.x = 0.37;
    start.position.y = 0.265;
    start.position.z = Z_MIN;
    start.orientation = tf2::toMsg(q_orientation1);

    test(start);

    start.position.x = 0.37;
    start.position.y = 0.22;
    start.position.z = Z_MIN;
    start.orientation = tf2::toMsg(q_orientation1);

    test(start);

    start.position.x = 0.37;
    start.position.y = 0.175;
    start.position.z = Z_MIN;
    start.orientation = tf2::toMsg(q_orientation1);

    test(start);

    start.position.x = 0.37;
    start.position.y = 0.13;
    start.position.z = Z_MIN;
    start.orientation = tf2::toMsg(q_orientation1);

    test(start);

    start.position.x = 0.37;
    start.position.y = 0.085;
    start.position.z = Z_MIN;
    start.orientation = tf2::toMsg(q_orientation1);

    test(start);

    start.position.x = 0.37;
    start.position.y = 0.04;
    start.position.z = Z_MIN;
    start.orientation = tf2::toMsg(q_orientation1);

    test(start);

    start.position.x = 0.37;
    start.position.y = -0.005;
    start.position.z = Z_MIN;
    start.orientation = tf2::toMsg(q_orientation1);

    test(start);*/
}