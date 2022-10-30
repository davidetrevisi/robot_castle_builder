/**
 * @file add_collisions.cpp
 * @author Davide Trevisi
 * @brief ROS Node che aggiunge le collisioni del workspace del robot all'ambiente MoveIt!
 * @version 1
 * @date 2022-07-21
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit_msgs/CollisionObject.h>

#include <ros/ros.h>

#include <signal.h>

// Definisco le costanti necessarie

#define Z_BASE_LINK 1.791
#define Z_DESK 0.8675

static const std::string PLANNING_GROUP_ARM = "manipulator";

// Definisco le variabili necessarie

moveit::planning_interface::MoveGroupInterface *arm_group;
moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
moveit_msgs::CollisionObject *desk;
moveit_msgs::CollisionObject *electric_panel;
moveit_msgs::CollisionObject *wall;
moveit_msgs::CollisionObject *front_wall;
moveit_msgs::CollisionObject *top_plate;
moveit_msgs::CollisionObject *plug;

/**
 * @brief Funzione che inizializza e carica le collisioni nella scena MoveIt!
 *
 */

void addCollisions()
{
    // Inizializzo e creo le variabili necessarie agli oggetti collisione
    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();

    desk = new moveit_msgs::CollisionObject();
    electric_panel = new moveit_msgs::CollisionObject();
    wall = new moveit_msgs::CollisionObject();
    front_wall = new moveit_msgs::CollisionObject();
    top_plate = new moveit_msgs::CollisionObject();
    plug = new moveit_msgs::CollisionObject();

    // Assegno il frame alle collision
    desk->header.frame_id = arm_group->getPlanningFrame();
    electric_panel->header.frame_id = arm_group->getPlanningFrame();
    wall->header.frame_id = arm_group->getPlanningFrame();
    front_wall->header.frame_id = arm_group->getPlanningFrame();
    top_plate->header.frame_id = arm_group->getPlanningFrame();
    plug->header.frame_id = arm_group->getPlanningFrame();

    // Assegno l'ID alle collision
    desk->id = "desk";
    electric_panel->id = "electric_panel";
    wall->id = "wall";
    front_wall->id = "front_wall";
    top_plate->id = "top_plate";
    plug->id = "plug";

    // Creo le mesh per gli oggetti
    shape_msgs::SolidPrimitive desk_primitive, electric_panel_primitive,
        wall_primitive, top_plate_primitive, electric_plug_primitive;

    // Definisco la collisione del tavolo
    desk_primitive.type = desk_primitive.BOX;
    desk_primitive.dimensions.resize(3);
    desk_primitive.dimensions[0] = 0.85;
    desk_primitive.dimensions[1] = 1;
    desk_primitive.dimensions[2] = Z_DESK;

    // Definisco la collisione del pannello elettrico posteriore
    electric_panel_primitive.type = electric_panel_primitive.BOX;
    electric_panel_primitive.dimensions.resize(3);
    electric_panel_primitive.dimensions[0] = 0.2;
    electric_panel_primitive.dimensions[1] = 1;
    electric_panel_primitive.dimensions[2] = 0.175;

    // Definisco la collisione del muro
    wall_primitive.type = wall_primitive.BOX;
    wall_primitive.dimensions.resize(3);
    wall_primitive.dimensions[0] = 0.05;
    wall_primitive.dimensions[1] = 1;
    wall_primitive.dimensions[2] = 1.841;

    // Definisco la collisione del supporto superiore
    top_plate_primitive.type = top_plate_primitive.BOX;
    top_plate_primitive.dimensions.resize(3);
    top_plate_primitive.dimensions[0] = 0.90;
    top_plate_primitive.dimensions[1] = 1;
    top_plate_primitive.dimensions[2] = 0.05;

    // Definisco la collisione della presa elettrica
    electric_plug_primitive.type = electric_plug_primitive.BOX;
    electric_plug_primitive.dimensions.resize(3);
    electric_plug_primitive.dimensions[0] = 0.1;
    electric_plug_primitive.dimensions[1] = 0.2;
    electric_plug_primitive.dimensions[2] = 0.175;

    // Creo le pose per gli oggetti
    geometry_msgs::Pose desk_pose, electric_panel_pose, wall_pose,
        front_wall_pose, top_plate_pose, plug_pose;

    // Definisco la posa del tavolo
    desk_pose.orientation.w = -0.707;
    desk_pose.orientation.z = 0.707;
    desk_pose.position.x = -0.015;
    desk_pose.position.y = 0.05;
    desk_pose.position.z = Z_DESK / 2;

    // Definisco la posa del pannello elettrico posteriore
    electric_panel_pose.orientation.w = -0.707;
    electric_panel_pose.orientation.z = 0.707;
    electric_panel_pose.position.x = -0.015;
    electric_panel_pose.position.y = -0.275;
    electric_panel_pose.position.z = Z_DESK + 0.175 / 2;

    // Definisco la posa del muro posteriore
    wall_pose.orientation.z = 0.707;
    wall_pose.orientation.w = -0.707;
    wall_pose.position.x = -0.015;
    wall_pose.position.y = -0.40;
    wall_pose.position.z = 0.918;

    // Definisco la posa del muro anteriore
    front_wall_pose.orientation.z = 0.707;
    front_wall_pose.orientation.w = -0.707;
    front_wall_pose.position.x = -0.015;
    front_wall_pose.position.y = 0.50;
    front_wall_pose.position.z = 0.918;

    // Definisco la posa del supporto superiore
    top_plate_pose.orientation.z = 0.707;
    top_plate_pose.orientation.w = -0.707;
    top_plate_pose.position.x = -0.015;
    top_plate_pose.position.y = 0.025;
    top_plate_pose.position.z = Z_BASE_LINK + 0.05 / 2;

    // Definisco la posa della presa elettrica
    plug_pose.orientation.z = 0.707;
    plug_pose.orientation.w = -0.707;
    plug_pose.position.x = -0.415;
    plug_pose.position.y = -0.12;
    plug_pose.position.z = Z_DESK + 0.175 / 2;

    // Aggiungo le pose e le mesh agli oggetti
    desk->primitives.push_back(desk_primitive);
    desk->primitive_poses.push_back(desk_pose);
    desk->operation = desk->ADD;

    electric_panel->primitives.push_back(electric_panel_primitive);
    electric_panel->primitive_poses.push_back(electric_panel_pose);
    electric_panel->operation = electric_panel->ADD;

    wall->primitives.push_back(wall_primitive);
    wall->primitive_poses.push_back(wall_pose);
    wall->operation = wall->ADD;

    front_wall->primitives.push_back(wall_primitive);
    front_wall->primitive_poses.push_back(front_wall_pose);
    front_wall->operation = wall->ADD;

    top_plate->primitives.push_back(top_plate_primitive);
    top_plate->primitive_poses.push_back(top_plate_pose);
    top_plate->operation = top_plate->ADD;

    plug->primitives.push_back(electric_plug_primitive);
    plug->primitive_poses.push_back(plug_pose);
    plug->operation = plug->ADD;

    // Aggiungo e applico gli oggetti all'interfaccia di planning di MoveIt!
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    collision_objects.push_back(*desk);
    collision_objects.push_back(*electric_panel);
    collision_objects.push_back(*wall);
    collision_objects.push_back(*front_wall);
    collision_objects.push_back(*top_plate);
    collision_objects.push_back(*plug);

    planning_scene_interface->applyCollisionObjects(collision_objects);
}

/**
 * @brief Funzione che rimuove le collisioni dalla scena MoveIt!
 *
 * @param sig numero del segnale
 */

void removeCollision(int sig)
{
    std::cout << std::endl
              << "Rimuovo le collisioni"
              << std::endl;

    // Rimuovo le collisioni e termino il programma
    std::vector<std::string> object_ids;
    object_ids.push_back(desk->id);
    object_ids.push_back(electric_panel->id);
    object_ids.push_back(wall->id);
    object_ids.push_back(front_wall->id);
    object_ids.push_back(top_plate->id);
    object_ids.push_back(plug->id);

    planning_scene_interface->removeCollisionObjects(object_ids);
    ros::shutdown();
}

///////////////////////////////////////////////////////////////////////////////

int main(int argc, char **args)
{
    // Inizializzo il nodo
    ros::init(argc, args, "collision_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    signal(SIGINT, removeCollision);

    arm_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM);

    // Aggiungo le collisioni
    addCollisions();

    // Aspetto l'uscita
    while (ros::ok())
    {
        ros::Duration(1).sleep();
    }
}