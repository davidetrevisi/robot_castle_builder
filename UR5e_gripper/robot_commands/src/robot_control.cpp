/**
 * @file robot_control.cpp
 * @author Davide Trevisi
 * @brief Ros node that receive commands from the commands/Position node and send motion command to the robot
 * @version 1
 * @date 2022-07-17
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <signal.h>

#include <gazebo_ros_link_attacher/Attach.h>
#include <gazebo_ros_link_attacher/AttachRequest.h>
#include <gazebo_ros_link_attacher/AttachResponse.h>

#include <ros/ros.h>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <castle_builder/Positions.h>

#define SIMULATION true
#define Z_BASE_LINK 1.79
#define Z_DESK 0.867
#define Z_MIN 1.02
#define Z_TOP 1.07

castle_builder::Positions pos_msg;
bool position_flag = false;

using namespace std;
using namespace Eigen;
using namespace ros;
using namespace boost;

// Define moveit groups' names

static const std::string PLANNING_GROUP_ARM = "manipulator";
moveit::planning_interface::MoveGroupInterface *arm_group;
moveit::planning_interface::MoveGroupInterface::Plan *arm_motion_plan;

// Gripper moveit groups in simulated environment

static const std::string PLANNING_GROUP_GRIPPER = "endeffector";
moveit::planning_interface::MoveGroupInterface *gripper_group;
moveit::planning_interface::MoveGroupInterface::Plan *gripper_motion_plan;

// Collision related parameters

int cube_index = 0;
int parallelepiped_index = 0;

moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
moveit_msgs::CollisionObject *cube_collision[30], *parallelepiped_collision[30];
shape_msgs::SolidPrimitive cube_primitive, parallelepiped_primitive;
geometry_msgs::Pose cube_pose, parallelepiped_pose;

void printGREEN(string s)
{
    cout << "\033[1;92m" << s << "\033[0m\n";
}

/**
 * @brief Initialize pointers, global variables and collision objects
 *
 */

void setup()
{
    // Initialize groups and plans

    arm_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM);
    arm_group->setPlanningTime(20.0);
    arm_group->setMaxVelocityScalingFactor(0.1);
    arm_group->setMaxAccelerationScalingFactor(0.1);
    arm_motion_plan = new moveit::planning_interface::MoveGroupInterface::Plan();

    if (SIMULATION)
    {
        gripper_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GRIPPER);
        gripper_group->setPlanningTime(5.0);
        gripper_group->setMaxVelocityScalingFactor(0.1);
        gripper_group->setMaxAccelerationScalingFactor(0.1);
        gripper_motion_plan = new moveit::planning_interface::MoveGroupInterface::Plan();
    }

    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();

    // Defining cube's collision
    cube_primitive.type = cube_primitive.BOX;
    cube_primitive.dimensions.resize(3);
    cube_primitive.dimensions[0] = 0.025;
    cube_primitive.dimensions[1] = 0.025;
    cube_primitive.dimensions[2] = 0.025;

    // Defining parallelepiped's collision
    parallelepiped_primitive.type = parallelepiped_primitive.BOX;
    parallelepiped_primitive.dimensions.resize(3);
    parallelepiped_primitive.dimensions[0] = 0.025;
    parallelepiped_primitive.dimensions[1] = 0.05;
    parallelepiped_primitive.dimensions[2] = 0.025;
}

/**
 * @brief Add cube collisions in planning scene and link to gripper
 *
 * @param pose pose for the object to add
 * @param gripper if true attach to gripper, otherwise attach to world
 * @param index index of the object to add
 */

void add_cube_collision(geometry_msgs::Pose pose, bool gripper, int index)
{
    // Define element and pose

    cube_collision[index] = new moveit_msgs::CollisionObject();
    cube_collision[index]->id = "cube_collision_" + index;
    cube_collision[index]->header.frame_id = arm_group->getPlanningFrame();

    cube_collision[index]->primitives.push_back(cube_primitive);
    cube_collision[index]->primitive_poses.push_back(pose);
    cube_collision[index]->operation = cube_collision[index]->ADD;

    // Add cube to scene

    if (gripper)
    {
        planning_scene_interface->applyCollisionObject(*cube_collision[index]);
        arm_group->attachObject(cube_collision[index]->id, "wrist_3_link");
    }
    else
    {
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.push_back(*cube_collision[index]);
        planning_scene_interface->applyCollisionObjects(collision_objects);
    }
}

/**
 * @brief Add parallelepiped collisions in planning scene and link to gripper
 *
 * @param pose pose for the object to add
 * @param gripper if true attach to gripper, otherwise attach to world
 * @param index index of the object to add
 */

void add_parallelepiped_collision(geometry_msgs::Pose pose, bool gripper, int index)
{
    // Define element and pose

    parallelepiped_collision[index] = new moveit_msgs::CollisionObject();
    parallelepiped_collision[index]->id = "parallelepiped_collision_" + index;
    parallelepiped_collision[index]->header.frame_id = arm_group->getPlanningFrame();

    parallelepiped_collision[index]->primitives.push_back(parallelepiped_primitive);
    parallelepiped_collision[index]->primitive_poses.push_back(pose);
    parallelepiped_collision[index]->operation = parallelepiped_collision[index]->ADD;

    // Add cube to scene

    if (gripper)
    {
        planning_scene_interface->applyCollisionObject(*parallelepiped_collision[index]);
        arm_group->attachObject(parallelepiped_collision[index]->id, "wrist_3_link");
    }
    else
    {
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.push_back(*parallelepiped_collision[index]);
        planning_scene_interface->applyCollisionObjects(collision_objects);
    }
}

/**
 * @brief Remove cube collisions in planning scene and unlink gripper
 *
 * @param gripper if true detach from gripper, otherwise remove from world
 * @param index index of the object to remove
 */

void remove_cube_collision(bool gripper, int index)
{
    if (gripper)
    {
        arm_group->detachObject(cube_collision[index]->id);
    }
    else
    {
        std::vector<std::string> object_ids;
        object_ids.push_back(cube_collision[index]->id);
        planning_scene_interface->removeCollisionObjects(object_ids);
    }
}

/**
 * @brief Remove parallelepiped collisions in planning scene and unlink gripper
 *
 * @param gripper if true detach from gripper, otherwise remove from world
 * @param index index of the object to remove
 */

void remove_parallelepiped_collision(bool gripper, int index)
{
    if (gripper)
    {
        arm_group->detachObject(parallelepiped_collision[index]->id);
    }
    else
    {
        std::vector<std::string> object_ids;
        object_ids.push_back(parallelepiped_collision[index]->id);
        planning_scene_interface->removeCollisionObjects(object_ids);
    }
}

/**
 * @brief [DEV] Function that set path and joint constraints for the robot
 *
 */

void robot_constraints()
{
    moveit_msgs::JointConstraint shoulder_pan_jcm, shoulder_lift_jcm, elbow_jcm, wrist_1_jcm, wrist_2_jcm, wrist_3_jcm;
    moveit_msgs::OrientationConstraint wrist_3_ocm;

    shoulder_pan_jcm.joint_name = "shoulder_pan_joint";
    shoulder_pan_jcm.position = -M_PI_2;
    shoulder_pan_jcm.tolerance_above = M_PI_2;
    shoulder_pan_jcm.tolerance_below = M_PI_2;
    shoulder_pan_jcm.weight = 1.0;

    shoulder_lift_jcm.joint_name = "shoulder_lift_joint";
    shoulder_lift_jcm.position = -0.0356;
    shoulder_lift_jcm.tolerance_above = 0.04;
    shoulder_lift_jcm.tolerance_below = 1;
    shoulder_lift_jcm.weight = 1.0;

    elbow_jcm.joint_name = "elbow_joint";
    elbow_jcm.position = -1.7728;
    elbow_jcm.tolerance_above = 0.9;
    elbow_jcm.tolerance_below = 0.9;
    elbow_jcm.weight = 1.0;

    wrist_1_jcm.joint_name = "wrist_1_joint";
    wrist_1_jcm.position = 0.2242;
    wrist_1_jcm.tolerance_above = M_PI_4;
    wrist_1_jcm.tolerance_below = M_PI_4;
    wrist_1_jcm.weight = 1.0;

    wrist_2_jcm.joint_name = "wrist_2_joint";
    wrist_2_jcm.position = M_PI_2;
    wrist_2_jcm.tolerance_above = M_PI_4;
    wrist_2_jcm.tolerance_below = M_PI_4;
    wrist_2_jcm.weight = 1.0;

    wrist_3_jcm.joint_name = "wrist_3_joint";
    wrist_3_jcm.position = 0;
    wrist_3_jcm.tolerance_above = 0;
    wrist_3_jcm.tolerance_below = M_PI;
    wrist_3_jcm.weight = 1.0;

    wrist_3_ocm.link_name = "wrist_3_link";
    wrist_3_ocm.header.frame_id = "world";
    wrist_3_ocm.orientation.y = 1;
    wrist_3_ocm.absolute_x_axis_tolerance = 0.02;
    wrist_3_ocm.absolute_y_axis_tolerance = 0.01;
    wrist_3_ocm.absolute_z_axis_tolerance = M_PI;
    wrist_3_ocm.weight = 1.0;

    moveit_msgs::Constraints robot_constraints;

    /*robot_constraints.joint_constraints.push_back(shoulder_pan_jcm);
    robot_constraints.joint_constraints.push_back(shoulder_lift_jcm);
    robot_constraints.joint_constraints.push_back(elbow_jcm);
    robot_constraints.joint_constraints.push_back(wrist_1_jcm);
    robot_constraints.joint_constraints.push_back(wrist_2_jcm);
    robot_constraints.joint_constraints.push_back(wrist_3_jcm);*/
    robot_constraints.orientation_constraints.push_back(wrist_3_ocm);

    arm_group->clearPathConstraints();
    arm_group->setPathConstraints(robot_constraints);
}

/**
 * @brief Execute cartesian path given the target position. If no cartesian solution are found, function return immediately otherwise EE will start moving
 *
 * @param target desired target pose (X,Y,Z,x,y,z,w) of EE
 */

void execute_Cartesian_Path(geometry_msgs::Pose target)
{
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose current_pose = (arm_group->getCurrentPose()).pose;
    waypoints.push_back(target);

    cout << "\033[1;34mMoving towards:\033[0m\n"
         << endl;
    cout << "\033[1;34mx: " << target.position.x << "\033[0m" << endl;
    cout << "\033[1;34my: " << target.position.y << "\033[0m" << endl;
    cout << "\033[1;34mz: " << target.position.z << "\033[0m" << endl;

    moveit_msgs::RobotTrajectory trajectory;

    // waypoints, EE resolution 1cm, jump threshold 0.0, trajectory (0.0 = disabled ONLY FOR SIMULATION)
    double fraction = arm_group->computeCartesianPath(waypoints, 0.01, 0, trajectory);

    if (fraction < 1)
    {
        cout << "Trajectory error!" << endl;
        ros::Duration(1).sleep();
    }

    if (fraction == 1)
    {
        arm_motion_plan = new moveit::planning_interface::MoveGroupInterface::Plan();
        arm_motion_plan->trajectory_ = trajectory;
        arm_group->execute(*arm_motion_plan);
    }
}

/**
 * @brief Function that set the robot target position and execute the motion
 *
 * @param target set a target position for the EE
 */

void motion_plan(geometry_msgs::Pose target)
{
    cout << "\033[1;34mMoving towards:\033[0m\n"
         << endl;
    cout << "\033[1;34mx: " << target.position.x << "\033[0m" << endl;
    cout << "\033[1;34my: " << target.position.y << "\033[0m" << endl;
    cout << "\033[1;34mz: " << target.position.z << "\033[0m" << endl;
    arm_group->setPoseTarget(target);

    bool success = (arm_group->plan(*arm_motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        printGREEN("Motion plan updated!");
        cout << endl
             << "Motion plan is now executing!";
        arm_group->move();
    }
    else
    {
        printGREEN("Error: Motion plan failed! Computing cartesian path instead");
        execute_Cartesian_Path(target);
    }
}

/**
 * @brief Open gripper in gazebo (and unlink object to grasp) or in real life
 *
 * @param pub Ros client that call gripper_controller_node for sending cmd to gripper
 * @param client Ros service client used to re-establish connection to robot after it drops due to msg sending
 * @param model name of the model of the object to detach from the EE (cube-parallelepiped)
 */

void open_gripper(ros::Publisher pub, ros::ServiceClient client, string model)
{
    if (SIMULATION)
    {
        gripper_group->setJointValueTarget(gripper_group->getNamedTargetValues("open"));
        bool success = (gripper_group->plan(*gripper_motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        printGREEN("Opening gripper");
        gripper_group->move();

        gazebo_ros_link_attacher::Attach detach;

        detach.request.link_name_1 = "link";
        detach.request.model_name_1 = model;
        detach.request.link_name_2 = "wrist_3_link";
        detach.request.model_name_2 = "robot";

        client.call(detach);
    }
    else
    {
        std_msgs::String msg;
        std::stringstream ss;

        ss << "open";
        msg.data = ss.str();
        cout << "Sending opening msg to gripper_cmd " << msg.data.c_str();
        pub.publish(msg);
        ros::Duration(1.3).sleep();
        std_srvs::Trigger srv;
        client.call(srv);
    }
}

/**
 * @brief Close gripper in gazebo (and link object to grasp) or in real life
 *
 * @param pub Ros client that call gripper_controller_node for sending cmd to gripper
 * @param client Ros service client used to re-establish connection to robot after it drops due to msg sending
 * @param model name of the model of the object to attach to the EE (cube-parallelepiped)
 */

void close_gripper(ros::Publisher pub, ros::ServiceClient client, string model)
{
    if (SIMULATION)
    {
        gripper_group->setJointValueTarget(gripper_group->getNamedTargetValues("close"));
        bool success = (gripper_group->plan(*gripper_motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        printGREEN("Closing gripper");
        gripper_group->move();

        gazebo_ros_link_attacher::Attach attach;

        attach.request.link_name_1 = "link";
        attach.request.model_name_1 = model;
        attach.request.link_name_2 = "wrist_3_link";
        attach.request.model_name_2 = "robot";

        client.call(attach);
    }
    else
    {
        std_msgs::String msg;
        std::stringstream ss;

        ss << "close";
        msg.data = ss.str();
        cout << "Sending closing msg to gripper_cmd " << msg.data.c_str();
        pub.publish(msg);
        ros::Duration(1.3).sleep();
        std_srvs::Trigger srv;
        client.call(srv);
    }
}

/**
 * @brief Populate information about the initial and final positions of the block to pick
 * and the block type from the received message
 *
 * @param msg Msg that come from the castle_builder node
 */

void position_callback(const castle_builder::Positions::ConstPtr &msg)
{
    for (int i = 0; i < 3; i++)
    {
        pos_msg.block_pos[i] = msg->block_pos[i];
        pos_msg.block_rpy[i] = msg->block_rpy[i];
        pos_msg.target_pos[i] = msg->target_pos[i];
        pos_msg.target_rpy[i] = msg->target_rpy[i];
        pos_msg.cube = msg->cube;
    }
    position_flag = true;
}

/**
 * @brief Demo of the motion and gripping
 *
 * @param client Ros client that call gripper_controller_node for sending cmd to gripper
 * @param gripper_pub
 */

void pick_place(ros::ServiceClient client, ros::Publisher gripper_pub)
{
    geometry_msgs::Pose target, object;
    vector<double> current_rpy;
    tf2::Quaternion target_q;

    // Go to pick object
    target.position.x = pos_msg.block_pos[0];
    cout << "x::" << target.position.x;
    target.position.y = pos_msg.block_pos[1];
    cout << "y::" << target.position.y;
    target.position.z = Z_TOP;

    // Orientation

    /*
    target_q.setX(-1);
    target_q.setY(0);
    target_q.setZ(0);
    target_q.setW(0);
    target_q.normalize();

    target.orientation = tf2::toMsg(target_q);*/

    motion_plan(target);
    ros::Duration(0.5).sleep();

    // Lower EE
    target.position.z = Z_MIN;
    execute_Cartesian_Path(target);
    ros::Duration(0.5).sleep();

    close_gripper(gripper_pub, client, "cubo");
    ros::Duration(0.5).sleep();

    // Add object to gripper collision
    object.position.x = target.position.x;
    object.position.y = target.position.y;
    object.position.z = Z_DESK + 0.025 / 2;
    object.orientation.x = -1.0;

    if (pos_msg.cube)
    {
        add_cube_collision(object, true, cube_index);
    }
    else
    {
        add_parallelepiped_collision(object, true, cube_index);
    }

    // Raise EE
    target.position.z = Z_TOP;
    execute_Cartesian_Path(target);
    ros::Duration(0.5).sleep();

    // Go to target location
    target.position.x = pos_msg.target_pos[0];
    target.position.y = pos_msg.target_pos[1];
    target.position.z = pos_msg.target_pos[2] + 0.5;

    /*target_q.setX(-1);
    target_q.setY(0);
    target_q.setZ(0);
    target_q.setW(0);
    target_q.normalize();

    target.orientation = tf2::toMsg(target_q);*/

    motion_plan(target);
    ros::Duration(0.5).sleep();

    // Lower EE
    target.position.z = Z_MIN + 0.01;
    execute_Cartesian_Path(target);
    ros::Duration(0.5).sleep();

    open_gripper(gripper_pub, client, "cubo");
    ros::Duration(0.5).sleep();

    // Remove object from gripper collision
    if (pos_msg.cube)
    {
        remove_cube_collision(true, cube_index);
    }
    else
    {
        remove_parallelepiped_collision(true, parallelepiped_index);
    }

    // Add object to world collision
    object.position.x = pos_msg.target_pos[0];
    object.position.y = pos_msg.target_pos[1];
    object.position.z = pos_msg.target_pos[2] + 0.025 / 2;
    object.orientation.x = -1.0;

    if (pos_msg.cube)
    {
        add_cube_collision(object, false, cube_index);
    }
    else
    {
        add_parallelepiped_collision(object, false, cube_index);
    }

    // Raise EE
    target.position.z = pos_msg.target_pos[2] + 0.5;
    execute_Cartesian_Path(target);
    ros::Duration(0.5).sleep();

    // Update index
    if (pos_msg.cube)
    {
        cube_index++;
    }
    else
    {
        parallelepiped_index++;
    }
}

/**
 * @brief Unload collision from moveit when node is stopped by signal
 *
 * @param sig Signal number
 */

void removeCollision(int sig)
{
    cout << endl
         << "Deleting collisions";

    // Remove object from world collision
    for (int i = 1; i < cube_index; i++)
    {
        remove_cube_collision(false, i - 1);
    }

    for (int i = 1; i < parallelepiped_index; i++)
    {
        remove_parallelepiped_collision(false, i - 1);
    }

    ros::shutdown();
}

/////////////////////////

int main(int argc, char **args)
{
    ros::init(argc, args, "move_group_interface");
    ros::NodeHandle n;
    signal(SIGINT, removeCollision);
    ros::AsyncSpinner spinner(1);
    ros::Publisher gripper_pub = n.advertise<std_msgs::String>("gripper_controller_cmd", 10);
    spinner.start();
    setup();

    // Going to home position
    arm_group->setJointValueTarget(arm_group->getNamedTargetValues("home"));
    bool success = (arm_group->plan(*arm_motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    arm_group->move();

    ros::ServiceClient attach_srv, detach_srv, client;

    if (SIMULATION)
    {
        attach_srv = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
        detach_srv = n.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
    }
    else
    {
        client = n.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/resend_robot_program");
    }

    ros::Subscriber sub = n.subscribe("/node/castle_builder_node", 1, position_callback);

    // Add joint constraints to the planner (currently in development)
    robot_constraints();

    printGREEN("Ready");

    while (ros::ok())
    {
        if (position_flag)
        {
            pick_place(client, gripper_pub);
            position_flag = false;
        }
    }
}