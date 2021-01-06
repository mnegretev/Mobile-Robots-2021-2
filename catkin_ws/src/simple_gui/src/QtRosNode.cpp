#include "QtRosNode.h"

QtRosNode::QtRosNode()
{
    this->gui_closed = false;
    publishing_cmd_vel = false;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
}

QtRosNode::~QtRosNode()
{
}

void QtRosNode::run()
{    
    ros::Rate loop(30);
    pubCmdVel   = n->advertise<geometry_msgs::Twist      >("/cmd_vel", 10);
    pubGoToXYA  = n->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    cltBFS      = n->serviceClient<nav_msgs::GetPlan>("/navigation/path_planning/breadth_first_search");
    cltDFS      = n->serviceClient<nav_msgs::GetPlan>("/navigation/path_planning/depth_first_search");
    cltDijkstra = n->serviceClient<nav_msgs::GetPlan>("/navigation/path_planning/dijkstra_search");
    cltAStar    = n->serviceClient<nav_msgs::GetPlan>("/navigation/path_planning/a_star_search");  

    int pub_zero_counter = 5;
    while(ros::ok() && !this->gui_closed)
    {
        if(publishing_cmd_vel)
        {
            pubCmdVel.publish(cmd_vel);
            pub_zero_counter = 5;
        }
        else if(--pub_zero_counter > 0)
        {
            if(pub_zero_counter <= 0)
                pub_zero_counter = 0;
            pubCmdVel.publish(cmd_vel);
        }
        ros::spinOnce();
        emit updateGraphics();
        loop.sleep();
    }
    emit onRosNodeFinished();
}

void QtRosNode::setNodeHandle(ros::NodeHandle* nh)
{
    this->n = nh;
}

void QtRosNode::publish_cmd_vel(float linear_frontal, float linear_lateral, float angular)
{
    cmd_vel.linear.x = linear_frontal;
    cmd_vel.linear.y = linear_lateral;
    cmd_vel.angular.z = angular;
    pubCmdVel.publish(cmd_vel);
}

void QtRosNode::start_publishing_cmd_vel(float linear_frontal, float linear_lateral, float angular)
{
    cmd_vel.linear.x = linear_frontal;
    cmd_vel.linear.y = linear_lateral;
    cmd_vel.angular.z = angular;
    publishing_cmd_vel = true;
}

void QtRosNode::stop_publishing_cmd_vel()
{
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
    publishing_cmd_vel = false;
}

bool QtRosNode::call_breadth_first_search(float start_x, float start_y, float goal_x, float goal_y)
{
    nav_msgs::GetPlan srv;
    srv.request.start.pose.position.x = start_x;
    srv.request.start.pose.position.y = start_y;
    srv.request.goal.pose.position.x = goal_x;
    srv.request.goal.pose.position.y = goal_y;
    return cltBFS.call(srv);
}

bool QtRosNode::call_depth_first_search(float start_x, float start_y, float goal_x, float goal_y)
{
    nav_msgs::GetPlan srv;
    srv.request.start.pose.position.x = start_x;
    srv.request.start.pose.position.y = start_y;
    srv.request.goal.pose.position.x = goal_x;
    srv.request.goal.pose.position.y = goal_y;
    return cltDFS.call(srv);
}

bool QtRosNode::call_dijkstra_search(float start_x, float start_y, float goal_x, float goal_y)
{
    nav_msgs::GetPlan srv;
    srv.request.start.pose.position.x = start_x;
    srv.request.start.pose.position.y = start_y;
    srv.request.goal.pose.position.x = goal_x;
    srv.request.goal.pose.position.y = goal_y;
    return cltDijkstra.call(srv);
}

bool QtRosNode::call_a_star_search(float start_x, float start_y, float goal_x, float goal_y)
{
    nav_msgs::GetPlan srv;
    srv.request.start.pose.position.x = start_x;
    srv.request.start.pose.position.y = start_y;
    srv.request.goal.pose.position.x = goal_x;
    srv.request.goal.pose.position.y = goal_y;
    return cltAStar.call(srv);
}

void QtRosNode::publish_goto_xya(float goal_x, float goal_y, float goal_a)
{
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = goal_x;
    msg.pose.position.y = goal_y;
    msg.pose.orientation.w = cos(goal_a/2);
    msg.pose.orientation.z = sin(goal_a/2);
    pubGoToXYA.publish(msg);
}

void QtRosNode::set_param_control_type(std::string control_type)
{
    n->setParam("/navigation/control_type/", control_type);
}

void QtRosNode::set_param_inflation_radius(float inflation_radius)
{
    n->setParam("/navigation/path_planning/inflation_radius", inflation_radius);
}

void QtRosNode::set_param_cost_radius(float cost_radius)
{
    n->setParam("/navigation/path_planning/cost_radius",  cost_radius);
}

void QtRosNode::set_param_smoothing_alpha(float smoothing_alpha)
{
    n->setParam("/navigation/path_planning/smoothing_alpha",  smoothing_alpha);
}
  
void QtRosNode::set_param_smoothing_beta(float  smoothing_beta)
{
    n->setParam("/navigation/path_planning/smoothing_beta" ,  smoothing_beta);
}

void QtRosNode::get_robot_pose(float& robot_x, float& robot_y, float& robot_a)
{
    tf::StampedTransform t;
    tf::Quaternion q;
    tf_listener.waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(0.5));
    tf_listener.lookupTransform("map", "base_link", ros::Time(0), t);
    robot_x = t.getOrigin().x();
    robot_y = t.getOrigin().y();
    q = t.getRotation();
    robot_a = atan2(q.z(), q.w())*2;
}
