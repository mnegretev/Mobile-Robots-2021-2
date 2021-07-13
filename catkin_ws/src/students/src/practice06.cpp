/*
 * AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2021-2
 * PRACTICE 6 - LOCALIZATION BY PARTICLE FILTERS - PART I
 *
 * Instructions:
 * Write the code necessary to implement localization by particle filters.
 * Modify only the sections marked with the TODO comment. 
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"
#include "random_numbers/random_numbers.h"
#include "occupancy_grid_utils/ray_tracer.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/transform_broadcaster.h"

#define NOMBRE "ALVARADO_RAMOS"

#define LASER_DOWNSAMPLING  10
#define SENSOR_NOISE        0.1
#define RESAMPLING_NOISE    0.1
#define MOVEMENT_NOISE      0.1
#define DISTANCE_THRESHOLD  0.2
#define ANGLE_THRESHOLD     0.2

sensor_msgs::LaserScan real_sensor_info;
sensor_msgs::LaserScan real_scan;

geometry_msgs::PoseArray get_initial_distribution(int N, float min_x, float max_x, float min_y, float max_y, float min_a, float max_a)
{
    random_numbers::RandomNumberGenerator rnd;
    geometry_msgs::PoseArray particles;
    particles.poses.resize(N);
    particles.header.frame_id = "map";

    /*
     * TODO:
     *
     * Generate a set of N particles (each particle represented by a Pose message)
     * with positions uniformly distributed within bounding box given by min_x, ..., max_a.
     * The set of particles must be a PoseArray message.
     * To generate uniformly distributed random numbers, you can use the funcion rnd.uniformReal(min, max)
     * Remember that orientation in a Pose message is represented by a quaternion (x,y,z,w)
     * For the Euler angles (roll, pitch, yaw) = (0,0,theta) the corresponding quaternion is
     * given by (0,0,sin(theta/2), cos(theta/2)). 
     */
    for(size_t i=0; i<N; i++){
    	particles.poses[i].position.x=rnd.uniformReal(min_x, max_x);
    	particles.poses[i].position.y=rnd.uniformReal(min_y, max_y);
    	float theta=rnd.uniformReal(min_a, max_a);
    	particles.poses[i].orientation.w=cos(theta/2);
    	particles.poses[i].orientation.z=sin(theta/2);
    }
    return particles;
}

std::vector<sensor_msgs::LaserScan> simulate_particle_scans(geometry_msgs::PoseArray& particles, nav_msgs::OccupancyGrid& map)
{
    std::vector<sensor_msgs::LaserScan> simulated_scans;
    simulated_scans.resize(particles.poses.size());
    return simulated_scans;
}

std::vector<float> calculate_particle_weights(std::vector<sensor_msgs::LaserScan>& simulated_scans, sensor_msgs::LaserScan& real_scan)
{
    std::vector<float> weights;
    weights.resize(simulated_scans.size());
    return weights;
}

int random_choice(std::vector<float>& weights)
{
    random_numbers::RandomNumberGenerator rnd;
    return -1;
}

geometry_msgs::PoseArray resample_particles(geometry_msgs::PoseArray& particles, std::vector<float>& weights)
{
    random_numbers::RandomNumberGenerator rnd;
    geometry_msgs::PoseArray resampled_particles;
    resampled_particles.header.frame_id = "map";
    resampled_particles.poses.resize(particles.poses.size());
    return resampled_particles;
}

void move_particles(geometry_msgs::PoseArray& particles, float delta_x, float delta_y, float delta_t)
{
    random_numbers::RandomNumberGenerator rnd;
    /*
     * TODO:
     *
     * Move each particle a displacement given by delta_x, delta_y and delta_t.
     * Displacement is given w.r.t. particles's frame, i.e., to calculate the new position for
     * each particle you need to rotate delta_x and delta_y, on Z axis, an angle theta_i, where theta_i
     * is the orientation of the i-th particle.
     * Add gaussian noise to each new position. Use MOVEMENT_NOISE as covariances. 
     */
    for(size_t i=0; i<particles.poses.size(); i++){
    	float a= atan2(particles.poses[i].orientation.z, particles.poses[i].orientation.w)*2;
    	particles.poses[i].position.x+=delta_x*cos(a)- delta_y*sin(a)+rnd.gaussian(0, MOVEMENT_NOISE);
    	particles.poses[i].position.y+=delta_x*sin(a)+ delta_y*cos(a)+rnd.gaussian(0, MOVEMENT_NOISE);
    	a+=delta_t+rnd.gaussian(0, MOVEMENT_NOISE);
    	particles.poses[i].orientation.w=cos(a/2);
    	particles.poses[i].orientation.z=sin(a/2);
    }
}

bool check_displacement(geometry_msgs::Pose2D& robot_pose, geometry_msgs::Pose2D& delta_pose)
{
    static geometry_msgs::Pose2D last_pose;
    float delta_x = robot_pose.x - last_pose.x;
    float delta_y = robot_pose.y - last_pose.y;
    float delta_a = robot_pose.theta - last_pose.theta;
    if(delta_a >  M_PI) delta_a -= 2*M_PI;
    if(delta_a < -M_PI) delta_a += 2*M_PI;
    if(sqrt(delta_x*delta_x + delta_y*delta_y) > DISTANCE_THRESHOLD || fabs(delta_a) > ANGLE_THRESHOLD)
    {
        last_pose = robot_pose;
        delta_pose.x =  delta_x*cos(robot_pose.theta) + delta_y*sin(robot_pose.theta);
        delta_pose.y = -delta_x*sin(robot_pose.theta) + delta_y*cos(robot_pose.theta);
        delta_pose.theta = delta_a;
        return true;
    }
    return false;
}

geometry_msgs::Pose2D get_robot_odometry(tf::TransformListener& listener)
{
    tf::StampedTransform t;
    geometry_msgs::Pose2D pose;
    try{
        listener.lookupTransform("odom", "base_link", ros::Time(0), t);
        pose.x = t.getOrigin().x();
        pose.y = t.getOrigin().y();
        pose.theta = atan2(t.getRotation().z(), t.getRotation().w())*2;
    }
    catch(std::exception &e){
        pose.x = 0;
        pose.y = 0;
        pose.theta = 0;
    }
    return pose;
}

geometry_msgs::Pose2D get_robot_pose_estimation(geometry_msgs::PoseArray& particles)
{
    geometry_msgs::Pose2D p;
    float z = 0;
    float w = 0;
    for(size_t i=0; i < particles.poses.size(); i++)
    {
        p.x += particles.poses[i].position.x;
        p.y += particles.poses[i].position.y;
        z   += particles.poses[i].orientation.z;
        w   += particles.poses[i].orientation.w;
        
    }
    p.x /= particles.poses.size();
    p.y /= particles.poses.size();
    z   /= particles.poses.size();
    w   /= particles.poses.size();
    p.theta = atan2(z, w)*2;
    return p;
}

void callback_laser_scan(const sensor_msgs::LaserScan::ConstPtr& msg){real_scan = *msg;}

tf::Transform get_map_to_odom_transform(geometry_msgs::Pose2D odom, geometry_msgs::Pose2D loc)
{
    tf::Transform odom_to_base(tf::Quaternion(0,0,sin(odom.theta/2),cos(odom.theta/2)), tf::Vector3(odom.x,odom.y,0));
    tf::Transform map_to_base(tf::Quaternion(0,0,sin(loc.theta/2),cos(loc.theta/2)), tf::Vector3(loc.x, loc.y, 0));
    return map_to_base*odom_to_base.inverse();
}

int main(int argc, char** argv)
{
    std::cout << "PRACTICE 06 - " << NOMBRE << std::endl;
    ros::init(argc, argv, "practice06");
    ros::NodeHandle n("~");
    ros::Rate loop(20);
    ros::Subscriber sub_scan      = n.subscribe("/scan", 1, callback_laser_scan);
    ros::Publisher  pub_particles = n.advertise<geometry_msgs::PoseArray>("/particle_cloud", 1); 
    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;
    nav_msgs::GetMap srv_get_map;

    float init_min_x = -1;
    float init_min_y = -1;
    float init_min_a = -1;
    float init_max_x = 2;
    float init_max_y = 2;
    float init_max_a = 1;
    float number_of_particles = 200;
    if(ros::param::has("~n"))
        ros::param::get("~n", number_of_particles);
    if(ros::param::has("~max_x"))
        ros::param::get("~max_x", init_max_x);
    if(ros::param::has("~min_x"))
        ros::param::get("~min_x", init_min_x);
    if(ros::param::has("~min_y"))
        ros::param::get("~min_y", init_min_y);
    if(ros::param::has("~min_a"))
        ros::param::get("~min_a", init_min_a);
    if(ros::param::has("~max_x"))
        ros::param::get("~max_x", init_max_x);
    if(ros::param::has("~max_y"))
        ros::param::get("~max_y", init_max_y);
    if(ros::param::has("~max_a"))
        ros::param::get("~max_a", init_max_a);

    /*
     * IMPORTANT VARIABLES FOR THE LOCALIZATION PROCESS
     */
    geometry_msgs::PoseArray particles;                   //A set of N particles
    nav_msgs::OccupancyGrid static_map;                   //A static map
    std::vector<sensor_msgs::LaserScan> simulated_scans;  //A set of simulated laser readings, one scan per particle
    std::vector<float> particle_weights;                  //A set of weights for each particle
    geometry_msgs::Pose2D robot_odom;                     //Position estimated by the odometry
    geometry_msgs::Pose2D delta_pose;                     //Displacement since last pose estimation
    geometry_msgs::Pose2D robot_pose;                     //Estimated robot position with respect to map
    tf::Transform map_to_odom_transform;                  //Transformation from map to odom frame (which corrects odometry estimation)

    /*
     * Sentences for getting the static map, info about real lidar sensor,
     * and initialization of corresponding arrays.
     */
    ros::service::waitForService("/static_map", ros::Duration(20));
    ros::service::call("/static_map", srv_get_map);
    static_map = srv_get_map.response.map;
    real_scan = *ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan");
    real_sensor_info = real_scan;
    real_sensor_info.angle_increment *= LASER_DOWNSAMPLING;
    std::cout << "Real Scan Info: Number of readings: " << real_scan.ranges.size() << std::endl;
    std::cout << "Min angle: " << real_scan.angle_min << std::endl;
    std::cout << "Angle increment: " << real_scan.angle_increment << std::endl;
    std::cout << "Scan downsampling: " << LASER_DOWNSAMPLING << std::endl;

    particles = get_initial_distribution(number_of_particles, init_min_x, init_max_x, init_min_y, init_max_y, init_min_a, init_max_a);
    robot_pose = get_robot_pose_estimation(particles);
    robot_odom = get_robot_odometry(listener);
    map_to_odom_transform = get_map_to_odom_transform(robot_odom, robot_pose);
    check_displacement(robot_odom, delta_pose);
    pub_particles.publish(particles);
    while(ros::ok())
    {
        robot_odom = get_robot_odometry(listener);
        if(check_displacement(robot_odom, delta_pose))
        {
            std::cout << "Displacement detected. Updating pose estimation..." << std::endl;
            /*
             * TODO:
             *
             * Move all particles a displacement given by delta_pose (Pose2D message) by calling the move_particles function.
             */
            move_particles(particles, delta_pose.x, delta_pose.y, delta_pose.theta);
            pub_particles.publish(particles);
            map_to_odom_transform = get_map_to_odom_transform(robot_odom, get_robot_pose_estimation(particles));
        }
        broadcaster.sendTransform(tf::StampedTransform(map_to_odom_transform, ros::Time::now(), "map", "odom"));
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
