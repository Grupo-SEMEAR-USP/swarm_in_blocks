#include <ros.h>
#include <clover/Navigate.h>
#include <clover/GetTelemetry.h>
#include <swarm_checker/SwarmState.h>

class Swarm{};

void main(int argc, char **argv) {

    ros::init(argc, argv, "/collision_avoidance");
    ros::NodeHandle nh("~");
    ros::NodeHandle n;

    // ros::Service 
}