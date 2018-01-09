#include "Map.h"

#include <stage.hh>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <deep_rl_map_coverage/stage_message.h>
#include <deep_rl_map_coverage/reset_position.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

using namespace Stg;

#define COLLISION_RADIUS 0.55

static const double cruisespeed = 0.4;
static const double avoidspeed = 0.05;
static const double avoidturn = 0.5;
static const double minfrontdistance = 1.0; // 0.6
static const bool verbose = true;
static const double stopdist = 0.3;
static const int avoidduration = 10;

class ModelOurPosition: public ModelPosition
{
public:
  bool TestCollision() 
  {
    return ModelPosition::TestCollision();
  }
};

struct ModelRobot
{
  ModelOurPosition* pos;
  ModelRanger* laser;
  Pose resetPose;
  int avoidcount, randcount;
};

ModelRobot* robot;
usec_t stgSpeedTime;
uint64_t stgUpdateCyle;

ros::NodeHandle* n;
ros::Publisher pub_state_;
ros::Publisher pub_cmd_vel_;
image_transport::Publisher image_pub_;
image_transport::Publisher full_map_pub_;

ros::Subscriber sub_vel_;
ros::ServiceServer reset_srv_;
ros::ServiceServer reset_random_srv_;

geometry_msgs::PoseStamped rosCurPose;
sensor_msgs::LaserScan rosLaserData;
bool collision = false;
bool allowNewMsg = true;
double minFrontDist;
ros::Time lastSentTime;

Map map;

int stgPoseUpdateCB( Model* mod, ModelRobot* robot)
{
  geometry_msgs::PoseStamped positionMsg;
  positionMsg.pose.position.x = robot->pos->GetPose().x;
  positionMsg.pose.position.y = robot->pos->GetPose().y;
  positionMsg.pose.position.z = robot->pos->GetPose().z;
  positionMsg.pose.orientation = tf::createQuaternionMsgFromYaw( robot->pos->GetPose().a);
  positionMsg.header.stamp = ros::Time::now();
  rosCurPose = positionMsg;

  // update robot position in map
  map.updateRobotPose(robot->pos->GetPose());

  // if (robot->pos->GetWorld()->UpdateCount() - stgUpdateCyle > 1) // 100000)
  //     robot->pos->SetSpeed( 0, 0, 0);

  ros::spinOnce();
  return 0;
}

int stgLaserCB( Model* mod, ModelRobot* robot)
{
  map.updateMap(robot->pos->GetPose(), robot->laser->GetSensors()[0]);

  sensor_msgs::LaserScan laserMsgs;
  const Stg::ModelRanger::Sensor& sensor = robot->laser->GetSensors()[0];
  double minDist = sensor.range.max;
  if( sensor.ranges.size() )
    {
      // Translate into ROS message format and publish
      laserMsgs.angle_min = -sensor.fov/2.0;
      laserMsgs.angle_max = +sensor.fov/2.0;
      laserMsgs.angle_increment = sensor.fov/(double)(sensor.sample_count-1);
      laserMsgs.range_min = sensor.range.min;
      laserMsgs.range_max = sensor.range.max;
      laserMsgs.ranges.resize(sensor.ranges.size());
      laserMsgs.intensities.resize(sensor.intensities.size());

      collision = false;
      minFrontDist = sensor.range.max;
      // added by sepehr for random position init:
      //        double min_laser_val = 99;
      collision = false;
      for(unsigned int i = 0; i < sensor.ranges.size(); i++)
        {
          laserMsgs.ranges[i] = sensor.ranges[i];
          if(sensor.ranges[i] < COLLISION_RADIUS)
            collision = true;
          if( i > (sensor.fov*180.0/M_PI - 45)/2 && i < (sensor.fov*180.0/M_PI + 45)/2 && sensor.ranges[i]  < minFrontDist)
            minFrontDist = sensor.ranges[i];
          if( sensor.ranges[i] < minDist)
            minDist = sensor.ranges[i];
          //            if(sensor.ranges[i] < min_laser_val)
          //                min_laser_val = sensor.ranges[i];
          laserMsgs.intensities[i] = sensor.intensities[i];
        }

      //        if( min_laser_val > 3.3 && rand()/float(RAND_MAX) > 0.1)
      //        {
      //            initial_poses.clear();
      //            initial_poses.push_back(robotmodel->positionmodel->GetGlobalPose());
      //        }
      laserMsgs.header.stamp = ros::Time::now();
      rosLaserData = laserMsgs;
    }


  //temp, just to check publish rate:
  // if( allowNewMsg
  //     && laserMsgs.header.stamp > lastSentTime
  //     && rosCurPose.header.stamp > lastSentTime)

//      if (robot->pos->GetVelocity().a > 0.01 ||
//          robot->pos->GetVelocity().a <-0.01)
      {
        collision = collision || robot->pos->TestCollision();
        if (collision)
        {
          // ROS_WARN("You collided");
        }

        // publish the map

        ros::Time now = ros::Time::now();
        // flip the image so that that it makes physical sense
        cv::Mat flipped_map;
        cv::flip(map.getMap(), flipped_map, 0);

        // publish the flipped image
        sensor_msgs::ImagePtr map_image = cv_bridge::CvImage(std_msgs::Header(), "rgb8", flipped_map).toImageMsg();
        map_image->header.stamp = now;
        image_pub_.publish(map_image);

        allowNewMsg = false;
        deep_rl_map_coverage::stage_message stage_msg;
        stage_msg.header.stamp = now;
        stage_msg.collision = collision;
        stage_msg.minFrontDist = minFrontDist;
        stage_msg.position = rosCurPose;
        stage_msg.laser = rosLaserData;
        stage_msg.map_image = *map_image;

        // full map
        cv::flip(map.getCompleteMap(), flipped_map, 0);
        map_image = cv_bridge::CvImage(std_msgs::Header(), "rgb8", flipped_map).toImageMsg();
        full_map_pub_.publish(map_image);
        

        // find number of explored pixels
        cv::Mat thresholded_map;
        cv::Mat separate_channels[3];
        cv::split(map.getMapWithoutRobot(), separate_channels);
        cv::threshold(separate_channels[EXPLORED_AREA_CHANNEL], thresholded_map, EXPLORED_AREA_COLOR-1, 255, cv::THRESH_BINARY);
        stage_msg.explored_pixels = cv::countNonZero(thresholded_map);


        pub_state_.publish(stage_msg);

        // publish the command velocity
        geometry_msgs::TwistStamped twist_msg;
        twist_msg.header.stamp = now;
        twist_msg.twist.linear.x = robot->pos->GetVelocity().x;
        twist_msg.twist.angular.z = robot->pos->GetVelocity().a;
        pub_cmd_vel_.publish(twist_msg);

        // print output
        std::cout << robot->pos->GetWorld()->SimTimeNow() / 1e6 << ", " << stage_msg.explored_pixels << std::endl;
    }

    return 0;
}

void rosVelocityCB( const geometry_msgs::TwistConstPtr& vel)
{
  // ROS_WARN("Vel recieved");
  robot->pos->SetXSpeed( vel->linear.x);
  robot->pos->SetTurnSpeed( vel->angular.z);
  lastSentTime = ros::Time::now();
  stgSpeedTime = robot->pos->GetWorld()->SimTimeNow();
  stgUpdateCyle = robot->pos->GetWorld()->UpdateCount();
  allowNewMsg = true;
}

bool rosResetSrvCB(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  // ROS_INFO("Resetting stage!");
  robot->pos->SetPose( robot->resetPose );
  map.reset();
  return true;
}

bool rosResetRandomSrvCB(deep_rl_map_coverage::reset_position::Request& request, deep_rl_map_coverage::reset_position::Response& response)
{
  // ROS_INFO("Resetting stage!");
  robot->pos->SetPose( robot->resetPose );
  map.reset();
  return true;

  Pose pose;
  pose.x = request.position.position.x;
  pose.y = request.position.position.y;
  pose.z = 0;
  pose.a = request.position.orientation.w;
  robot->pos->SetPose(pose);
  map.reset();
  return true;
}


extern "C" int Init( Model* mod )
{
  int argc = 0;
  char** argv;
  ros::init( argc, argv, "target_controller_node");
  n = new ros::NodeHandle();
  lastSentTime = ros::Time::now();
  pub_state_ = n->advertise<deep_rl_map_coverage::stage_message>("input_data", 15);
  pub_cmd_vel_ = n->advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped", 15);

  // map publisher stuffs
  image_transport::ImageTransport it(*n);
  image_pub_ = it.advertise("/map", 1);
  full_map_pub_ = it.advertise("/full_map", 1);

  sub_vel_ = n->subscribe( "cmd_vel", 15, &rosVelocityCB);
  reset_srv_ = n->advertiseService("reset_positions", &rosResetSrvCB);
  reset_random_srv_ = n->advertiseService("reset_random_positions", &rosResetRandomSrvCB);
  robot = new ModelRobot;
  robot->pos = (ModelOurPosition*) mod;
  robot->pos->AddCallback( Model::CB_UPDATE, (model_callback_t)stgPoseUpdateCB, robot);
  robot->pos->Subscribe();
  robot->resetPose = robot->pos->GetPose();
  //    robot->pos->GetChild("ranger:0")->Subscribe();
  robot->laser = (ModelRanger*)mod->GetChild("ranger:0");
  robot->laser->AddCallback( Model::CB_UPDATE, (model_callback_t)stgLaserCB, robot);
  robot->laser->Subscribe();

  Model *floorplan = robot->pos->GetWorld()->GetModel("blank");
  map.initMap(
    floorplan->GetGeom().size.x,
    floorplan->GetGeom().size.y,
    robot->pos
  );

  return 0; //ok
}
