#define NUM_TRIALS 100
#define TRAIL_DURATION_MINS 1 // in minutes
#define TRIAL_RESET_DELAY_US 10000 // in us
#include "Map.h"

#include <iostream>
#include <fstream>
#include <unistd.h>

#include <stage.hh>
#include "ros/ros.h"
#include "ros/package.h"

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <follower_rl/stage_message.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

using namespace Stg;

static const double cruisespeed = 0.4; 
static const double avoidspeed = 0.05; 
static const double avoidturn = 0.5;
static const double minfrontdistance = 1.0; // 0.6  
static const bool verbose = false;
static const double stopdist = 0.3;
static const int avoidduration = 10;

struct ModelRobot
{
  ModelPosition* pos;
  ModelRanger* laser;
  Pose resetPose;
  int avoidcount, randcount;
};

ModelRobot* robot;
usec_t stgSpeedTime;

ros::NodeHandle* n;
ros::Publisher pub_state_;
ros::Publisher pub_cmd_vel_;
image_transport::Publisher image_pub_;

ros::Subscriber sub_vel_;
ros::ServiceServer reset_srv_;


geometry_msgs::PoseStamped rosCurPose;
sensor_msgs::LaserScan rosLaserData;
bool collision = false;
bool allowNewMsg = true;
double minFrontDist;
ros::Time lastSentTime;

Map map;

float start_positions[NUM_TRIALS][3];
int trial_idx = 0;
usec_t trial_start_time;

bool lock=false;

void loadStartPositions(void)
{
  std::string file_path = ros::package::getPath("follower_rl") + "/fixed_positions.txt";
  std::ifstream position_file(file_path.c_str());

  float x, y, a;
  for (int i = 0; i < NUM_TRIALS; i++) {
    position_file >> x >> y >> a;
    start_positions[i][0] = x;
    start_positions[i][1] = y;
    start_positions[i][2] = a;  
  }
}

void nextTrial(void)
{
  trial_idx++;
  
  if (trial_idx >= NUM_TRIALS) {
    ROS_INFO("All trials have ended");
  } else {
    // lock every thing
    lock = true;

    robot->pos->SetPose(Pose(
      start_positions[trial_idx][0], start_positions[trial_idx][1], 0.0, start_positions[trial_idx][2]
    ));
    map.reset();

    // wait for things to settle down    
    usleep(TRIAL_RESET_DELAY_US);

    trial_start_time = robot->pos->GetWorld()->SimTimeNow();

    lock = false;

  }

}

void spinTrial(void)
{
  if (
    (robot->pos->GetWorld()->SimTimeNow() - trial_start_time)/(1e6*60) >= TRAIL_DURATION_MINS
  ) {
    ROS_INFO("end of trial %d", trial_idx);
    nextTrial();
  }

}

int stgPoseUpdateCB( Model* mod, ModelRobot* robot)
{
  if (lock) {
    return 0;
  }

  spinTrial();

  geometry_msgs::PoseStamped positionMsg;
  positionMsg.pose.position.x = robot->pos->GetPose().x;
  positionMsg.pose.position.y = robot->pos->GetPose().y;
  positionMsg.pose.position.z = robot->pos->GetPose().z;
  positionMsg.pose.orientation = tf::createQuaternionMsgFromYaw( robot->pos->GetPose().a);
  positionMsg.header.stamp = ros::Time::now();
  rosCurPose = positionMsg;

  // update robot position in map
  map.updateRobotPose(robot->pos->GetPose());

  return 0;
}

int stgLaserCB( Model* mod, ModelRobot* robot)
{
  if (lock) {
    return 0;
  }

  map.updateMap(robot->pos->GetPose(), robot->laser->GetSensors()[0]);
  // get the data
  const std::vector<meters_t>& scan = robot->laser->GetSensors()[0].ranges;
  uint32_t sample_count = scan.size();
  if( sample_count < 1 )
    return 0;

  bool obstruction = false;
  bool stop = false;

  // find the closest distance to the left and right and check if
  // there's anything in front
  double minleft = 1e6;
  double minright = 1e6;

  for (uint32_t i = 0; i < sample_count; i++)
    {

      // if( verbose ) printf( "%.3f ", scan[i] );

      if( (i > (sample_count/3))
    && (i < (sample_count - (sample_count/3)))
    && scan[i] < minfrontdistance)
  {
    if( verbose ) puts( "  obstruction!" );
    obstruction = true;
  }

      if( scan[i] < stopdist )
  {
    if( verbose ) puts( "  stopping!" );
    stop = true;
  }

      if( i > sample_count/2 )
  minleft = std::min( minleft, scan[i] );
      else
  minright = std::min( minright, scan[i] );
    }

  if( verbose )
    {
      puts( "" );
      printf( "minleft %.3f \n", minleft );
      printf( "minright %.3f\n ", minright );
    }

  if( obstruction || stop || (robot->avoidcount>0) )
    {
      if( verbose ) printf( "Avoid %d\n", robot->avoidcount );

      robot->pos->SetXSpeed( stop ? 0.0 : avoidspeed );

      /* once we start avoiding, select a turn direction and stick
   with it for a few iterations */
      if( robot->avoidcount < 1 )
        {
    if( verbose ) puts( "Avoid START" );
          robot->avoidcount = random() % avoidduration + avoidduration;

    if( minleft < minright  )
      {
        robot->pos->SetTurnSpeed( -avoidturn );

        if( verbose ) printf( "turning right %.2f\n", -avoidturn );
      }
    else
      {
        robot->pos->SetTurnSpeed( +avoidturn );

    if( verbose ) printf( "turning left %2f\n", +avoidturn );
      }
        }

      robot->avoidcount--;
    }
  else
    {
      if( verbose ) puts( "Cruise" );

      robot->avoidcount = 0;
      robot->pos->SetXSpeed( cruisespeed );
      robot->pos->SetTurnSpeed(  0 );

    }

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
      for(unsigned int i = 0; i < sensor.ranges.size(); i++)
        {
          laserMsgs.ranges[i] = sensor.ranges[i];
          if(sensor.ranges[i] < 0.45)
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
        if (collision)
        {
          ROS_WARN("You collided");
        }

        allowNewMsg = false;
        follower_rl::stage_message stage_msg;
        stage_msg.header.stamp = ros::Time::now();
        stage_msg.collision = collision;
        stage_msg.minFrontDist = minFrontDist;
        stage_msg.position = rosCurPose;
        stage_msg.laser = rosLaserData;
        
        pub_state_.publish(stage_msg);

        // publish the command velocity
        geometry_msgs::TwistStamped twist_msg;
        twist_msg.header.stamp = stage_msg.header.stamp;
        twist_msg.twist.linear.x = robot->pos->GetVelocity().x;
        twist_msg.twist.angular.z = robot->pos->GetVelocity().a;
        pub_cmd_vel_.publish(twist_msg);

        // publish the map
        
        // flip the image so that that it makes physical sense
        cv::Mat flipped_map;               
        cv::flip(map.getMap(), flipped_map, 0);

        // publish the flipped image
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", flipped_map).toImageMsg();
        img_msg->header.stamp = stage_msg.header.stamp;
        image_pub_.publish(img_msg);
      
    }

    return 0;
}

void rosVelocityCB( const geometry_msgs::TwistConstPtr vel)
{
// ROS_WARN("Vel recieved");
  robot->pos->SetXSpeed( vel->linear.x);
  robot->pos->SetTurnSpeed( vel->angular.z);
  lastSentTime = ros::Time::now();
  stgSpeedTime = robot->pos->GetWorld()->SimTimeNow();
  allowNewMsg = true;
}

bool rosResetSrvCB(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Resetting stage!");
  robot->pos->SetPose( robot->resetPose);
  return true;
}

extern "C" int Init( Model* mod )
{ 
  int argc = 0;
  char** argv;
  ros::init( argc, argv, "target_controller_node");
  n = new ros::NodeHandle();
  lastSentTime = ros::Time::now();
  pub_state_ = n->advertise<follower_rl::stage_message>("input_data", 15);
  pub_cmd_vel_ = n->advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped", 15);

  // map publisher stuffs
  image_transport::ImageTransport it(*n);
  image_pub_ = it.advertise("/map", 1);

  sub_vel_ = n->subscribe( "cmd_vel", 15, &rosVelocityCB);
  reset_srv_ = n->advertiseService("reset_positions", &rosResetSrvCB);
  robot = new ModelRobot;
  robot->pos = (ModelPosition*) mod;
  robot->pos->AddCallback( Model::CB_UPDATE, (model_callback_t)stgPoseUpdateCB, robot);
  robot->pos->Subscribe();
  robot->resetPose = robot->pos->GetPose();
  //    robot->pos->GetChild("ranger:0")->Subscribe();
  robot->laser = (ModelRanger*)mod->GetChild("ranger:0");
  robot->laser->AddCallback( Model::CB_UPDATE, (model_callback_t)stgLaserCB, robot);
  robot->laser->Subscribe();

  loadStartPositions();
  robot->pos->SetPose( Pose(start_positions[0][0], start_positions[0][1], 0.0, start_positions[0][2]) );
  
  Model *floorplan = robot->pos->GetWorld()->GetModel("blank");
  map.initMap(
    floorplan->GetGeom().size.x,
    floorplan->GetGeom().size.y,
    robot->pos
  );

  trial_start_time = robot->pos->GetWorld()->SimTimeNow();
  
  return 0; //ok
}

