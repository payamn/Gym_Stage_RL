
#include <boost/thread/thread.hpp>
#include <X11/Xlib.h>

#include <stage.hh>
#include "ros/ros.h"
#include <ros/package.h>


#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <follower_rl/stage_message.h>
#include <follower_rl/reset_position.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <string>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

using namespace Stg;
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
#define COLLISION_RADIUS 0.55

static const double cruisespeed = 0.4;
static const double avoidspeed = 0.05;
static const double avoidturn = 0.5;
static const double minfrontdistance = 1.0; // 0.6
static const bool verbose = true;
static const double stopdist = 0.3;
static const int avoidduration = 5;


#include<sstream>
template <typename T>
std::string to_string(T value)
{
  //create an output string stream
  std::ostringstream os ;

  //throw the value into the string stream
  os << value ;

  //convert the string stream into a string and return
  return os.str() ;
}

class StepWorldGui : public WorldGui
{
public:
  using WorldGui::WorldGui;
  void setPauseStatus(bool _is_paused)
  {
	paused = _is_paused;
	is_stepping_ = false;
  }

  void step()
  {
	// for (int i=0 ; i <4; i++)
	// {


	if (is_stepping_)
	{
	  return;
	}

	is_stepping_ = true;
	Stop();
	Fl::lock();
	ros::Time t1 = ros::Time::now();
	World::Update();
	ros::Time t2 = ros::Time::now();
	ROS_INFO("Freq: %f", 1.0f/(t2.toSec() - t1.toSec()));
	Fl::unlock();
	Fl::awake();
	is_stepping_ = false;
  // }
  }

protected:
  bool is_stepping_;
};

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
  ros::Publisher pub_state_;
  ros::Publisher pub_cmd_vel_;

  ros::Subscriber sub_vel_;
  ros::ServiceServer reset_srv_;
  ros::ServiceServer reset_random_srv_;

  int avoidcount, randcount;
};

ModelRobot* robots[20];
ModelRobot* robots_w[20];

usec_t stgSpeedTime;
uint64_t stgUpdateCyle;

ros::NodeHandle* n;

geometry_msgs::PoseStamped rosCurPose;
sensor_msgs::LaserScan rosLaserData;
bool collision = false;
bool allowNewMsg = true;
double minFrontDist;
ros::Time lastSentTime;


StepWorldGui *world;

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

  if (robot->pos->GetWorld()->UpdateCount() - stgUpdateCyle > 1) // 100000)
	  robot->pos->SetSpeed( 0, 0, 0);

  ros::spinOnce();
  return 0;
}

int stgLaserCB( Model* mod, ModelRobot* robot)
{

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
		  ROS_WARN("You collided");
		}


		ros::Time now = ros::Time::now();


		allowNewMsg = false;
		follower_rl::stage_message stage_msg;
		stage_msg.header.stamp = now;
		stage_msg.collision = collision;
		stage_msg.minFrontDist = minFrontDist;
		stage_msg.position = rosCurPose;
		stage_msg.laser = rosLaserData;


		// publish the command velocity
		geometry_msgs::TwistStamped twist_msg;
		twist_msg.header.stamp = now;
		twist_msg.twist.linear.x = robot->pos->GetVelocity().x;
		twist_msg.twist.angular.z = robot->pos->GetVelocity().a;
		robot->pub_cmd_vel_.publish(twist_msg);

		robot->pub_state_.publish(stage_msg);
	}

	return 0;
}

void rosVelocityCB( const geometry_msgs::TwistConstPtr& vel, ModelRobot* robot)
{
  // ROS_WARN("Vel recieved");
  robot->pos->SetXSpeed( vel->linear.x);
  robot->pos->SetTurnSpeed( vel->angular.z);
  lastSentTime = ros::Time::now();
  stgSpeedTime = robot->pos->GetWorld()->SimTimeNow();
  stgUpdateCyle = robot->pos->GetWorld()->UpdateCount();
  allowNewMsg = true;

  ROS_INFO("Stepping the world");
  world->step();
}

bool rosResetSrvCB(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response, ModelRobot* robot)
{
  ROS_INFO("Resetting stage!");
  robot->pos->SetPose( robot->resetPose );
  return true;
}

bool rosResetRandomSrvCB(follower_rl::reset_position::Request& request, follower_rl::reset_position::Response& response, ModelRobot* robot)
{
  ROS_INFO("Resetting stage!");

  Pose pose;
  pose.x = request.position.position.x;
  pose.y = request.position.position.y;
  pose.z = 0;
  pose.a = request.position.orientation.w;
  robot->pos->SetPose(pose);
  return true;
}

void spinThread()
{
  while (true)
  {
	ros::spinOnce();
  }
}


int LaserCBWanderer( Model* mod, ModelRobot* robot)
{

  ROS_INFO("in laser cv wanderer");

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

	return 0;
	}



void init_robot( ModelRobot* robot, std::string r_name, std::string r_number)
{
	Model* mod = world->GetModel(r_name + "_" + r_number);
	while (!mod)
	{
		mod = world->GetModel(r_name + "_" + r_number);
	}
	robot->avoidcount = 4;
	robot->pos = (ModelOurPosition*) mod;
	robot->pos->AddCallback( Model::CB_UPDATE, (model_callback_t)stgPoseUpdateCB, robot);
	robot->pos->Subscribe();

	if (!r_name.compare("W"))
	{
		robot->laser = (ModelRanger*)mod->GetChild("ranger:0");
		robot->laser->AddCallback( Model::CB_UPDATE, (model_callback_t)LaserCBWanderer, robot);
		robot->laser->Subscribe();
		Model *floorplan = robot->pos->GetWorld()->GetModel("blank");
		ROS_INFO("adding cpu node %s_%s", r_name.c_str(), r_number.c_str());
		return ;
	}

	robot->pub_state_ = n->advertise<follower_rl::stage_message>("input_data", 15);
	robot->pub_cmd_vel_ = n->advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped", 15);
	robot->sub_vel_ = n->subscribe<geometry_msgs::Twist>("cmd_vel", 15, boost::bind(&rosVelocityCB, _1, robot));
	robot->reset_srv_ = n->advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("reset_positions", boost::bind(rosResetSrvCB, _1, _2, robot));
	robot->reset_random_srv_ = n->advertiseService<follower_rl::reset_position::Request, follower_rl::reset_position::Response>("reset_random_positions", boost::bind(&rosResetRandomSrvCB, _1, _2, robot) );

	robot->resetPose = robot->pos->GetPose();
	//    robot->pos->GetChild("ranger:0")->Subscribe();
	robot->laser = (ModelRanger*)mod->GetChild("ranger:0");
	robot->laser->AddCallback( Model::CB_UPDATE, (model_callback_t)stgLaserCB, robot);
	robot->laser->Subscribe();

	Model *floorplan = robot->pos->GetWorld()->GetModel("blank");
	ROS_INFO("adding node %s_%s", r_name.c_str(), r_number.c_str());
}

int main(int argc, char **argv)
{
  XInitThreads();

  if (argc < 2)
  {
	std::cerr << "world file not provided" << std::endl;
	return 1;
  }
    namedWindow( "show", WINDOW_AUTOSIZE );

    std::string path = ros::package::getPath("follower_rl");
    Mat image;
    image = imread(path+"/world/map.png", cv::IMREAD_GRAYSCALE);
    ROS_INFO("map width %d height %d", image.rows, image.cols);
    cv::bitwise_not ( image, image );
    while (true){
     imshow( "show", image  );
          cv::waitKey(0);

    }
     namedWindow( "show", WINDOW_AUTOSIZE );
     imshow( "show", image  );
//    cv::subtract(cv::Scalar:all(255),image,image);
    std::cout <<  "M = "<< std::endl << " "  << image << std::endl << std::endl;
     exit(0);
  // initialize libstage
  Stg::Init( &argc, &argv );
  world = new StepWorldGui(800, 700, "follow rl");
  world->Load( argv[1] );
  world->setPauseStatus(true);

  ros::init( argc, argv, "target_controller_node");
  n = new ros::NodeHandle();
  lastSentTime = ros::Time::now();

// multi robot in one map
//  for (int i=0 ; i<16 ; i++)
//  {
//      robots[i] = new ModelRobot;
//      ModelRobot* robot = robots[i];
//      init_robot(robot, "RL", to_string(i));
//  }
//  for (int j=0 ; j<3 ; j++)
//  {
//      robots_w[j] = new ModelRobot;
//      ModelRobot* robot = robots_w[j];
//      init_robot(robot, "W", to_string(j));
//  }
//
	robots[0] = new ModelRobot;
	init_robot(robots[0], "RL", "0");

	robots_w[0] = new ModelRobot;

	init_robot(robots_w[0], "W", "0");

  /*boost::thread spin_thread([](void) -> void {
	while (true)
	{
	  ros::spinOnce();
	}
  });*/
  boost::thread spin_thread(&spinThread);
  while (Fl::first_window())
  {
	Fl::wait();

  }

  return 0; //ok
}
