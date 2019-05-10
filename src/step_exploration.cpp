
#include <boost/thread/thread.hpp>
#include <X11/Xlib.h>
#include <iostream>

#include <stage.hh>
#include "ros/ros.h"
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
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

static const double cruisespeed = 0.1;
static const double avoidspeed = 0.02;
static const double avoidturn = 0.5;
static const double minfrontdistance = 2.0; // 0.6
static const bool verbose = false;
static const double stopdist = 0.1;
static const int avoidduration = 5;
static const Point World_Size = Point(40,40);

#include<sstream>

#include <exception>



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
class StepWorldGui : public World
//class StepWorldGui : public WorldGui
{
public:
  using World::World;
//  using WorldGui::WorldGui;
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
//	ROS_INFO("Freq: %f", 1.0f/(t2.toSec() - t1.toSec()));
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
	std::string name;
  std::vector< ModelBlobfinder::Blob > blobs;
  ros::Time last_blob_update;
  ModelOurPosition* pos;
  ModelRanger* laser;
  ModelBlobfinder* blob_finder;
  Pose resetPose;
  ros::Publisher pub_state_;
  ros::Publisher pub_cmd_vel_;
  image_transport::Publisher map_image_pub_;
  Mat map;   
  Mat visual_pos;
  ros::Subscriber sub_vel_;
  ros::ServiceServer reset_srv_;
  ros::ServiceServer reset_random_srv_;

  int avoidcount, randcount;
};
float set_other_robot_pose( ModelRobot* robot);
void show_robot(ModelRobot* robot);

ModelRobot* robots[20];
ModelRobot* robots_w[20];

usec_t stgSpeedTime;
uint64_t stgUpdateCyle;

ros::NodeHandle* n;

geometry_msgs::PoseStamped rl_agent_pose;
geometry_msgs::PoseStamped human_agent_pose;

sensor_msgs::LaserScan rosLaserData;
bool collision = false;
bool allowNewMsg = true;
double minFrontDist;
ros::Time lastSentTime;


StepWorldGui *world;

bool check_obstacle (Point2f start, Point2f end ,ModelRobot* robot)
{
	cv::LineIterator line_iterator(robot->visual_pos, start, end);
	cv::LineIterator it = line_iterator;
	bool is_obstacle = false;
	for(int j = 0; j < line_iterator.count; j++, ++it)
	{
		// circle(robot->visual_pos, it.pos(), 5, (255, 255, 0), 1, 8);
		if (robot->visual_pos.at<uint8_t>(it.pos()) == 0)
		{
			// there is an obstacle
			is_obstacle = true;
			break;
		}
	}
	if (!is_obstacle)
	{
		return false;
	}
	return true;
}


int stgPoseUpdateCB( Model* mod, ModelRobot* robot)
{
	if (!robot->name.compare("RL"))
	{
		geometry_msgs::PoseStamped positionMsgRl;
		positionMsgRl.pose.position.x = robot->pos->GetPose().x;
		positionMsgRl.pose.position.y = robot->pos->GetPose().y;
		positionMsgRl.pose.position.z = robot->pos->GetPose().z;
		positionMsgRl.pose.orientation = tf::createQuaternionMsgFromYaw( robot->pos->GetPose().a);
		positionMsgRl.header.stamp = ros::Time::now();
		rl_agent_pose = positionMsgRl;


		geometry_msgs::PoseStamped positionMsgH;
		positionMsgH.pose.position.x = robots_w[0]->pos->GetPose().x;
		positionMsgH.pose.position.y = robots_w[0]->pos->GetPose().y;
		positionMsgH.pose.position.z = robots_w[0]->pos->GetPose().z;
		positionMsgH.pose.orientation = tf::createQuaternionMsgFromYaw( robots_w[0]->pos->GetPose().a);
		positionMsgH.header.stamp = ros::Time::now();

		human_agent_pose = positionMsgH;
	}

  // update robot position in map

  if (robot->pos->GetWorld()->UpdateCount() - stgUpdateCyle > 1) // 100000)
	  robot->pos->SetSpeed( 0, 0, 0);


  ros::spinOnce();
  return 0;
}

int stgBlobCB( Model* mod, ModelRobot* robot)
{
    robot->blobs = robot->blob_finder->GetBlobs();
//    ROS_WARN_STREAM (""<< ros::Time::now() - robot->last_blob_update << "blob size:" << robot->blobs.size());
    robot->last_blob_update =ros::Time::now();
//    for (int i=0 ; i<robot->blobs.size() ; i++)
//    {
//        ModelBlobfinder::Blob t = robot->blobs.at(i);
//        ROS_WARN("Blob range:%f left:%d right:%d top:%d bottom:%d ",
//                t.range, t.left, t.right, t.top, t.bottom);
//    }
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
		  laserMsgs.intensities[i] = sensor.intensities[i];
		}

	  laserMsgs.header.stamp = ros::Time::now();
	  rosLaserData = laserMsgs;
	}


	collision = collision || robot->pos->TestCollision();
//	collision = robot->pos->TestCollision();
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
	stage_msg.position = rl_agent_pose;
	stage_msg.target = human_agent_pose;

	stage_msg.laser = rosLaserData;
			if ( robot->blobs.size()>0)
			{
					stage_msg.range = robot->blobs[0].range;
					stage_msg.top = robot->blobs[0].top;
					stage_msg.bottom = robot->blobs[0].bottom;
					stage_msg.left = robot->blobs[0].left;
					stage_msg.right = robot->blobs[0].right;
			}
			else // not detected
			{
				stage_msg.range = -1;
					stage_msg.top = -1;
					stage_msg.bottom = -1;
					stage_msg.left = -1;
					stage_msg.right = -1;
			}

	// publish the command velocity
	geometry_msgs::TwistStamped twist_msg;
	twist_msg.header.stamp = now;
	twist_msg.twist.linear.x = robot->pos->GetVelocity().x;
	twist_msg.twist.angular.z = robot->pos->GetVelocity().a;
	robot->pub_cmd_vel_.publish(twist_msg);

	robot->pub_state_.publish(stage_msg);
	show_robot(robot);


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

//  ROS_INFO("robot name: %s", robot->name.c_str());
	if (robot->name.compare("W"))
  		world->step();
}

bool rosResetSrvCB(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response, ModelRobot* robot)
{
//  ROS_INFO("Resetting stage!");
  robot->pos->SetPose( robot->resetPose );
	try
		{
			set_other_robot_pose(robot);
		}
		catch (std::runtime_error& e)
		{
			ROS_ERROR("couldnt set the other robot pose");
		}
  return true;
}

bool rosResetRandomSrvCB(follower_rl::reset_position::Request& request, follower_rl::reset_position::Response& response, ModelRobot* robot)
{
//  ROS_INFO("Resetting stage!");

  Pose pose;
  pose.x = request.position.position.x;
  pose.y = request.position.position.y;
  pose.z = 0;
  pose.a = request.position.orientation.w;
  robot->pos->SetPose(pose);

  try
	{
  	  pose.a = set_other_robot_pose(robot);
	}
	catch (std::runtime_error& e)
	{
		ROS_ERROR("couldnt set the other robot pose");
	}
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

Point2f to_image_coordinate(Point2f source, Point2f map)
{
    return Point2f
    (
        (source.x + World_Size.x/2) * (map.x/World_Size.x),
        (-source.y + World_Size.y/2) * ( map.y/World_Size.y)
    );
}

Point2f to_stage_coordinate(Point2f source, Point2f map)
{
    return Point2f
    (
        ((source.x * World_Size.x)/(float)map.x - World_Size.x/2),
        -((source.y *  World_Size.y)/(float)map.y - World_Size.y/2)
    );
}


int LaserCBWanderer( Model* mod, ModelRobot* robot)
{

//  ROS_INFO("in laser cv wanderer");

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


int RandU(int nMin, int nMax)
{
    return nMin + (int)((double)rand() / (RAND_MAX+1) * (nMax-nMin+1));
}

void show_robot(ModelRobot* robot)
{
	robot->visual_pos = robot->map.clone();
	Point2f robot_1 = to_image_coordinate(Point2f (robot->pos->GetPose().x, robot->pos->GetPose().y), Point2f(robot->map.cols, robot->map.rows));
	Point2f robot_2 = to_image_coordinate(Point2f (robots_w[0]->pos->GetPose().x, robots_w[0]->pos->GetPose().y), Point2f(robot->map.cols, robot->map.rows));
	float degree = robot->pos->GetPose().a;
	Point2f robot_1_end = Point2f(robot_1.x + sin(degree + M_PI/2)*4, robot_1.y + cos(degree + M_PI/2)*4);
	line(robot->visual_pos, robot_1, robot_1_end, (255, 0, 0));
	float degree_2 = robots_w[0]->pos->GetPose().a;
	Point2f robot_2_end = Point2f(robot_2.x + sin(degree_2+M_PI/2)*4, robot_2.y + cos(degree_2+M_PI/2)*4);
	line(robot->visual_pos, robot_2, robot_2_end, (255, 0, 0));
	circle(robot->visual_pos, robot_1, 4, (255, 0, 0), 1, 8);
	circle(robot->visual_pos, robot_2, 2, (255, 100, 100), 1, 8);
	nav_msgs::OccupancyGrid map_occupancy_grid_;
	cv_bridge::CvImage cv_ptr;
	cv_ptr.image = robot->visual_pos;
	cv_ptr.encoding = "mono8";
	cv_ptr.header = map_occupancy_grid_.header;
	robot->map_image_pub_.publish(cv_ptr.toImageMsg());
}

float set_other_robot_pose( ModelRobot* robot)
{
  robot->visual_pos = robot->map.clone();
  Point2f center = to_image_coordinate(Point2f (robot->pos->GetPose().x, robot->pos->GetPose().y), Point2f(robot->map.cols, robot->map.rows));
  Point2f test = to_stage_coordinate(center, Point2f(robot->map.cols, robot->map.rows));  cv::Size axes(50, 50);
  std::vector<Point> circle_points;
  ellipse2Poly(center, axes, 0, 0, 360, 1, circle_points);
  int wait_for_stable_point = 10;
   std::vector<Point2f> stable_points;
  for(int i = 0; i < circle_points.size(); i++){
//       ROS_WARN("center: %d %d size: %d", circle_points[i].x, circle_points[i].y,circle_points.size());
        if (check_obstacle (center, circle_points[i], robot) == false)
        {
            cv::Size ax(25, 25);
            std::vector<Point> circle_around_stable_points;
            ellipse2Poly(circle_points[i], ax, 0, 0, 360, 1, circle_around_stable_points);
            bool is_stable = true;
            for(int j = 0; j < circle_around_stable_points.size(); j++)
            {
//          ROS_WARN("center: %d %d to %d %d ", circle_points[i].x, circle_points[i].y,circle_around_stable_points[j].x, circle_around_stable_points[j].y);
                try
								{
                if (circle_around_stable_points[j].y < robot->visual_pos.rows && circle_around_stable_points[j].x < robot->visual_pos.cols && robot->visual_pos.at<uint8_t>(circle_around_stable_points[j]) == 0)
                    is_stable = false;
            		}
            		catch (cv::Exception& e)
            		{
            			is_stable = false;
            		}
            }
            if (is_stable==true)
            {
                circle(robot->visual_pos, circle_points[i], 1, (255, 0, 0), 1, 8);
                stable_points.push_back(circle_points[i]);
            }
        }
    }
		if (stable_points.size() == 0)
		{
			throw std::runtime_error("No stable point found");
		}
    int index = -RandU(0,stable_points.size()-1);
    circle(robot->visual_pos, stable_points.at(index), 5, (255, 255, 0), 1, 8);
    Point2f pose_other_robot = to_stage_coordinate(stable_points.at(index), Point2f(robot->map.cols, robot->map.rows));
    Pose pose;
    pose.x = pose_other_robot.x;
    pose.y = pose_other_robot.y;
    pose.z = 0;
    pose.a = atan2(pose.y - robot->pos->GetPose().y, pose.x - robot->pos->GetPose().x ) - M_PI;
    robots_w[0]->pos->SetPose(pose);
    circle(robot->visual_pos, center, 4, (255, 0, 0), 1, 8);
    return (pose.a);
}


void init_robot( ModelRobot* robot, std::string r_name, std::string r_number)
{
    srand((unsigned)time(0));

	Model* mod = world->GetModel(r_name + "_" + r_number);
	while (!mod)
	{
		mod = world->GetModel(r_name + "_" + r_number);
	}
	robot->avoidcount = 4;
	robot->pos = (ModelOurPosition*) mod;
	robot->pos->AddCallback( Model::CB_UPDATE, (model_callback_t)stgPoseUpdateCB, robot);
	robot->pos->Subscribe();
    robot->name = r_name;
	if (!r_name.compare("W"))
	{
		robot->laser = (ModelRanger*)mod->GetChild("ranger:0");
		robot->laser->AddCallback( Model::CB_UPDATE, (model_callback_t)LaserCBWanderer, robot);
		robot->laser->Subscribe();
		Model *floorplan = robot->pos->GetWorld()->GetModel("blank");
		ROS_INFO("adding cpu node %s_%s", r_name.c_str(), r_number.c_str());
		return ;
	}

	std::string path = ros::package::getPath("follower_rl");
	robot->map = imread(path+"/world/map.png", cv::IMREAD_GRAYSCALE);
	image_transport::ImageTransport image_transport_(*n);
	robot->map_image_pub_ = image_transport_.advertise("map_image", 1);
	robot->pub_state_ = n->advertise<follower_rl::stage_message>("input_data", 15);
	robot->pub_cmd_vel_ = n->advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped", 15);
	robot->sub_vel_ = n->subscribe<geometry_msgs::Twist>("cmd_vel", 15, boost::bind(&rosVelocityCB, _1, robot));
//	robot->reset_srv_ = n->advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("reset_positions", boost::bind(rosResetSrvCB, _1, _2, robot));
	robot->reset_random_srv_ = n->advertiseService<follower_rl::reset_position::Request, follower_rl::reset_position::Response>("reset_random_positions", boost::bind(&rosResetRandomSrvCB, _1, _2, robot) );

	robot->resetPose = robot->pos->GetPose();
	//    robot->pos->GetChild("ranger:0")->Subscribe();
	robot->blob_finder = (ModelBlobfinder*)mod->GetChild("blobfinder:0");
	robot->blob_finder->AddCallback( Model::CB_UPDATE, (model_callback_t)stgBlobCB, robot);
	robot->blob_finder->Subscribe();
	robot->laser = (ModelRanger*)mod->GetChild("ranger:0");
	robot->laser->AddCallback( Model::CB_UPDATE, (model_callback_t)stgLaserCB, robot);
	robot->laser->Subscribe();

	try
	{
  	set_other_robot_pose(robot);
	}
	catch (std::runtime_error& e)
	{
		ROS_ERROR("couldnt set the other robot pose");
	}
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

    // ROS_INFO("map width %d height %d", image.rows, image.cols);
    // cv::bitwise_not(image, image);
    // while (true){
    // imshow("show", image);
    //     cv::waitKey(0);
    // }
//      namedWindow( "show", WINDOW_AUTOSIZE );
//      imshow( "show", image  );
//  // cv::subtract(cv::Scalar:all(255),image,image);
   // initialize libstage
  Stg::Init( &argc, &argv );
//  world = new StepWorldGui(800, 700, "follow rl");
  world = new StepWorldGui("follow rl");
  world->Load( argv[1] );
  world->setPauseStatus(false);

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
    ROS_INFO("begin robots init");

	robots_w[0] = new ModelRobot;
	init_robot(robots_w[0], "W", "0");


	robots[0] = new ModelRobot;
	init_robot(robots[0], "RL", "0");

    ROS_INFO("end robots init");

  /*boost::thread spin_thread([](void) -> void {
	while (true)
	{
	  ros::spinOnce();
	}
  });*/
  boost::thread spin_thread(&spinThread);
  while (true)
  {
	Fl::wait();

  }

  return 0; //ok
}
