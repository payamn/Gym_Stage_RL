#ifndef CONFIG_H
#define CONFIG_H

// check if we need basic map
// #define IS_BASIC_MAP

#define IS_LOCAL 1
#define LOCAL_X 299
#define LOCAL_Y 299

// how large compared to the original footprint do you want your robot to be
#define ROBOT_SCALE_FACTOR .5

#define MAP_LEAST_COUNT 0.1 // in meters

// how big is the obstacle given by each laser beam (in pixel)
#define LASER_BEAM_WIDTH 1

// we have a gradient between these two colors
#define CURRENT_ROBOT_COLOR 255
#define PREVIOUS_ROBOT_TRAJECTORY_COLOR 100
#define POSITION_GRADIENT_DECAY 0.25

#define OBSTACLE_CHANNEL 0
#define EXPLORED_AREA_CHANNEL 1
#define ROBOT_TRAIL_CHANNEL 2

// #define OBSTACLE_COLOR 127
// #define EXPLORED_AREA_COLOR 30

#define EXPLORED_AREA_COLOR 170
#define RECENTLY_EXPLORED_AREA_COLOR 255
#define EXPLORED_AREA_GRADIENT_DECAY 0.1

#define OBSTACLE_COLOR 250

// in secs
#define RECENTLY_EXPLORED_PIXELS_TIME 0.6


#endif
