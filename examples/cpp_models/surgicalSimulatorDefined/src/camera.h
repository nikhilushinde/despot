/*
* This file contains the description for the camera class that is used by the robot to observe the environment
* that it is in. The camera class acts as a lidar with a certain field of view and samples at every theta spacing. 
* the lidar detects both the distance to the points and what kind of point it collided with. 
*/
#ifndef CAMERA_H
#define CAMERA_H

#include "surgical_utils.h"
#include "robot_arm.h"
#include "robot.h"
#include "obstacle.h"

namespace despot {

// the different type of intersections that the camera rays can have in the environment. 
enum intersectionType {
    int_top_wall = 0, 
    int_bottom_wall = 1, 
    int_left_wall = 2, 
    int_right_wall = 3, 
    int_obstacle_intersect = 4, 
    int_free_space = 5, 
    int_arm_intersect = 6,
};

// intersection point type - an array of these types is returned by the scan observation - (x,y) the point of intersection of the camera ray and the environment
struct cameraIntersectionPoint {
    float x;
    float y;
    intersectionType int_type;
    bool is_set; // indicator to indicate whether the point has been set / should be used - true means it should
};

// intersection distance type - an array of these types is returned by the distance scan observation 
struct cameraIntersectionDistance {
    float dist;
    intersectionType int_type;
    bool is_set; // indicator to indicate whether the point has been set / should be used - true means it should
};

// exception for incorrect return size on arrays in camera class functions
struct cameraArrayReturnSizeException: public std::exception {
    const char * what() const throw () {
        return "Incorrect size of return array in camera functions!!!";
    }
};
// error in initializing the camera fov
struct cameraFOVSetError: public std::exception {
    const char * what() const throw () {
        return "Incorrect field of view set for the camera - should be less than 360 degrees!!!";
    }
};

class camera {
public:
    // constructors
    camera(); // the default constructor
    camera(robotArmCoords camera_coords, int corresponding_arm_index, int fov_degrees, int angle_resolution_deg, int max_distance); // the full constructor
    camera(const camera &camera_to_copy); // copy constructor for the camera class

    // set and initialization functions
    void init_camera(robotArmCoords camera_coords, int corresponding_arm_index, int fov_degrees, int angle_resolution_deg, int max_distance); // function to initialize the camera - NOTE: Must be called after default constructor for proper operation

    // get functions
    robotArmCoords get_camera_coords() const;
    float get_fov_deg() const; // returns m_fov_deg
    float get_angle_resolution_deg() const; // returns m_angle_resolution_deg
    int get_corresponding_arm_index() const; // returns m_corresponding_arm_index
    float get_max_distance() const; // returns m_max_distance
    int get_num_scan_angles() const; // returns m_num_scan_angles
    void get_camera_scan_angles(float *ret_camera_scan_angles_deg, int ret_array_size) const; // returns an array of floats where each float is a scan angle of the camera in degrees

    // TODO: See if you need the intersection_greater_than_x flag and see if you really want to return by pass by value. 
    void get_arm_intersection_points(float scan_angle_deg, int num_robot_arms, const robot_arm *arms, cameraIntersectionPoint *ret_camera_intersection_pts, int ret_array_size) const; // returns the intersection points with all the arms along that scan angle
    void get_obstacle_intersection_points(float scan_angle_deg, int num_obstacles, const obstacle *obstacles,  cameraIntersectionPoint *ret_camera_intersection_pts, int ret_array_size) const; // returns all the intersection ponts with the obstacles along that angle
    void single_scan_perpendicular(float scan_angle_deg, int num_robot_arms, const robot_arm *arms, int num_obstacles, const obstacle *obstacles, cameraIntersectionPoint &ret_camera_intersection_point) const; // special case function for camera scans when the angle is 90 or -90 degrees
    void single_scan(float scan_angle_deg, int num_robot_arms, const robot_arm *arms, int num_obstacles, const obstacle *obstacles, cameraIntersectionPoint &ret_camera_intersection_point) const; // scans the environment using a single ray
    void scan_environment(int num_robot_arms, const robot_arm *arms, int num_obstacles, const obstacle *obstacles, cameraIntersectionPoint *ret_camera_intersection_points, int ret_array_size) const; // complete scan of the environment - scans over the entire field of view in angle resolution spacing
    void scan_environment_distance(int num_robot_arms, const robot_arm *arms, int num_obstacles, const obstacle *obstacles, cameraIntersectionDistance *ret_camera_intersection_distances, int ret_array_size) const; // complete scan of the environment - only returns the distances to the intersection points not the points themselves

    // state mutation and other functions
    void step_to_coord(robotArmCoords new_camera_coords); // steps the camera to the coordinate and orientation that is specified
    void find_startpt_slope(float scan_angle_deg, environmentCoords &start_pt, float &slope) const; // returns the start point and the slope in pixels of the line going through the camera_coords x and y with the specified angle (to simulate the camera rays)


private:

    // using '_m' suffix to denote class member attributes 

    robotArmCoords camera_coords_m; // the coordinates and orientation of the camera

    int corresponding_robot_arm_index_m; // the index of the robot arm that the camera is installed on
    float fov_deg_m; // the field of view of the camera in degrees - only allow integer field of views
    float angle_resolution_deg_m; // the spacing of the rays from the camera indegrees. 
    int max_distance_m; // the maximum distance that the camera rays can get a reading. 

    // calculated attributes - upon initialization
    int num_scan_angles_m; // the number of scan angles of the camera - is set after calculation using m_fov_deg and m_angle_resolution_deg in init_camera function

    // TODO: ADD SCAN AND LAST_ENVIRONMENT_SCAN VARIABLES - figure out the datatype for the scan. - see if this is even needed. 
};

} // end namespace despot

#endif