/*
* This file contains function descriptions for the camera class that is described in "camera.h"
* NOTE: THE CAMERA DOES NOT SUPPORT FULL 360 DEGREE VIEWING. - fov should be less than 360. - this is as the camera
* does not consider intersections of its rays with the arm that it is on so results in strange behavior. 
* Since the camera is mounted on an arm adding support for 360 degree viewing would not make sense. 
*/

#include "camera.h"

using std::cout;
using std::cerr;
using std::endl;
using std::min;
using std::max;
using std::min_element;
using std::distance;

namespace despot {
/*
* ********************************************************************************
* BEGIN: CAMERA CONSTRUCTOR
* ********************************************************************************
*/

camera::camera() {
    /*
    * The default constructor for the camera. Initializes all numerical attributes to zero. 
    * NOTE: make sure to call init_camera before using the camera - otherwise the code will error. 
    */ 
    camera_coords_m.x = 0;
    camera_coords_m.y = 0;
    camera_coords_m.theta_degrees = 0;
    
    corresponding_robot_arm_index_m = 0;
    fov_deg_m = 0;
    num_scan_angles_m = 0;
    angle_resolution_deg_m = 0;
    max_distance_m = 0;

    debug_m = DEFAULT_DEBUG_FLAG_g;
}

camera::camera(robotArmCoords camera_coords, int corresponding_arm_index, int fov_degrees, int angle_resolution_deg, int max_distance) {
    /*
    * Full constructor of the camera class
    */
    init_camera(camera_coords, corresponding_arm_index, fov_degrees, angle_resolution_deg, max_distance);
}

camera::camera(const camera &camera_to_copy) {
    /*
    * Copy constructor for the camera class
    * args:
    *   - camera_to_copy: the object to copy from 
    */ 
    
    camera_coords_m = camera_to_copy.camera_coords_m;

    corresponding_robot_arm_index_m = camera_to_copy.corresponding_robot_arm_index_m;
    fov_deg_m = camera_to_copy.fov_deg_m;
    angle_resolution_deg_m = camera_to_copy.angle_resolution_deg_m;
    max_distance_m = camera_to_copy.max_distance_m;

    num_scan_angles_m = camera_to_copy.num_scan_angles_m;
    debug_m = camera_to_copy.debug_m;
}

/*
* ********************************************************************************
* END: CAMERA CONSTRUCTORS
* ********************************************************************************
*/

/*
* ********************************************************************************
* BEGIN: CAMERA Set and Initialization functions 
* ********************************************************************************
*/

void camera::init_camera(robotArmCoords camera_coords, int corresponding_arm_index, int fov_degrees, int angle_resolution_deg, int max_distance) {
    /*
    * Initialize the camera class completely. Call this function after the default constructor for the camera class to work properly
    */ 
    if (fov_degrees >= 360) {
        throw cameraFOVSetError();
        return;
    }

    camera_coords_m = camera_coords;
    
    corresponding_robot_arm_index_m = corresponding_arm_index;
    fov_deg_m = fov_degrees;
    angle_resolution_deg_m = angle_resolution_deg;
    max_distance_m = max_distance;
    
    // calculated camera attributes 
    num_scan_angles_m = 0;
    float half_fov_deg = fov_deg_m/2.0;
    // the positive half of the scan angles
    for (float angle_deg = 0; angle_deg <= half_fov_deg; angle_deg += angle_resolution_deg_m) {
        num_scan_angles_m ++;
    }
    // the negative half of the scan angles
    for (float angle_deg = -angle_resolution_deg_m; angle_deg >= -half_fov_deg; angle_deg -= angle_resolution_deg_m) {
        num_scan_angles_m ++;
    }

    debug_m = DEFAULT_DEBUG_FLAG_g;
}

/*
* ********************************************************************************
* END: CAMERA Set and Initialization functions 
* ********************************************************************************
*/

/*
* ********************************************************************************
* BEGIN: CAMERA GET functions
* ********************************************************************************
*/
robotArmCoords camera::get_camera_coords() const{
    return camera_coords_m;
}
float camera::get_fov_deg() const{
    return fov_deg_m;
} 
float camera::get_angle_resolution_deg() const{
    return angle_resolution_deg_m;
}
int camera::get_corresponding_arm_index() const{
    return corresponding_robot_arm_index_m;
}
float camera::get_max_distance() const{
    return max_distance_m;
}
int camera::get_num_scan_angles() const{
    return num_scan_angles_m;
}
void camera::get_camera_scan_angles(float *ret_camera_scan_angles_deg, int ret_array_size) const{
    /*
    * Populate the ret_camera_scan_angles_deg list with all the angles that the robot will be scanning 
    * These angles already account for the offset from the camera's angle.
    * 
    * The angles together with the (x,y) coordinate of the camera completely characterize the camera ray 
    * line that gets the reading from the environment. 
    */ 
    if (ret_array_size != num_scan_angles_m) {
        cerr << "Error in get camera scan angles" << endl;
        throw cameraArrayReturnSizeException();
        return;
    }

    int counter = ret_array_size;
    float half_fov_deg = fov_deg_m/2.0;
    int half_array_size = (int) floor(ret_array_size/2); // this is the index (since we are 0 indexing) of the 0 angle in the array 
    int array_index = half_array_size;

    // populate the array with the positive scan angles
    for (float angle_deg = 0; angle_deg <= half_fov_deg; angle_deg += angle_resolution_deg_m) {
        ret_camera_scan_angles_deg[array_index] = angle_deg + camera_coords_m.theta_degrees;
        array_index ++;
    }
    if (num_scan_angles_m > 1) {
        // if the number of scan angles is greater than 1 then there must be a nonzero angle
        array_index = half_array_size - 1;
    }
    // populate the array with the negative scan angles
    for (float angle_deg = -angle_resolution_deg_m; angle_deg >= -half_fov_deg; angle_deg -= angle_resolution_deg_m) {
        ret_camera_scan_angles_deg[array_index] = angle_deg + camera_coords_m.theta_degrees;
        array_index --;
    }
    return;
}

// ADVANCED GET FUNCTIONS 
void camera::get_arm_intersection_points(float scan_angle_deg, int num_robot_arms, const robot_arm *arms, cameraIntersectionPoint *ret_camera_intersection_pts, int ret_array_size) const{
    /*
    * get the intersection points of the camera ray intersecting with all possible robot arms. 
    * args:
    *   - scan_angle_deg: the scan angle of the camera array (describes the line in conjunction with the (x,y) coordinates of the camera)
    *   - num_robot_arms: number of robot arms
    *   - arms: pointer to the list of robot arms
    *   - intersection_greater_than_x: if the intersecton point should be greater than or equal to x. 
    *   - ret_array_size: the size of the return array 
    * returns: 
    *   - ret_camera_intersection_pts: this list is num_robot_arms long and contains the intersection point for each robot arm where the 
    *       the index corresponds to the intersected robot arm. If no intersection the set_point attribute of the intersectoin point struct is 
    *       set to false. 
    * 
    * NOTE: THE RETURN ARRAY SIZE must be set to THE NUMBER OF ROBOT ARMS.
    * NOTE: THE ANGLES ARE INVERTED - NEGATIVE ANGLES LOOK UPWARDS AND POSITIVE ANGLES LOOK DOWNWARDS 
    */ 
    // should the camera ray intersection point have an x coordinate greater than the camera x coordinate. 
    bool intersection_greater_than_x = (-90 < scan_angle_deg && scan_angle_deg < 90);
    if (num_robot_arms != ret_array_size) {
        cerr << "Error in get arm intersection points" << endl;
        throw cameraArrayReturnSizeException();
        return;
    }
    // start by setting all the intersection points to not be set and to default values. 
    for (int i = 0; i < num_robot_arms; i++) {
        ret_camera_intersection_pts[i].x = 0;
        ret_camera_intersection_pts[i].y = 0;
        ret_camera_intersection_pts[i].int_type = int_free_space;
        ret_camera_intersection_pts[i].is_set = false;
    }

    // if there are no visible robot arms just return - no inersection points
    bool no_visible_robot_arms = true;
    for (int arm_num = 0; arm_num < num_robot_arms; arm_num++) {
        if (arms[arm_num].get_x() > 0) {
            no_visible_robot_arms = false;
            break;
        }
    }
    if (no_visible_robot_arms) {
        return;
    }


    float intersect_distance;
    if (scan_angle_deg == 90 || scan_angle_deg == -90) {
        // special case to handle perpendicular scans
        environmentCoords robot_arm_start_pt;
        float robot_arm_m, robot_arm_b;
        float intersect_x, intersect_y;
        for (int arm_index = 0; arm_index < num_robot_arms; arm_index ++) {
            // camera ray cannot intersect with the arm that it is on 
            if (arm_index == corresponding_robot_arm_index_m) {
                continue;
            }

            arms[arm_index].find_startpt_slope(robot_arm_start_pt, robot_arm_m);
            robot_arm_b = robot_arm_start_pt.y;

            // find the intersection point of the two lines
            intersect_x = camera_coords_m.x;
            intersect_y = (robot_arm_m * intersect_x) + robot_arm_b;

            // if the intersection points are not within the max distance continue to the next arm
            intersect_distance = sqrt(pow(intersect_x - camera_coords_m.x, 2) + pow(intersect_y - camera_coords_m.y, 2));
            if (intersect_distance > max_distance_m) {
                continue;
            }

            // check that the intersection point is actually on the robot arm and not the extrapolated line
            if ((intersect_x >= 0 || cmp_floats(intersect_x, 0.0)) && (intersect_x <= arms[arm_index].get_x() || cmp_floats(intersect_x, arms[arm_index].get_x())) ) {
                // handle 90 and -90 degree angles appropriately
                if (scan_angle_deg == -90) { // scan above
                    // TODO: MAYBE ADD ROUNDING IN COMPARISON HERE
                    if (intersect_y > camera_coords_m.y) {
                        ret_camera_intersection_pts[arm_index].x = intersect_x;
                        ret_camera_intersection_pts[arm_index].y = intersect_y;
                        ret_camera_intersection_pts[arm_index].int_type = int_arm_intersect;
                        ret_camera_intersection_pts[arm_index].is_set = true;
                    }
                } else { // scan below
                    if (intersect_y < camera_coords_m.y) {
                        ret_camera_intersection_pts[arm_index].x = intersect_x;
                        ret_camera_intersection_pts[arm_index].y = intersect_y;
                        ret_camera_intersection_pts[arm_index].int_type = int_arm_intersect;
                        ret_camera_intersection_pts[arm_index].is_set = true;
                    }
                }
            }
        }
    } else { 
        // handles all other angles
        environmentCoords robot_arm_start_pt, camera_ray_start_pt;
        float robot_arm_m, robot_arm_b, camera_ray_m, camera_ray_b;
        float intersect_x, intersect_y;

        this->find_startpt_slope(scan_angle_deg, camera_ray_start_pt, camera_ray_m);
        camera_ray_b = camera_ray_start_pt.y;

        for (int arm_index = 0; arm_index < num_robot_arms; arm_index ++) {
            // camera ray cannot intersect with the arm that it is on 
            if (arm_index == corresponding_robot_arm_index_m) {
                continue;
            }
            
            arms[arm_index].find_startpt_slope(robot_arm_start_pt, robot_arm_m);
            robot_arm_b = robot_arm_start_pt.y;

            // TODO: NOTE: Might want to remove this code - shouldn't enter this. 
            // deal with the case that camera is in the middle of an arm (if arms allowed to overlap)
            if (robot_arm_m == camera_ray_m && robot_arm_b == camera_ray_b) {

                // if the intersection points are not within the max distance continue to the next arm
                intersect_distance = sqrt(pow(arms[arm_index].get_x() -camera_coords_m.x, 2) + pow(arms[arm_index].get_y() -camera_coords_m.y, 2));
                if (intersect_distance > max_distance_m) {
                    continue;
                }

                if (camera_coords_m.x <= arms[arm_index].get_x()) {
                    // if the arm is ahead
                    ret_camera_intersection_pts[arm_index].x =camera_coords_m.x;
                    ret_camera_intersection_pts[arm_index].y =camera_coords_m.y;
                    ret_camera_intersection_pts[arm_index].int_type = int_arm_intersect;
                    ret_camera_intersection_pts[arm_index].is_set = true;
                } else {
                    // if arm is not ahead
                    ret_camera_intersection_pts[arm_index].x = arms[arm_index].get_x();
                    ret_camera_intersection_pts[arm_index].y = arms[arm_index].get_y();
                    ret_camera_intersection_pts[arm_index].int_type = int_arm_intersect;
                    ret_camera_intersection_pts[arm_index].is_set = true;
                }
            }

            // if the camera ray and robot arm slopes are they same and intercepts are not the same - lines should not intersect
            if (camera_ray_m != robot_arm_m) {
                intersect_x = (robot_arm_b - camera_ray_b)/(camera_ray_m - robot_arm_m);
                intersect_y = (camera_ray_m * intersect_x) + camera_ray_b;


                // if the intersection points are not within the max distance continue to the next arm
                // if the intersection points are not within the max distance continue to the next arm
                intersect_distance = sqrt(pow(intersect_x -camera_coords_m.x, 2) + pow(intersect_y -camera_coords_m.y, 2));
                if (intersect_distance > max_distance_m) {
                    continue;
                }

                if ((intersect_x >= 0 || cmp_floats(intersect_x, 0.0)) && (intersect_x <= arms[arm_index].get_x() || cmp_floats(intersect_x, arms[arm_index].get_x()))) {
                    if (intersection_greater_than_x) {
                        if (intersect_x >=camera_coords_m.x || cmp_floats(intersect_x,camera_coords_m.x)) {
                            ret_camera_intersection_pts[arm_index].x = intersect_x;
                            ret_camera_intersection_pts[arm_index].y = intersect_y;
                            ret_camera_intersection_pts[arm_index].int_type = int_arm_intersect;
                            ret_camera_intersection_pts[arm_index].is_set = true;
                        }
                    } else {
                        if (intersect_x <= camera_coords_m.x || cmp_floats(intersect_x, camera_coords_m.x)) {
                            ret_camera_intersection_pts[arm_index].x = intersect_x;
                            ret_camera_intersection_pts[arm_index].y = intersect_y;
                            ret_camera_intersection_pts[arm_index].int_type = int_arm_intersect;
                            ret_camera_intersection_pts[arm_index].is_set = true;
                        }
                    }
                }
            }
        }
    }
    return;
}

void camera::get_obstacle_intersection_points(float scan_angle_deg, int num_obstacles, const obstacle *obstacles, cameraIntersectionPoint *ret_camera_intersection_pts, int ret_array_size) const{
    /*
    * get the intersection points of the camera ray intersecting with all the possible obstacles. 
    * args:
    *   - scan_angle_deg: the scan angle of the camera array (describes the line in conjunction with the (x,y) coordinates of the camera)
    *   - num_obstacles: number of obstacles
    *   - obstacles: pointer to the list of obstacles
    *   - ret_array_size: the size of the return array 
    * returns: 
    *   - ret_camera_intersection_pts: this list is num_robot_arms long and contains the intersection point for each robot arm where the 
    *       the index corresponds to the intersected obstacle - matches up with the indexes in the obstacles list.
    *       If no intersection the set_point attribute of the intersectoin point struct is set to false. 
    * 
    * NOTE: THE RETURN ARRAY SIZE must be set to THE NUMBER OF OBSTACLES. 
    * NOTE: THE ANGLES ARE INVERTED - NEGATIVE ANGLES LOOK UPWARDS AND POSITIVE ANGLES LOOK DOWNWARDS
    */ 
    // should the intersection point be greater than the camera coordinate value 
    bool intersection_greater_than_x = (-90 < scan_angle_deg && scan_angle_deg < 90);


    if (num_obstacles != ret_array_size) {
        cerr << "Error in get obstacle intersection points" << endl;
        throw cameraArrayReturnSizeException();
        return;
    }

    // start by setting all the intersection points to not be set and to default values. 
    for (int i = 0; i < num_obstacles; i++) {
        ret_camera_intersection_pts[i].x = 0;
        ret_camera_intersection_pts[i].y = 0;
        ret_camera_intersection_pts[i].int_type = int_free_space;
        ret_camera_intersection_pts[i].is_set = false;
    }
    if (num_obstacles <= 0) {
        return;
    }

    float intersect_distance;
    if (scan_angle_deg == 90 || scan_angle_deg == -90) {
        // special case to deal with perpendicular obstacle scans
        float delta_x;
        float pos_delta_y, pos_int_y, pos_int_dist, neg_delta_y, neg_int_y, neg_int_dist; 
        float obstacle_radius;
        float inside_sqrt; 
        environmentCoords obstacle_center;
        bool pos_int_set = false; // boolean indicator whether the positive intersection point was chosen 
        
        for (int obstacle_num = 0; obstacle_num < num_obstacles; obstacle_num ++) {
            pos_int_set = false;
            obstacle_center = obstacles[obstacle_num].get_center();
            obstacle_radius = (float) obstacles[obstacle_num].get_radius();
            delta_x = abs(camera_coords_m.x - obstacle_center.x);

            inside_sqrt = pow(obstacle_radius, 2) - pow(delta_x, 2);
            if (cmp_floats(inside_sqrt, 0.0)) {
                inside_sqrt = 0;
            }

            if (inside_sqrt >= 0) { // To ensure delta_y values will not be nan
                pos_delta_y = sqrt(inside_sqrt); // pythagorean theorem used to find the delta_y for intersection point
                pos_int_y = obstacle_center.y + pos_delta_y;
                neg_delta_y = - pos_delta_y; 
                neg_int_y = obstacle_center.y + neg_delta_y;

                // since we have perpendicular scans - camera ray straight up or down - thus intersect_x - camera_x = 0
                pos_int_dist = sqrt( pow(pos_int_y - camera_coords_m.y, 2));
                neg_int_dist = sqrt( pow(neg_int_y - camera_coords_m.y, 2));
                // the closer of the two intersection points will be registered by the camera ray - and if the closer point violates some other conditions - check if the other point should be set
                if ((pos_int_dist <= max_distance_m || cmp_floats(pos_int_dist, max_distance_m))  && (pos_int_dist <= neg_int_dist || cmp_floats(pos_int_dist, neg_int_dist)) ) {
                    if (scan_angle_deg == -90 && (pos_int_y >= camera_coords_m.y || cmp_floats(pos_int_y, camera_coords_m.y)) ) { // check above
                        ret_camera_intersection_pts[obstacle_num].x = camera_coords_m.x;
                        ret_camera_intersection_pts[obstacle_num].y = pos_int_y;
                        ret_camera_intersection_pts[obstacle_num].int_type = int_obstacle_intersect;
                        ret_camera_intersection_pts[obstacle_num].is_set = true;

                        pos_int_set = true;
                    } else if (scan_angle_deg == 90 && (pos_int_y <= camera_coords_m.y || cmp_floats(pos_int_y, camera_coords_m.y)) ) { // check below
                        ret_camera_intersection_pts[obstacle_num].x = camera_coords_m.x;
                        ret_camera_intersection_pts[obstacle_num].y = pos_int_y;
                        ret_camera_intersection_pts[obstacle_num].int_type = int_obstacle_intersect;
                        ret_camera_intersection_pts[obstacle_num].is_set = true;

                        pos_int_set = true;
                    }
                } else if ((neg_int_dist <= max_distance_m || cmp_floats(neg_int_dist, max_distance_m)) && (neg_int_dist < pos_int_dist || !pos_int_set)) {
                    // positive intersection point was not set, check to see if the negative intersection point should be set
                    if (scan_angle_deg == -90 && (neg_int_y >= camera_coords_m.y || cmp_floats(neg_int_y, camera_coords_m.y)) ) { // check above
                        ret_camera_intersection_pts[obstacle_num].x = camera_coords_m.x;
                        ret_camera_intersection_pts[obstacle_num].y = neg_int_y;
                        ret_camera_intersection_pts[obstacle_num].int_type = int_obstacle_intersect;
                        ret_camera_intersection_pts[obstacle_num].is_set = true;
                    } else if (scan_angle_deg == 90 && (neg_int_y <= camera_coords_m.y || cmp_floats(neg_int_y, camera_coords_m.y))) { // check below
                        ret_camera_intersection_pts[obstacle_num].x = camera_coords_m.x;
                        ret_camera_intersection_pts[obstacle_num].y = neg_int_y;
                        ret_camera_intersection_pts[obstacle_num].int_type = int_obstacle_intersect;
                        ret_camera_intersection_pts[obstacle_num].is_set = true;
                    }
                }
            }
        } 
    } else {
        
        if (debug_m) {
            cout << endl << endl << "Inside the OBSTACLE GET INTERSECTION POINTS FUNCTION NOT 90" << endl;;
            cout << "Inside the OBSTACLE GET INTERSECTION POINTS FUNCTION NOT 90 " << endl;
            cout << scan_angle_deg << endl;
            cout << num_obstacles << endl ;    
        }

        // handles scanning obstacles with all other angles
        float a_qf, b_qf, c_qf;
        float pos_int_x, pos_int_y, pos_int_dist, neg_int_x, neg_int_y, neg_int_dist; 
        float obstacle_radius;
        float inside_sqrt; 
        environmentCoords obstacle_center;
        bool pos_int_set = false; // boolean indicator whether the positive intersection point was chosen 
        bool intersection_x_criteria; // boolean indicator on whether the intersection point meets the intersection_greater_than_x criteria

        environmentCoords camera_ray_start_pt;
        float camera_ray_m, camera_ray_b;
        this->find_startpt_slope(scan_angle_deg, camera_ray_start_pt, camera_ray_m);
        camera_ray_b = camera_ray_start_pt.y;
        for (int obstacle_num = 0; obstacle_num < num_obstacles; obstacle_num++) {
            // reset boolean indicators for each obstacle loop
            pos_int_set = false;
            intersection_x_criteria = false;

            obstacle_center = obstacles[obstacle_num].get_center();
            obstacle_radius = obstacles[obstacle_num].get_radius();

            a_qf = 1 + pow(camera_ray_m, 2);
            b_qf = 2*(camera_ray_m * (camera_ray_b - obstacle_center.y) - obstacle_center.x);
            c_qf = pow(obstacle_center.x, 2) + pow((camera_ray_b - obstacle_center.y), 2) - pow(obstacle_radius, 2);

            inside_sqrt = pow(b_qf, 2) - (4 * a_qf * c_qf);
            if (cmp_floats(inside_sqrt, 0)) {
                inside_sqrt = 0;
            }

            if (debug_m) {
                cout << "OBSTACLE GET INTERSECTION FUNCTION: THE OBSTACLE RADIUS IS:" << obstacle_radius << " from "<< obstacles[obstacle_num].get_radius() << endl;
                cout << camera_ray_m << ", " << camera_ray_b << ", " << a_qf << ", " << b_qf << ", " << c_qf << endl;
                cout << "inside sqrt: " << inside_sqrt << endl << endl;
            }
            if (inside_sqrt >= 0) { // ensure that it is not nan
                pos_int_x = (-b_qf + sqrt(inside_sqrt))/(2*a_qf);
                pos_int_y = (camera_ray_m * pos_int_x) + camera_ray_b;
                neg_int_x = (-b_qf - sqrt(inside_sqrt))/(2*a_qf);
                neg_int_y = (camera_ray_m * neg_int_x) + camera_ray_b;

                // since we have perpendicular scans - camera ray straight up or down - thus intersect_x - camera_x = 0
                pos_int_dist = sqrt(pow(pos_int_x - camera_coords_m.x, 2) + pow(pos_int_y - camera_coords_m.y, 2));
                neg_int_dist = sqrt(pow(neg_int_x - camera_coords_m.x, 2) + pow(neg_int_y - camera_coords_m.y, 2));
                
                if (debug_m) {
                    cout << "IN find OBSTACLE INTERSECTION function: " << endl;
                    cout << "pos: (" << pos_int_x << "," << pos_int_y << ")" << endl;
                    cout << "neg: (" << neg_int_x << "," << neg_int_y << ")" << endl;
                    cout << "posintdist: " << pos_int_dist << "negintdist: " << neg_int_dist << endl;
                }
                // the closer of the two intersection points will be registered by the camera ray - and if the closer point violates some other conditions - check if the other point should be set
                if ((pos_int_dist <= max_distance_m || cmp_floats(pos_int_dist, max_distance_m)) && (pos_int_dist <= neg_int_dist || cmp_floats(pos_int_dist, neg_int_dist)) ) {

                    if (intersection_greater_than_x) {
                        if (pos_int_x >= camera_coords_m.x || cmp_floats(pos_int_x, camera_coords_m.x)) {
                            intersection_x_criteria = true;
                        } 
                    } else {
                        if (pos_int_x <= camera_coords_m.x || cmp_floats(pos_int_x, camera_coords_m.x)) {
                            intersection_x_criteria = true;
                        } 
                    }

                    if (intersection_x_criteria) {
                        ret_camera_intersection_pts[obstacle_num].x = pos_int_x;
                        ret_camera_intersection_pts[obstacle_num].y = pos_int_y;
                        ret_camera_intersection_pts[obstacle_num].int_type = int_obstacle_intersect;
                        ret_camera_intersection_pts[obstacle_num].is_set = true;

                        pos_int_set = true;
                    } 
                } else if ((neg_int_dist <= max_distance_m || cmp_floats(neg_int_dist, max_distance_m)) && (neg_int_dist < pos_int_dist || !pos_int_set)) {
                    // positive intersection point was not set, check to see if the negative intersection point should be set
                    if (intersection_greater_than_x) {
                        if (neg_int_x >= camera_coords_m.x || cmp_floats(neg_int_x, camera_coords_m.x)) {
                            intersection_x_criteria = true;
                        } 
                    } else {
                        if (neg_int_x <= camera_coords_m.x || cmp_floats(neg_int_x, camera_coords_m.x)) {
                            intersection_x_criteria = true;
                        } 
                    }

                    if (intersection_x_criteria) {
                        ret_camera_intersection_pts[obstacle_num].x = neg_int_x;
                        ret_camera_intersection_pts[obstacle_num].y = neg_int_y;
                        ret_camera_intersection_pts[obstacle_num].int_type = int_obstacle_intersect;
                        ret_camera_intersection_pts[obstacle_num].is_set = true;
                    } else {
                        if (debug_m) {
                            cout << "neg didnt meet intersection x criteria" << endl;
                            cout << "CAMERA: " << camera_coords_m.x << "," << camera_coords_m.y << endl;
                            cout << "INTERSECTION POINT: " << neg_int_x << "," << neg_int_y << endl;
                            cout << "x equal: " << (neg_int_x == camera_coords_m.x) << ", x difference: " <<  neg_int_x - camera_coords_m.x << endl;
                        }
                    }
                }
            }
        }
    }
    return;
}

void camera::single_scan_perpendicular(float scan_angle_deg, int num_robot_arms, const robot_arm *arms, int num_obstacles, const obstacle *obstacles, cameraIntersectionPoint &ret_camera_intersection_point) const{
    /*
    * Special case function of single array scan for the 90 and -90 degree perpendicular scans (do not follow y = mx + b). 
    * args;
    *   - scan_angle_deg: the scan angle of the camera array in degrees
    *   - num_robot_arms: the number of robot arms
    *   - arms: the pointer that is a list of robot arm objects
    *   - num_obstacles: the number of obstacles
    *   - obstacles: a pointer that is a list of obstacles
    * returns: by pass by reference
    *   - ret_camera_intersection_point: returns the intersection point of the camera array in the environment
    * 
    * NOTE: THE ANGLES ARE INVERTED - NEGATIVE ANGLES LOOK UPWARDS AND POSITIVE ANGLES LOOK DOWNWARDS
    */ 

    cameraIntersectionPoint camera_arm_intersection_pts[num_robot_arms];
    cameraIntersectionPoint camera_obstacle_intersection_pts[num_obstacles]; 
    float distances[num_robot_arms + num_obstacles];
    bool any_set = false; // boolean indicator - true if even a single intersection point has been set for arms or obstacles

    this->get_arm_intersection_points(scan_angle_deg, num_robot_arms, arms, camera_arm_intersection_pts, num_robot_arms);
    this->get_obstacle_intersection_points(scan_angle_deg, num_obstacles, obstacles, camera_obstacle_intersection_pts, num_obstacles);
    for (int arm_num = 0; arm_num < num_robot_arms; arm_num ++) {
        if (camera_arm_intersection_pts[arm_num].is_set) {
            any_set = true;
            distances[arm_num] = sqrt(pow(camera_arm_intersection_pts[arm_num].x - camera_coords_m.x, 2) + 
                                      pow(camera_arm_intersection_pts[arm_num].y - camera_coords_m.y, 2));
        } else {
            // if not set then just set the distance to be 1 greater than the max distance allowed - essentially an invalid value
            distances[arm_num] = max_distance_m + 1;
        }
    }
    for (int obs_num = 0; obs_num < num_obstacles; obs_num ++) {
        if (camera_obstacle_intersection_pts[obs_num].is_set) {
            any_set = true;
            distances[obs_num + num_robot_arms] = sqrt(pow(camera_obstacle_intersection_pts[obs_num].x - camera_coords_m.x, 2) + 
                                      pow(camera_obstacle_intersection_pts[obs_num].y - camera_coords_m.y, 2));
        } else {
            // if not set then just set the distance to be 1 greater than the max distance allowed - essentially an invalid value
            distances[obs_num + num_robot_arms] = max_distance_m + 1;
        }
    }


    if (!any_set) {
        // no intersection points found so the camera found only free space in that direction
        float delta_y_direction;
        if (scan_angle_deg == 90) {
            delta_y_direction = -1;
        } else {
            delta_y_direction = 1;
        }

        ret_camera_intersection_point.x = camera_coords_m.x;
        ret_camera_intersection_point.y = min(max(camera_coords_m.y + (delta_y_direction * max_distance_m), (float) 0.0), (float) ENV_HEIGHT_g);

        if (ret_camera_intersection_point.y == 0) {
            ret_camera_intersection_point.int_type = int_bottom_wall;
        } else if (ret_camera_intersection_point.y == ENV_HEIGHT_g) {
            ret_camera_intersection_point.int_type = int_top_wall;
        } else {
            ret_camera_intersection_point.int_type = int_free_space;
        }
        ret_camera_intersection_point.is_set = true;
        return;
    }

    // if any intersection points were found. 
    int min_dist_index = distance(distances, min_element(distances, distances + num_robot_arms + num_obstacles));
    if (min_dist_index < num_robot_arms) {
        ret_camera_intersection_point = camera_arm_intersection_pts[min_dist_index];
    } else {
        ret_camera_intersection_point = camera_obstacle_intersection_pts[min_dist_index - num_robot_arms];
    }

    return;
}

void camera::single_scan(float scan_angle_deg, int num_robot_arms, const robot_arm *arms, int num_obstacles, const obstacle *obstacles, cameraIntersectionPoint &ret_camera_intersection_point) const{
    /*
    * Scans using a camera ray of scan_angle_deg in the environment and returns the point of intersection in the environment using pass by reference. 
    * args;
    *   - scan_angle_deg: the scan angle of the camera array in degrees
    *   - num_robot_arms: the number of robot arms
    *   - arms: the pointer that is a list of robot arm objects
    *   - num_obstacles: the number of obstacles
    *   - obstacles: a pointer that is a list of obstacles
    * returns: by pass by reference
    *   - ret_camera_intersection_point: returns the intersection point of the camera array in the environment
    */ 

    ret_camera_intersection_point.x = 0;
    ret_camera_intersection_point.y = 0;
    ret_camera_intersection_point.int_type = int_free_space;
    ret_camera_intersection_point.is_set = false;

    // if the camera is not in the visible domain just return without setting anything
    if (camera_coords_m.x <= 0) {
        return;
    }

    if (scan_angle_deg == 90 || scan_angle_deg == -90) {
        cameraIntersectionPoint perp_camera_intersection_point;
        this->single_scan_perpendicular(scan_angle_deg, num_robot_arms, arms, num_obstacles, obstacles, perp_camera_intersection_point);
        ret_camera_intersection_point = perp_camera_intersection_point;
        return;
    }
    cameraIntersectionPoint camera_arm_intersection_pts[num_robot_arms];
    cameraIntersectionPoint camera_obstacle_intersection_pts[num_obstacles];
    float distances[num_robot_arms + num_obstacles];
    bool any_set = false; // boolean indicator - true if even a single intersection point has been set for arms or obstacles

    this->get_arm_intersection_points(scan_angle_deg, num_robot_arms, arms, camera_arm_intersection_pts, num_robot_arms);
    this->get_obstacle_intersection_points(scan_angle_deg, num_obstacles, obstacles, camera_obstacle_intersection_pts, num_obstacles);
    for (int arm_num = 0; arm_num < num_robot_arms; arm_num ++) {
        if (camera_arm_intersection_pts[arm_num].is_set) {
            any_set = true;
            distances[arm_num] = sqrt(pow(camera_arm_intersection_pts[arm_num].x - camera_coords_m.x, 2) + 
                                      pow(camera_arm_intersection_pts[arm_num].y - camera_coords_m.y, 2));
        } else {
            // if not set then just set the distance to be 1 greater than the max distance allowed - essentially an invalid value
            distances[arm_num] = max_distance_m + 1;
        }
    }
    for (int obs_num = 0; obs_num < num_obstacles; obs_num ++) {
        if (camera_obstacle_intersection_pts[obs_num].is_set) {
            any_set = true;
            distances[obs_num + num_robot_arms] = sqrt(pow(camera_obstacle_intersection_pts[obs_num].x - camera_coords_m.x, 2) + 
                                      pow(camera_obstacle_intersection_pts[obs_num].y - camera_coords_m.y, 2));
        } else {
            // if not set then just set the distance to be 1 greater than the max distance allowed - essentially an invalid value
            distances[obs_num + num_robot_arms] = max_distance_m + 1;
        }
    }

    if (debug_m) {
        cout << "in single scan camera: all the distances: " << endl;
        for (int i = 0; i < num_robot_arms + num_obstacles; i ++) {
            cout <<   distances[i] << ",";
        }
        cout << endl;
    }

    if (!any_set) {
        float camera_ray_m, camera_ray_b;
        environmentCoords camera_ray_start_pt;
        this->find_startpt_slope(scan_angle_deg, camera_ray_start_pt, camera_ray_m);
        camera_ray_b = camera_ray_start_pt.y;

        // if nothing set, just return an intersection point max_distance away in the appropriate direction (bounded)
        ret_camera_intersection_point.x = min(max((max_distance_m * cos(scan_angle_deg * THETA_MULTIPLIER)) + camera_coords_m.x, (float) 0.0), (float) ENV_LENGTH_g);
        ret_camera_intersection_point.y = min(max( (camera_ray_m * ret_camera_intersection_point.x) + camera_ray_b, (float) 0.0), (float) ENV_HEIGHT_g);

        // set the type
        if (ret_camera_intersection_point.x == 0) {
            ret_camera_intersection_point.int_type = int_left_wall;
        } else if (ret_camera_intersection_point.x == ENV_LENGTH_g) {
            ret_camera_intersection_point.int_type = int_right_wall;
        } else if (ret_camera_intersection_point.y == 0) {
            ret_camera_intersection_point.int_type = int_bottom_wall;
        } else if (ret_camera_intersection_point.y == ENV_HEIGHT_g) {
            ret_camera_intersection_point.int_type = int_top_wall;
        } else {
            ret_camera_intersection_point.int_type = int_free_space;
        }
        ret_camera_intersection_point.is_set = true;
        return;
    }

    // if any intersection points were found. 
    int min_dist_index = distance(distances, min_element(distances, distances + num_robot_arms + num_obstacles));
    if (min_dist_index < num_robot_arms) {
        ret_camera_intersection_point = camera_arm_intersection_pts[min_dist_index];
    } else {
        ret_camera_intersection_point = camera_obstacle_intersection_pts[min_dist_index - num_robot_arms];
    }
    return;
}

void camera::scan_environment(int num_robot_arms, const robot_arm *arms, int num_obstacles, const obstacle *obstacles, cameraIntersectionPoint *ret_camera_intersection_points, int ret_array_size) const{
    /*
    * Scans the environment with camera rays from [-fov/2, fov/2] in degrees with spacing of anlge_resolution_deg. 
    * returns the intersection points by pass by value.
    * args:
    *   - num_robot_arms: the number of robot arms
    *   - robot_arm_relations
    *   - arms: pointer to the list of arms
    *   - num_obstacles: the number of obstacles in the environment
    *   - obstacles: pointer to the list of obstacles. 
    *   - ret_array_size: the length of the array referred to by the pointer given to the function
    * returns: by passing pointer to the array 
    *   - ret_camera_intersection_points: pointer to an array with length ret_array_size that we populate to return the intersection points
    */
    if (ret_array_size != num_scan_angles_m) {
        cerr << "Error in scan environment" << endl;
        throw cameraArrayReturnSizeException();
        return;
    }

    // set everythig to a default value. 
    for (int angle_index = 0; angle_index < num_scan_angles_m; angle_index ++) {
        ret_camera_intersection_points[angle_index].x = 0;
        ret_camera_intersection_points[angle_index].y = 0;
        ret_camera_intersection_points[angle_index].int_type = int_free_space;
        ret_camera_intersection_points[angle_index].is_set = false;
    }
   
    float all_camera_scan_angles[num_scan_angles_m];
    cameraIntersectionPoint temp_intersection_point;
    this->get_camera_scan_angles(all_camera_scan_angles, num_scan_angles_m);

    for (int angle_index = 0; angle_index < num_scan_angles_m; angle_index ++) {
        this->single_scan(all_camera_scan_angles[angle_index], num_robot_arms, arms, num_obstacles, obstacles, temp_intersection_point);\
        ret_camera_intersection_points[angle_index] = temp_intersection_point;

        if (debug_m) {
            cout << "angle: " << all_camera_scan_angles[angle_index] << endl;
            cout << "intersection point: " << temp_intersection_point.x << "," << temp_intersection_point.y << endl << endl;
        }

        if (camera_coords_m.x != 0 && !temp_intersection_point.is_set) {
            cerr << "ERROR: In Scan Environment of camera class - scan points should have been set" <<endl;
            exit(1);
        }
    }
    return;
}

void camera::scan_environment_distance(int num_robot_arms, const robot_arm *arms, int num_obstacles, const obstacle *obstacles, cameraIntersectionDistance *ret_camera_intersection_distances, int ret_array_size) const{
    /*
    * Scans the environment with camera rays from [-fov/2, fov/2] in degrees with spacing of anlge_resolution_deg. 
    * returns the intersection distances by pass by value.
    * args:
    *   - num_robot_arms: the number of robot arms
    *   - arms: pointer to the list of arms
    *   - num_obstacles: the number of obstacles in the environment
    *   - obstacles: pointer to the list of obstacles. 
    *   - ret_array_size: the length of the array referred to by the pointer given to the function
    * returns: by passing pointer to the array 
    *   - ret_camera_intersection_distances: pointer to an array with length ret_array_size that we populate to return the intersection distances
    */ 

    if (ret_array_size != num_scan_angles_m) {
        cerr << "Error in scan environment distance" << endl;
        throw cameraArrayReturnSizeException();
        return;
    }
    // set everythig to a default value. 
    for (int angle_index = 0; angle_index < num_scan_angles_m; angle_index ++) {
        ret_camera_intersection_distances[angle_index].dist = 0;
        ret_camera_intersection_distances[angle_index].int_type = int_free_space;
        ret_camera_intersection_distances[angle_index].is_set = false;
    }
   
    float all_camera_scan_angles[num_scan_angles_m];
    cameraIntersectionPoint temp_intersection_point;
    this->get_camera_scan_angles(all_camera_scan_angles, num_scan_angles_m);

    for (int angle_index = 0; angle_index < num_scan_angles_m; angle_index ++) {
        this->single_scan(all_camera_scan_angles[angle_index], num_robot_arms, arms, num_obstacles, obstacles, temp_intersection_point);
        ret_camera_intersection_distances[angle_index].dist = sqrt(pow(temp_intersection_point.x - camera_coords_m.x, 2) + 
                                                                   pow(temp_intersection_point.y - camera_coords_m.y, 2));
        ret_camera_intersection_distances[angle_index].int_type = temp_intersection_point.int_type;
        ret_camera_intersection_distances[angle_index].is_set = true;
    }
    return;
}

/*
* ********************************************************************************
* END: CAMERA GET functions
* ********************************************************************************
*/

/*
* ********************************************************************************
* BEGIN: CAMERA state mutation and other functions
* ********************************************************************************
*/

void camera::step_to_coord(robotArmCoords new_camera_coords_m) {
    camera_coords_m = new_camera_coords_m;
}

void camera::find_startpt_slope(float scan_angle_deg, environmentCoords &start_pt, float &slope) const{
    /*
    * Find the start point and slope of the line given the scan angle in degrees passing through the (x,y) coordinate
    * of the camera. 
    * args:
    *   - scan_angle_deg: the angle in degrees that describes the line/ camera ray
    */ 
    float delta_x, delta_y;
    delta_x = camera_coords_m.x;
    delta_y = delta_x * tan(scan_angle_deg * THETA_MULTIPLIER);

    start_pt.x = 0;
    start_pt.y = camera_coords_m.y + delta_y;

    if (camera_coords_m.x != 0) {
        slope = (start_pt.y - camera_coords_m.y)/(start_pt.x - camera_coords_m.x);
    } else {
        slope = tan(scan_angle_deg * THETA_MULTIPLIER) - start_pt.y;
    }

    return;
}

/*
* ********************************************************************************
* END: CAMERA state mutation and other functions
* ********************************************************************************
*/

} // end namespace despot