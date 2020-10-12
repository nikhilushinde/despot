#include "render_sim.h"

// the things you are going to use from namespace cv
using cv::Mat;
using cv::Point;
using cv::line;
using cv::circle;
using cv::Scalar;
using cv::namedWindow;
using cv::WINDOW_AUTOSIZE;
using cv::imshow;
using cv::imwrite;
using cv::flip;
using cv::waitKey;

// TODO: NOTE: CHANGE ALL THE PASS BY VALUES TO PASS BY REFERENCE!!!!!!!!!!!!!

namespace despot {
render_sim::render_sim() {
    /*
    * This function serves as the default constructor for the render_sim class 
    * it assigns the values 0 for both the environment_height and the environment_lenght
    * 
    * NOTE: make sure to call:
    *   - set_env_height()
    *   - set_env_length()
    * after this to properly initialize the environment
    */
    window_name_m = RENDER_SIM_WINDOW_NAME;
    debug_m = DEFAULT_DEBUG_FLAG_g;
}

render_sim::render_sim(const render_sim &render_sim_to_copy) {
    /*
    * Copy constructor
    * args:
    *   - render_sim_to_copy: the object to copy
    */ 

    // just keep the same window name as normal initialization - have that be the only allowable window name for this class
    window_name_m = RENDER_SIM_WINDOW_NAME;
    debug_m = render_sim_to_copy.debug_m;
}

void render_sim::render_arm(const robot_arm &arm) const{
    /*
    * This function renders the robot arm in the environment and displays the image
    * in a window with the window_name
    * 
    * args:
    *   - arm: an object of type robot_arm 
    */
   // require a flipped image as in opencv (0,0) is top left - we need to flip vertically to get correct image
    Mat image(ENV_HEIGHT_g, ENV_LENGTH_g, CV_8UC3, Scalar(255, 255, 255));
    Mat flippedImage(ENV_HEIGHT_g, ENV_LENGTH_g, CV_8UC3, Scalar(255, 255, 255));

    environmentCoords start_pt;
    float slope;
    arm.find_startpt_slope(start_pt, slope);

    // points are correctly formatted as (x,y) pairs in opencv
    Point endPt(arm.get_x(), arm.get_y());
    Point startPt((int) start_pt.x, (int) start_pt.y);
    Scalar colorLine(0,0,0); // Green
    int thicknessLine = 2;

    line(image, startPt, endPt, colorLine, thicknessLine);

    // get the flipped image
    int flipDirection = 0; // 0 - means flip vertically
    flip(image, flippedImage, flipDirection);

    // display the window
    namedWindow(window_name_m, WINDOW_AUTOSIZE);
    imshow(window_name_m, flippedImage);
    waitKey(10);
}

void render_sim::render_robot(const robot_arm arms[], int num_robot_arms) const{
    /*
    * This function renders the robot arm in the environment and displays the image
    * in a window with the window_name
    * 
    * args:
    *   - arm: an object of type robot_arm 
    */
    // require a flipped image as in opencv (0,0) is top left - we need to flip vertically to get correct image
    Mat image(ENV_HEIGHT_g, ENV_LENGTH_g, CV_8UC3, Scalar(255, 255, 255));
    Mat flippedImage(ENV_HEIGHT_g, ENV_LENGTH_g, CV_8UC3, Scalar(255, 255, 255));

    environmentCoords start_pt;
    float slope;
    Point startPt;
    Point endPt;
    Scalar colorLine = Scalar(0,0,0);
    int thicknessLine = 2;
    
    for (int i = 0; i < num_robot_arms; i ++) {
        arms[i].find_startpt_slope(start_pt, slope);
        // points are correctly formatted as (x,y) pairs in opencv
        endPt = Point(arms[i].get_x(), arms[i].get_y());
        startPt = Point((int) start_pt.x, (int) start_pt.y);

        line(image, startPt, endPt, colorLine, thicknessLine);
    }
    
    // get the flipped image
    int flipDirection = 0; // 0 - means flip vertically
    flip(image, flippedImage, flipDirection);

    // display the window
    namedWindow(window_name_m, WINDOW_AUTOSIZE);
    imshow(window_name_m, flippedImage);
    waitKey(10);
}

void render_sim::render_environment(const environment &env, int waitKeyTime) const{
    /*
    * This function renders the entire enviornment which includes all the robot arms and all the obstacles
    * in a window with the window_name
    * 
    * args:
    *   - env: the environment object to render
    *   - waitKeyTime: Default = 10: the default time to wait between renders of the environment
    */
    // require a flipped image as in opencv (0,0) is top left - we need to flip vertically to get correct image
    Mat image(ENV_HEIGHT_g, ENV_LENGTH_g, CV_8UC3, Scalar(255, 255, 255));
    Mat flippedImage(ENV_HEIGHT_g, ENV_LENGTH_g, CV_8UC3, Scalar(255, 255, 255));

    // get robot paramters
    int num_robot_arms = env.get_num_robot_arms();
    int num_obstacles = env.get_num_obstacles();
    environmentCoords ret_start_pts[num_robot_arms];
    environmentCoords ret_end_pts[num_robot_arms]; 
    env.get_all_robot_start_end_pts(ret_start_pts, ret_end_pts, num_robot_arms);
    // get obstacle parameters
    int obstacle_radius = env.get_obstacle_radius();
    environmentCoords ret_obstacle_centers[num_obstacles];
    env.get_current_obstacle_centers(ret_obstacle_centers, num_obstacles);

    // render the robot 
    Point startPt, endPt;
    Scalar colorLine = Scalar(0,0,0);
    int thicknessLine = 2;
    for (int i = 0; i < num_robot_arms; i++) {
        startPt = Point((int) ret_start_pts[i].x, (int) ret_start_pts[i].y);
        endPt = Point((int) ret_end_pts[i].x, (int) ret_end_pts[i].y);
        line(image, startPt, endPt, colorLine, thicknessLine);
    }

    // render the obstacles
    Scalar obstacleFillColor = Scalar(0, 0, 255);
    Scalar obstacleCenterColor = Scalar(255, 0, 0);
    Point obstacleCenterPoint;
    int centerRadius = 3;
    int fill_thickness = -1;

    for (int i = 0; i < num_obstacles; i++) {
        if (ret_obstacle_centers[i].y > env.obstacle_center_y_m[i]) {
            obstacleCenterPoint = Point((int) ret_obstacle_centers[i].x, (int) ceil(ret_obstacle_centers[i].y));
        } else {
            obstacleCenterPoint = Point((int) ret_obstacle_centers[i].x, (int) ret_obstacle_centers[i].y);
        }
        circle(image, obstacleCenterPoint, env.get_obstacle_radius(), obstacleFillColor, fill_thickness);
        circle(image, obstacleCenterPoint, centerRadius, obstacleCenterColor, fill_thickness);
    }
    
    // render the goal coordinate and goal region
    environmentCoords goal_coord = env.get_goal_coord();
    Point goalPoint = Point((int) goal_coord.x, (int) goal_coord.y);
    int goal_radius = env.get_goal_radius();
    Scalar goal_color = Scalar(0, 255, 0);
    int region_line_thickness = 3;
    // draw filled circle at center and a not filled one aroudn the whole region
    circle(image, goalPoint, centerRadius, goal_color, fill_thickness);
    circle(image, goalPoint, goal_radius, goal_color, region_line_thickness);

    if (debug_m) {
        // display all the camera points
        std::cout << "the number of scan angles in the first render is: " << env.get_cam_num_scan_angles() << std::endl;
    }

    cameraIntersectionPoint intersection_points[env.get_cam_num_scan_angles()];
    env.observe_points(intersection_points, env.get_cam_num_scan_angles());
    Point scan_point;
    Scalar scan_color;
    int scan_radius = 3;

    int circle_count = 0;
    for (int i = 0; i < env.get_cam_num_scan_angles(); i++) {
        if (intersection_points[i].is_set) {
            circle_count ++;
            if ((intersection_points[i].int_type == int_bottom_wall) ||
            (intersection_points[i].int_type == int_top_wall) ||
            (intersection_points[i].int_type == int_left_wall) ||
            (intersection_points[i].int_type == int_right_wall) ) {
                // wall color
                scan_color = Scalar(0, 255, 125);
            } else if (intersection_points[i].int_type == int_obstacle_intersect) {
                // obstacle intersect color
                scan_color = Scalar(0, 0, 0);
            } else if (intersection_points[i].int_type == int_arm_intersect) {
                // arm intersect color
                scan_color = Scalar(0, 255, 0);
            } else if (intersection_points[i].int_type == int_free_space) {
                scan_color = Scalar(255, 125, 0);
            }
            scan_point = Point((int) intersection_points[i].x, (int) intersection_points[i].y);
            circle(image, scan_point, scan_radius, scan_color, fill_thickness);

            if (debug_m) {
                std::cout << "Drawing scan circle at:" << intersection_points[i].x << "," << intersection_points[i].y << std::endl;
            }

        }
    }

    if (debug_m) {
        std::cout << "TOTAL SCAN CIRCLES: " << circle_count << std::endl;
    }

    // get the flipped image
    int flipDirection = 0; // 0 - means flip vertically
    flip(image, flippedImage, flipDirection);

    // display the window
    namedWindow(window_name_m, WINDOW_AUTOSIZE);
    imshow(window_name_m, flippedImage);
    waitKey(waitKeyTime);

    std::string save_file_name; 
    save_file_name = results_folder_name_g;
    save_file_name += std::to_string(saved_image_number_g);
    save_file_name += ".jpg";
    saved_image_number_g ++; // increment the global saved image number 
    imwrite(save_file_name, flippedImage);
}

} // end namespace despot