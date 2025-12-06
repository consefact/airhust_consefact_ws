#include <string>
#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/time.h>
// #include <template.h>


int IMAGE_WIDTH  = 320;
int IMAGE_HEIGHT  = 240;
double k_xy = 0.0005;
float error_max_image = 5;
bool bbox_move_flag = false;
std::string bbox_class_name;
ros::Time last_bbox_detection_time; // 新增：记录最后检测时间
const double DETECTION_TIMEOUT = 0.5; // 新增：超时阈值（秒），如果0.5秒没新数据就认为目标丢失

void bbox_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
bool move_to_center(double error_x, double error_y, float z);
bool objection_detect();

darknet_ros_msgs::BoundingBoxes current_bbox_state;

void bbox_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
    current_bbox_state = *msg;
    last_bbox_detection_time = ros::Time::now();
}

double error_x = 0;
double error_y = 0;
bool objection_detect(){
    // 检查1: 检测数据是否已过期？
    if ((ros::Time::now() - last_bbox_detection_time).toSec() > DETECTION_TIMEOUT) {
        // ROS_DEBUG_THROTTLE(1.0, “检测数据已超时，无最新图像。”);
        return false; // 数据太旧，认为目标已丢失
    }

    // 检查2: 在有效数据中寻找目标
    for(const auto& bbox : current_bbox_state.bounding_boxes){
        if(bbox.Class == "qrcode" && bbox.probability > 0.25){
            double center_x = (bbox.xmin + bbox.xmax) / 2.0;
            double center_y = (bbox.ymin + bbox.ymax) / 2.0;

            error_x = IMAGE_WIDTH / 2.0 - center_x;
            error_y = IMAGE_HEIGHT / 2.0 - center_y;
            bbox_class_name = bbox.Class;
            ROS_INFO("%s detected, \nerror_x: %.2lf  error_y: %.2lf", bbox.Class.c_str(), error_x, error_y);
            return true;
        }
    }
    return false;
}

bool move_to_center(double error_x, double error_y, float z){
    float vel_x = 0, vel_y = 0;
    vel_x = k_xy * error_x;
    vel_y = k_xy * error_y;

    setpoint_raw.type_mask = 1 + 2 +/* 4 + 8 + 16 +*/ 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.velocity.x = vel_y;
    setpoint_raw.velocity.y = vel_x;
    setpoint_raw.yaw = 0;
    // setpoint_raw.position.z = z + init_position_z_take_off;
    if(fabs(error_x) < error_max_image && fabs(error_y) < error_max_image){
        ROS_INFO("have moved to center");
        return true;
    }
    return false;
}