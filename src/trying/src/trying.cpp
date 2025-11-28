#include <template.h>
#include <collision_avoidance.h>

// 全局变量定义
int mission_num = 0;
float if_debug = 0;
float err_max = 0.2;
float target_x_mission_2 = 8;                                                 //期望位置_x
float target_y_mission_2 = -2;                                                 //期望位置_y
float target_yaw = 0;

void print_param()
{
  std::cout << "=== 控制参数 ===" << std::endl;
  std::cout << "err_max: " << err_max << std::endl;
  std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
  std::cout << "if_debug: " << if_debug << std::endl;
  if(if_debug == 1) std::cout << "自动offboard" << std::endl;
  else std::cout << "遥控器offboard" << std::endl;
}


int main(int argc, char **argv)
{
  // 防止中文输出乱码
  setlocale(LC_ALL, "");

  // 初始化ROS节点
  ros::init(argc, argv, "trying");
  ros::NodeHandle nh;

  // 订阅mavros相关话题
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);
  ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/laser/scan", 1000, lidar_cb);//【订阅】Lidar数据
  ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);//【订阅】无人机当前位置 坐标系 NED系

  // 发布无人机多维控制话题
  ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

  // 创建服务客户端
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

  // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
  ros::Rate rate(20);

  // 参数读取

  nh.param<float>("err_max", err_max, 0);
  nh.param<float>("if_debug", if_debug, 0);

  nh.param<float>("target_x_mission", target_x_mission_2,8);
  nh.param<float>("target_y_mission", target_y_mission_2,-2);
  nh.param<float>("target_yaw", target_yaw,0);

  nh.param<float>("R_outside", R_outside, 2);
  nh.param<float>("R_inside", R_inside, 1);

  nh.param<float>("p_xy", p_xy, 0.5);

  nh.param<float>("vel_track_max", vel_track_max, 0.5);

  nh.param<float>("p_R", p_R, 0.0);
  nh.param<float>("p_r", p_r, 0.0);

  nh.param<float>("vel_collision_max", vel_collision_max, 0.0);
  nh.param<float>("vel_sp_max", vel_sp_max, 0.0);

  nh.param<int>("range_min", range_min, 0.0);
  nh.param<int>("range_max", range_max, 0.0);

  print_param();

  
  int choice = 0;
  std::cout << "1 to go on , else to quit" << std::endl;
  std::cin >> choice;
  if (choice != 1) return 0;
  ros::spinOnce();
  rate.sleep();
  
  // 等待连接到飞控
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }
  //设置无人机的期望位置
 
  setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
  setpoint_raw.coordinate_frame = 1;
  setpoint_raw.position.x = 0;
  setpoint_raw.position.y = 0;
  setpoint_raw.position.z = ALTITUDE;
  setpoint_raw.yaw = 0;

  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }
  std::cout<<"ok"<<std::endl;

  // 定义客户端变量，设置为offboard模式
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  // 定义客户端变量，请求无人机解锁
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  // 记录当前时间，并赋值给变量last_request
  ros::Time last_request = ros::Time::now();

  while (ros::ok())
  {
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
    {
      if(if_debug == 1)
      {
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
          ROS_INFO("Offboard enabled");
        }
      }
      else
      {
        ROS_INFO("Waiting for OFFBOARD mode");
      }
      last_request = ros::Time::now();
    }
    else
    {
      if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }
    // 当无人机到达起飞点高度后，悬停3秒后进入任务模式，提高视觉效果
    if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.2)
    {
      if (ros::Time::now() - last_request > ros::Duration(1.0))
      {
        mission_num = 1;
 	      last_request = ros::Time::now();
        break;
      }
    }

    mission_pos_cruise(0, 0, ALTITUDE, 0, err_max); 
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }
  
  int mission_flag = 0;
  //takeoff
  while(ros::ok()){// mission1: 起飞
    ROS_WARN("mission_num = 1");
    if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
    {
      mission_num = 2;
      mission_flag = 1;
      last_request = ros::Time::now();
      break;
    }
    else if(ros::Time::now() - last_request >= ros::Duration(10.0))
    {
      mission_num = 2;
      mission_flag = 1;
      last_request = ros::Time::now();
      break;
    }
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }

  while (ros::ok())
  {
    ROS_WARN("mission_num = %d", mission_num);
    switch (mission_num)
    {
      // //世界系前进
      // case 2:
      //   if (mission_pos_cruise(1.0, 0.0, ALTITUDE, 0.0 , err_max))
      //   {
      //     mission_num = 3;
      //     last_request = ros::Time::now();
      //   }
      //   break;
      case 2:
        printf_param();
        vel_track[0]= 0;
        vel_track[1]= 0;

        vel_collision[0]= 0;
        vel_collision[1]= 0;

        vel_sp_body[0]= 0;
        vel_sp_body[1]= 0;

        vel_sp_ENU[0]= 0;
        vel_sp_ENU[1]= 0;
        

        while (ros::ok())
        { 
          ROS_INFO("now (%.2f,%.2f,%.2f,%.2f) to ( %.2f, %.2f, %.2f, %.2f)", local_pos.pose.pose.position.x ,local_pos.pose.pose.position.y, local_pos.pose.pose.position.z, target_yaw * 180.0 / M_PI, target_x_mission_2 + init_position_x_take_off, target_y_mission_2 + init_position_y_take_off, ALTITUDE + init_position_z_take_off, target_yaw * 180.0 / M_PI );
          //回调一次 更新传感器状态
          //1. 更新雷达点云数据，存储在Laser中,并计算四向最小距离
          ros::spinOnce();
          collision_avoidance(target_x_mission_2,target_y_mission_2);
          
          setpoint_raw.type_mask = 1 + 2 /*+ 4 + 8 + 16 */+ 32 +64 + 128 + 256 + 512 /*+ 1024 + 2048*/; // xy 速度控制模式 z 位置控制模式  //
          setpoint_raw.velocity.x =  vel_sp_ENU[0];
          setpoint_raw.velocity.y =  vel_sp_ENU[1];  //ENU frame
          setpoint_raw.position.z =  ALTITUDE;
          setpoint_raw.yaw = 0 ;
          mavros_setpoint_pos_pub.publish(setpoint_raw);

          // float abs_distance;
          // abs_distance = sqrt((pos_drone.pose.position.x - target_x) * (pos_drone.pose.position.x - target_x) + (pos_drone.pose.position.y - target_y) * (pos_drone.pose.position.y - target_y));
          // if(abs_distance < 0.3 || flag_land == 1)
          // {
          //     Command_now.command = 3;     //Land
          //     flag_land = 1;
          // }
          // if(flag_land == 1) Command_now.command = Land;
          // command_pub.publish(Command_now);
          // //打印
          printf();
          rate.sleep();
          if (fabs(local_pos.pose.pose.position.x - target_x_mission_2 - init_position_x_take_off) < err_max && fabs(local_pos.pose.pose.position.y - target_y_mission_2 - init_position_y_take_off) < err_max && fabs(local_pos.pose.pose.position.z - ALTITUDE - init_position_z_take_off) < err_max && fabs(yaw - target_yaw) < 0.1)
          {
            ROS_INFO("到达目标点，巡航点任务完成");
            mission_num = 100;
            break;
          }
        }
        break;
    }
    if(mission_num == 100) break;
  }

  //land
  while(ros::ok()){//降落
    ROS_WARN("mission_num = -1");
    if(precision_land())
    {
      exit(0); // 任务结束
      last_request = ros::Time::now();
    }
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
    
  }
  return 0;
}


