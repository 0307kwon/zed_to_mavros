#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <string.h>



static tf::Vector3 position_orig, position_body;

static tf::Quaternion quat_cam, quat_cam_to_body_x, quat_cam_to_body_y, quat_cam_to_body_z, quat_rot_z, quat_body;

void camera_sub_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

int main(int argc, char** argv){
  ros::init(argc, argv, "zed_to_mavros");

  ros::NodeHandle node;

  // 카메라의 x,y,z정보를 mavros의 좌표계로 변환해서 publish
  ros::Publisher camera_pose_publisher = node.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
  geometry_msgs::PoseStamped msg_body_pose;
  //

  //zed에서 카메라 정보를 읽어옵니다.
  ros::Subscriber camera_pose_subscribe = node.subscribe<geometry_msgs::PoseStamped>("/zedm/zed_node/pose",10,camera_sub_callback);
  //


  double output_rate = 30, roll_cam = 0, pitch_cam = 0, yaw_cam = 1.5707963, gamma_world = -1.5707963;

  // Read parameters from launch file, including: target_frame_id, source_frame_id, output_rate
  {
    // The frame in which we find the transform into, the original "world" frame

    // The rate at which we wish to publish final pose data
    if(node.getParam("output_rate", output_rate))
    {
      ROS_INFO("Get output_rate parameter: %f", output_rate);
    }
    else
    {
      ROS_WARN("Using default output_rate: %f", output_rate);
    }

    // The rotation around z axis between original world frame and target world frame, assuming the z axis needs not to be changed
    // In this case, target world frame has y forward, x to the right and z upwards (ENU as ROS dictates)
    if(node.getParam("gamma_world", gamma_world))
    {
      ROS_INFO("Get gamma_world parameter: %f", gamma_world);
    }
    else
    {
      ROS_WARN("Using default gamma_world: %f", gamma_world);
    }

    // The roll angle around camera's own axis to align with body frame
    if(node.getParam("roll_cam", roll_cam))
    {
      ROS_INFO("Get roll_cam parameter: %f", roll_cam);
    }
    else
    {
      ROS_WARN("Using default roll_cam: %f", roll_cam);
    }

    // The pitch angle around camera's own axis to align with body frame
    if(node.getParam("pitch_cam", pitch_cam))
    {
      ROS_INFO("Get pitch_cam parameter: %f", pitch_cam);
    }
    else
    {
      ROS_WARN("Using default pitch_cam: %f", pitch_cam);
    }

    // The yaw angle around camera's own axis to align with body frame
    if(node.getParam("yaw_cam", yaw_cam))
    {
      ROS_INFO("Get yaw_cam parameter: %f", yaw_cam);
    }
    else
    {
      ROS_WARN("Using default yaw_cam: %f", yaw_cam);
    }
  }


  ros::Rate rate(output_rate);


  while(node.ok()){

      // 1) Rotation from original world frame to world frame with y forward.
      // See the full rotation matrix at https://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
      //position_orig = transform.getOrigin();

      position_body.setX( cos(gamma_world) * position_orig.getX() + sin(gamma_world) * position_orig.getY());
      position_body.setY(-sin(gamma_world) * position_orig.getX() + cos(gamma_world) * position_orig.getY());
      position_body.setZ(position_orig.getZ());

      // 2) Rotation from camera to body frame.
      //quat_cam = transform.getRotation();

      quat_cam_to_body_x = tf::createQuaternionFromRPY(roll_cam, 0, 0);
      quat_cam_to_body_y = tf::createQuaternionFromRPY(0, pitch_cam, 0);
      quat_cam_to_body_z = tf::createQuaternionFromRPY(0, 0, yaw_cam);

      // 3) Rotate body frame 90 degree (align body x with world y at launch)
      quat_rot_z = tf::createQuaternionFromRPY(0, 0, -gamma_world);

      quat_body = quat_rot_z * quat_cam * quat_cam_to_body_x * quat_cam_to_body_y * quat_cam_to_body_z;
      quat_body.normalize();

      // Create PoseStamped message to be sent
      msg_body_pose.header.stamp = ros::Time::now();
      msg_body_pose.header.frame_id = "map";
      msg_body_pose.pose.position.x = position_body.getX();
      msg_body_pose.pose.position.y = position_body.getY();
      msg_body_pose.pose.position.z = position_body.getZ();
      msg_body_pose.pose.orientation.x = quat_body.getX();
      msg_body_pose.pose.orientation.y = quat_body.getY();
      msg_body_pose.pose.orientation.z = quat_body.getZ();
      msg_body_pose.pose.orientation.w = quat_body.getW();

      // Publish pose of body frame in world frame
      camera_pose_publisher.publish(msg_body_pose);
      ros::spinOnce();
      rate.sleep();
  }



  return 0;
}

void camera_sub_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  position_orig = tf::Vector3(msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z);
  quat_cam = tf::Quaternion(msg->pose.orientation.x,
  msg->pose.orientation.y,
  msg->pose.orientation.z,
  msg->pose.orientation.w);
}
