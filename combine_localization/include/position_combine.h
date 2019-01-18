#include <iostream>
#include <sstream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <ros/package.h>
#include <QString>
#include <QSettings>
#include <vector>
#include <std_msgs/Int64.h>
#include "mrpt_msgs/ObservationRangeBeacon.h"
#include <sensor_msgs/Imu.h>
#include<math.h>


using namespace std;

#define IF_SELF_LOCALIZATION_TYPE 0 //0:关闭；1: 一直gnss定位；2: 一直uwb定位；3: 一直2维激光定位;
#define RIGION_DECIDE_DISTANCE  1.0 //"m"
#define RIGION_DECIDE_RADIAN   1.57 // 90
extern "C" {
typedef struct {
     double x,y,yaw;
     bool flag;
     ros::Time time;
	 int map_num; //地图编号
	 std::string frame_id;
} position;



};
struct TransformListenerWrapper : public tf::TransformListener
{
  inline tf2_ros::Buffer &getBuffer() {return tf2_buffer_;}
};

class PositionCombine
{
  public:
    /*函数声明*/
    PositionCombine();
    ~PositionCombine();
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void gpsPositionCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void uwbPositionCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void laserPositionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void gpsRawCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void uwbSignalCallback(const mrpt_msgs::ObservationRangeBeacon::ConstPtr& msg);
    void pross();
    bool tfPub(position data);
    bool tf2Pub(position data);
    void init();
    void initPosePub(position msg);
	void getParam(std::string &msg);
	void initPoseFilter();
	void chargeSignalCallback(const std_msgs::Int64::ConstPtr& msg);
	void mapNumCallback(const std_msgs::Int64::ConstPtr& msg);
	void localizationTypeCallback(const std_msgs::Int64::ConstPtr& msg);
	void uwbRawPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
	void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	int localizationRegionDecide();
	bool poseDecide(position robot_pose ,position start_pose);
	position gnssToLaserTranslate(position map_pose,int type);
	position LaserTognssTranslate(position laser_pose, int num);

  private:
    ros::NodeHandle position_combine_nh;
    ros::Subscriber uwb_position_sub,gps_position_sub,laser_position_sub,odom_sub,gps_raw_sub,uwb_signal_sub;
	ros::Subscriber charge_signal_sub,uwb_raw_pose_sub,imu_sub,initpose_sub;
    ros::Publisher combine_pose_pub,initpose_pub,localization_type_pub,stop_pub;
	ros::Subscriber map_num_sub,localization_type_sub;
	ros::Publisher map_num_pub;
    tf::TransformBroadcaster last_tf_transform;
    std::string base_frame_id_;
    std::string odom_frame_id_;
    std::string global_frame_id_;
	std::string laser_map_frame_id;
    tf::Transform latest_tf_,latest_tf_map_laser;

    tf::TransformBroadcaster* tfb_;
    TransformListenerWrapper* tf_;
	tf::TransformBroadcaster map_to_laser_transform;
	tf::TransformBroadcaster* tf2b_;
    TransformListenerWrapper* tf2_;
    tf::TransformListener map_listener;
    tf::StampedTransform map_transform;
    position laser_position,uwb_position,gnss_position,charge_position;
    position pose,uwb_raw_pose,init_pose,combine_pose;
	position map_to_laser_pose;
    nav_msgs::Odometry odom_position;
    bool first_flag,imu_flag,region_change_flag;
	vector<position> laser_start_pose;
	vector<position> uwb_start_pose;
	vector<position> gnss_start_pose;
vector<position> map_laser_position;
vector<position> map_gnss_position;
	int localization_type; //0:不处理；1:gnss定位；2:uwb定位；3:2维激光定位;
	bool gps_signal_flag,uwb_signal_flag,charge_signal_flag;
	ros::Time gps_signal_time,uwb_signal_time,charge_signal_time,region_change_time;
	double imu_yaw;
	int map_num;
	bool init_one_time_flag;//切换定位模式时，初始化一次
	int tmp_localization_type;
	bool navigate_stop;


 };
