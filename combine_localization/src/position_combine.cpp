
#include<position_combine.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "position_combine");
    ROS_INFO("Position combine node start!");
	
    PositionCombine position_combine;
	
	std::string config_name = ros::package::getPath("position_combine")+"/config/position_combine.ini";
	position_combine.getParam(config_name);
	
    ros::Rate rate(10);
    while(ros::ok())
    {   

    position_combine.pross();
	ros::spinOnce();
	rate.sleep();
    }
    return 0;
}
   
PositionCombine::PositionCombine()
{	
	//订阅相应消息
    odom_sub = position_combine_nh.subscribe<nav_msgs::Odometry>("/odom",1,boost::bind(&PositionCombine::odomCallback, this, _1));
    gps_position_sub = position_combine_nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1, boost::bind(&PositionCombine::gpsPositionCallback, this, _1));
    uwb_position_sub = position_combine_nh.subscribe<nav_msgs::Odometry>("/robot/mrpt_uwb_pose", 1, boost::bind(&PositionCombine::uwbPositionCallback, this, _1));
    laser_position_sub = position_combine_nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, boost::bind(&PositionCombine::laserPositionCallback, this, _1));
  	gps_raw_sub=position_combine_nh.subscribe<nav_msgs::Odometry>("/odometry/gps", 1, boost::bind(&PositionCombine::gpsRawCallback, this, _1));
	uwb_signal_sub=position_combine_nh.subscribe<mrpt_msgs::ObservationRangeBeacon>("/beacon", 1, boost::bind(&PositionCombine::uwbSignalCallback, this, _1));
	charge_signal_sub=position_combine_nh.subscribe<std_msgs::Int64>("/robot/charge_flag", 1, boost::bind(&PositionCombine::chargeSignalCallback, this, _1));
	uwb_raw_pose_sub=position_combine_nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/uwb/pose", 1, boost::bind(&PositionCombine::uwbRawPoseCallback, this, _1));
	imu_sub=position_combine_nh.subscribe<sensor_msgs::Imu>("/imu/data", 1, boost::bind(&PositionCombine::imuCallback, this, _1));
	initpose_sub=position_combine_nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, boost::bind(&PositionCombine::initPoseCallback, this, _1));
	map_num_sub=position_combine_nh.subscribe<std_msgs::Int64>("/robot/map_num_manual", 1, boost::bind(&PositionCombine::mapNumCallback, this, _1));
	localization_type_sub=position_combine_nh.subscribe<std_msgs::Int64>("/robot/localization_type_manual", 1, boost::bind(&PositionCombine::localizationTypeCallback, this, _1));

	map_num_pub=position_combine_nh.advertise<std_msgs::Int64>("/robot/map_num", 1,true);
	combine_pose_pub=position_combine_nh.advertise<nav_msgs::Odometry>("/robot/combine_pose", 2);
    initpose_pub=position_combine_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot/initialpose", 1,true);
	localization_type_pub=position_combine_nh.advertise<std_msgs::Int64>("/robot/localization_type", 1);
	stop_pub=position_combine_nh.advertise<std_msgs::Int64>("/robot/localization_to_stop", 2);//0:忽略 1:关闭
		
    init();
}

PositionCombine::~PositionCombine()
{
   ROS_INFO("Slam Test Stop!");
   delete tfb_;
   delete tf_;
   laser_start_pose.clear();
   uwb_start_pose.clear();
   gnss_start_pose.clear();
   map_laser_position.clear();
   map_gnss_position.clear();
   ros::shutdown();
   exit(0);
}

void PositionCombine::init()
{
    tfb_ = new tf::TransformBroadcaster();
    tf_ = new TransformListenerWrapper();
	tf2b_ = new tf::TransformBroadcaster();
    tf2_ = new TransformListenerWrapper();
    base_frame_id_=std::string("base_footprint");
    odom_frame_id_=std::string("odom");
    global_frame_id_=std::string("map");
	laser_map_frame_id=std::string("map_laser");

    laser_position.flag=false;
    uwb_position.flag=false;
    gnss_position.flag=false;
	uwb_raw_pose.flag=false;
    first_flag=true;
    pose.x=0;
    pose.y=0;
    pose.yaw=0;
	pose.flag=false;
	init_pose.flag=false;
	localization_type=0;
	gps_signal_flag=false;
	uwb_signal_flag=false;
	charge_signal_flag=false;
	imu_flag=false;
	region_change_flag=false;
	map_num=0;
	init_one_time_flag=false;
	tmp_localization_type=0;
	map_to_laser_pose.x=0;
	map_to_laser_pose.y=0;
	map_to_laser_pose.yaw=0;
	map_to_laser_pose.flag=false;
	navigate_stop=false;

}

void PositionCombine::initPosePub(position msg)
{
    geometry_msgs::PoseWithCovarianceStamped temp_pose;
	
    temp_pose.header.stamp=ros::Time::now();
    temp_pose.header.frame_id=msg.frame_id;
    temp_pose.pose.pose.position.x=msg.x;
    temp_pose.pose.pose.position.y=msg.y;
    temp_pose.pose.pose.orientation=tf::createQuaternionMsgFromYaw(msg.yaw);
    ROS_INFO("Position_combine:: init pose=(%f,%f,%f)",msg.x,msg.y,msg.yaw*180/3.1415926);
	//if(msg.frame_id==global_frame_id_)
	//combine_pose=msg;
    initpose_pub.publish(temp_pose);
}


/**
*@ brief : 读取配置文件参数 
*@ parameter: 配置文件路径
*@ return 
*/

void PositionCombine::getParam(std::string &msg)
{
	QSettings settings(msg.c_str(), QSettings::IniFormat);
	
	if(!laser_start_pose.empty())  laser_start_pose.clear();
	
	if(!uwb_start_pose.empty())  uwb_start_pose.clear();
	
	if(!gnss_start_pose.empty())  gnss_start_pose.clear();
	
	int n= 0;
	n=settings.value("Laser/pose_num").toInt();
	ROS_INFO("Laser/pose_num=%d",n);
	for(int i=0;i<n;i++){
	     QString qStr=QString::number(i+1, 10);
	     std::string num_str=qStr.toStdString();
	     std::string x_str="Laser/pose_x_"+num_str;
	     std::string y_str="Laser/pose_y_"+num_str;
		 std::string yaw_str="Laser/pose_yaw_"+num_str;
		 std::string map_num_str="Laser/pose_map_num_"+num_str;
		 
		 position temp_data;
		 temp_data.x=settings.value(x_str.c_str()).toDouble();
		 temp_data.y=settings.value(y_str.c_str()).toDouble();
		 temp_data.yaw=settings.value(yaw_str.c_str()).toDouble();
		 temp_data.map_num=settings.value(map_num_str.c_str()).toInt();
		 laser_start_pose.push_back(temp_data);
		 ROS_INFO("laser_pose %d=(map %d(%f,%f,%f))",i+1,laser_start_pose[i].map_num,laser_start_pose[i].x,laser_start_pose[i].y,laser_start_pose[i].yaw);
	}

	n=settings.value("Uwb/pose_num").toInt();
	ROS_INFO("Uwb/pose_num=%d",n);
	for(int i=0;i<n;i++){
	     QString qStr=QString::number(i+1, 10);
	     std::string num_str=qStr.toStdString();
	     std::string x_str="Uwb/pose_x_"+num_str;
	     std::string y_str="Uwb/pose_y_"+num_str;
		 std::string yaw_str="Uwb/pose_yaw_"+num_str;
		 
		 position temp_data;
		 temp_data.x=settings.value(x_str.c_str()).toDouble();
		 temp_data.y=settings.value(y_str.c_str()).toDouble();
		 temp_data.yaw=settings.value(yaw_str.c_str()).toDouble();
		 uwb_start_pose.push_back(temp_data);
		 ROS_INFO("Uwb_pose %d=(%f,%f,%f)",i+1,uwb_start_pose[i].x,uwb_start_pose[i].y,uwb_start_pose[i].yaw);
	}

	n=settings.value("Gnss/pose_num").toInt();
	ROS_INFO("Gnss/pose_num=%d",n);
	for(int i=0;i<n;i++){
	     QString qStr=QString::number(i+1, 10);
	     std::string num_str=qStr.toStdString();
	     std::string x_str="Gnss/pose_x_"+num_str;
	     std::string y_str="Gnss/pose_y_"+num_str;
		 std::string yaw_str="Gnss/pose_yaw_"+num_str;
		 
		 position temp_data;
		 temp_data.x=settings.value(x_str.c_str()).toDouble();
		 temp_data.y=settings.value(y_str.c_str()).toDouble();
		 temp_data.yaw=settings.value(yaw_str.c_str()).toDouble();
		 gnss_start_pose.push_back(temp_data);
		 ROS_INFO("Gnss_pose %d=(%f,%f,%f)",i+1,gnss_start_pose[i].x,gnss_start_pose[i].y,gnss_start_pose[i].yaw);
	}
	
	n=settings.value("Charge_Pose/pose_num").toInt();
	ROS_INFO("Charge_Pose/pose_num=%d",n);
	for(int i=0;i<n;i++){
	     QString qStr=QString::number(i+1, 10);
	     std::string num_str=qStr.toStdString();
	     std::string x_str="Charge_Pose/pose_x_"+num_str;
	     std::string y_str="Charge_Pose/pose_y_"+num_str;
		 std::string yaw_str="Charge_Pose/pose_yaw_"+num_str;
		 
		 
		 charge_position.x=settings.value(x_str.c_str()).toDouble();
		 charge_position.y=settings.value(y_str.c_str()).toDouble();
		 charge_position.yaw=settings.value(yaw_str.c_str()).toDouble();
		 charge_position.frame_id=laser_map_frame_id;
		
		 ROS_INFO("Charge_Pose_pose %d=(%f,%f,%f)",i+1,charge_position.x,charge_position.y,charge_position.yaw);
	}	

	n=settings.value("Map_Laser/map_num").toInt();
	ROS_INFO("Map_Laser/map_num=%d",n);
	for(int i=0;i<n;i++){
	     QString qStr=QString::number(i, 10);
	     std::string num_str=qStr.toStdString();
	     std::string x_str="Map_Laser/map_laser_x_"+num_str;
	     std::string y_str="Map_Laser/map_laser_y_"+num_str;
		 std::string yaw_str="Map_Laser/map_laser_yaw_"+num_str;
		 
		 position temp_data;
		 temp_data.x=settings.value(x_str.c_str()).toDouble();
		 temp_data.y=settings.value(y_str.c_str()).toDouble();
		 temp_data.yaw=settings.value(yaw_str.c_str()).toDouble();
		 map_laser_position.push_back(temp_data);

		 x_str="Map_Laser/map_gnss_x_"+num_str;
		 y_str="Map_Laser/map_gnss_y_"+num_str;
		 yaw_str="Map_Laser/map_gnss_yaw_"+num_str;
		 
		 temp_data.x=settings.value(x_str.c_str()).toDouble();
		 temp_data.y=settings.value(y_str.c_str()).toDouble();
		 temp_data.yaw=settings.value(yaw_str.c_str()).toDouble();
		 map_gnss_position.push_back(temp_data);
		
		 ROS_INFO("map_laser_pose %d=(laser(%f,%f,%f),gnss(%f,%f,%f))",i+1,map_laser_position[i].x,map_laser_position[i].y,map_laser_position[i].yaw,map_gnss_position[i].x,map_gnss_position[i].y,map_gnss_position[i].yaw);
	}	
	
}


/**
*@ brief : 初始,选择定位模式
*@ parameter: 
*@ return 
*/

void PositionCombine::initPoseFilter()
{	
	if(0==IF_SELF_LOCALIZATION_TYPE){
		
		if(charge_signal_flag){
			localization_type=3;
			pose=charge_position;
			pose.flag=true;
		}
		else if(gps_signal_flag)
			localization_type=1;
		else if(uwb_signal_flag){
			localization_type=2;	
		}
		else if(init_pose.flag){
			localization_type=3; //开机时收到初始化坐标，默认为激光定位模式
			pose=init_pose;
			init_pose.flag=false;

		}
		else localization_type=0;

	}
	else 
		localization_type=IF_SELF_LOCALIZATION_TYPE;
	//std_msgs::Int64 tmp_type;
	//tmp_type.data=localization_type;
	//localization_type_pub.publish(tmp_type);
	
}


/**
*@ brief : 将gnss坐标系/map下的坐标点转化为激光坐标系下/amcl_map的坐标点
*@ parameter: gnss坐标系的位置
*@ return :激光坐标系下的位置
*/


position PositionCombine::gnssToLaserTranslate(position gnss_pose, int num)
{
		int i=num;
		position tmp_laser,tmp_map_laser;
		double theta=map_gnss_position[i].yaw;
		
		if(i<0) {
			ROS_INFO("Position_combine:: map_num error,can't translate to laser map");
			
			return tmp_map_laser;
		}
		
		
		//转化到机器人坐标系
	 	tmp_laser.x = (gnss_pose.x - map_gnss_position[i].x) * cos(theta) + (gnss_pose.y -  map_gnss_position[i].y) * sin(theta);
					
		tmp_laser.y = (gnss_pose.y - map_gnss_position[i].y) * cos(theta) - (gnss_pose.x -map_gnss_position[i].x) * sin(theta);

		tmp_laser.yaw=gnss_pose.yaw-map_gnss_position[i].yaw;
		
		if(tmp_laser.yaw>M_PI) tmp_laser.yaw-=2*M_PI;
		else if(tmp_laser.yaw<-M_PI) tmp_laser.yaw+=2*M_PI;

		//转化到map_laser坐标系
		theta=map_laser_position[i].yaw;
		tmp_map_laser.x = map_laser_position[i].x + tmp_laser.x * cos(theta) -tmp_laser.y * sin(theta);
    		tmp_map_laser.y = map_laser_position[i].y + tmp_laser.x * sin(theta) + tmp_laser.y * cos(theta);
		tmp_map_laser.yaw=tmp_laser.yaw+map_laser_position[i].yaw;
		
		if(tmp_map_laser.yaw>M_PI) tmp_map_laser.yaw-=2*M_PI;
		else if(tmp_map_laser.yaw<-M_PI) tmp_map_laser.yaw+=2*M_PI;	

		return tmp_map_laser;
				
}


/**
*@ brief : 将gnss坐标系/map下的坐标点转化为激光坐标系下/amcl_map的坐标点
*@ parameter: 激光坐标系的位置,地图编号
*@ return :gnss坐标系下的位置
*/


position PositionCombine::LaserTognssTranslate(position laser_pose, int num)
{
	int i=num;
	position tmp_laser,tmp_map_gnss;
	if(i<0) {
		ROS_INFO("Position_combine:: map_num error,can't translate to gnss map");
		return tmp_map_gnss;
	}
	double theta=map_laser_position[i].yaw;
	
	//转化到机器人坐标系
	tmp_laser.x = (laser_pose.x - map_laser_position[i].x) * cos(theta) + (laser_pose.y -map_laser_position[i].y) * sin(theta);
				
	tmp_laser.y = (laser_pose.y - map_laser_position[i].y) * cos(theta) - (laser_pose.x -map_laser_position[i].x) * sin(theta);
	
	tmp_laser.yaw=laser_pose.yaw-map_laser_position[i].yaw;
	
	if(tmp_laser.yaw>M_PI) tmp_laser.yaw-=2*M_PI;
	else if(tmp_laser.yaw<-M_PI) tmp_laser.yaw+=2*M_PI;
	
	
	//转化到map_gnss坐标系
	theta=map_gnss_position[i].yaw;
	tmp_map_gnss.x = map_gnss_position[i].x + tmp_laser.x * cos(theta) -tmp_laser.y * sin(theta);
	tmp_map_gnss.y = map_gnss_position[i].y + tmp_laser.x * sin(theta) + tmp_laser.y * cos(theta);
	tmp_map_gnss.yaw=tmp_laser.yaw+map_gnss_position[i].yaw;
	
	if(tmp_map_gnss.yaw>M_PI) tmp_map_gnss.yaw-=2*M_PI;
	else if(tmp_map_gnss.yaw<-M_PI) tmp_map_gnss.yaw+=2*M_PI;
	
	return tmp_map_gnss;

}


void PositionCombine::pross()
{

  if(first_flag){
  	   // ROS_INFO("localization=%d",localization_type);
	  initPoseFilter(); //开机选择,发送定位模式
	 // ROS_INFO("init localization_type=%d",localization_type);

	  if(3==localization_type){ //激光定位
	  	  std_msgs::Int64 num;
		  num.data=map_num;
		  map_num_pub.publish(num); //发送地图编号
		  
	      initPosePub(pose);
	      tfPub(pose);//更新odom -> map 坐标系的对应关系
	  	  pose.flag=false;
	  }
	  else if(2==localization_type){ //uwb定位
		  	if(uwb_raw_pose.flag){
				pose=uwb_raw_pose;
				initPosePub(pose);
		      	tfPub(pose);  
				uwb_raw_pose.flag=false;
				pose.flag=false;
			}
			else return;
			
	}

	else if(1==localization_type){
		ROS_INFO("Position_combine::gnss localization start!!!"); 		
	   }
	else return;

  first_flag=false;

  position tmp_pose;
  tmp_pose.x=0;
  tmp_pose.y=0;
  tmp_pose.yaw=0;
  map_to_laser_pose=gnssToLaserTranslate(tmp_pose,map_num);
  //map_to_laser_pose=tmp_pose;
  tf2Pub(map_to_laser_pose);// 发布map -->map_laser坐标系
  return;  
}

  
  bool pose_combine_pub_flag=true;
    //超时判断
    
  if(( ros::Time::now()-gps_signal_time).toSec() >1)
  	  gps_signal_flag = false;
  if(( ros::Time::now()-uwb_signal_time).toSec() >1)
  	  uwb_signal_flag = false;
  
  if(( ros::Time::now()-uwb_position.time).toSec() >0.5)
      uwb_position.flag=false;
  if( (ros::Time::now()-gnss_position.time).toSec() >0.5)
      gnss_position.flag=false;
  if( (ros::Time::now()-laser_position.time).toSec() >0.5)
      laser_position.flag=false;

  //判断定位区域

  if(0==IF_SELF_LOCALIZATION_TYPE && !region_change_flag && !navigate_stop){

	tmp_localization_type=localizationRegionDecide();
	
	//切换定位模式
	if(tmp_localization_type!=0 && tmp_localization_type!=localization_type && localization_type!=0){
		region_change_flag=true;
		ROS_INFO("Position_combine::localization type change from %d to %d",localization_type,tmp_localization_type);
		//
		
	}	
  }
  
  //先停止，停止后，发送新的模式，收到数据5s后退出继续导航
   if(region_change_flag){
	  std_msgs::Int64 tmp_stop;
  	  tmp_stop.data=1;
	  navigate_stop=true;
  	  stop_pub.publish(tmp_stop);

	  if(0==odom_position.twist.twist.linear.x && 0==odom_position.twist.twist.angular.z && !init_one_time_flag){

	  	position tmp_init_pose;
	  	if(3==tmp_localization_type && localization_type!=3 ){//非激光模式转到激光模式
	  	
			tmp_init_pose=gnssToLaserTranslate(combine_pose,map_num);
			tmp_init_pose.frame_id=laser_map_frame_id;
			std_msgs::Int64 num;
			num.data=map_num;
			map_num_pub.publish(num);
		}
		else {
			tmp_init_pose=combine_pose;
			tmp_init_pose.frame_id=global_frame_id_;
		}
		init_one_time_flag=true;
	  	localization_type = tmp_localization_type;
		initPosePub(tmp_init_pose);//切换模式时初始化一次
		
	  }

	  pose.flag=false;
   }
  
  
  std_msgs::Int64 tmp_type;
  tmp_type.data=localization_type;
  localization_type_pub.publish(tmp_type);

  
   //读取定位数据
   
  if(uwb_position.flag && 2== localization_type ){
        pose=uwb_position;
        uwb_position.flag=false;
        tfPub(pose);

   }
    else{
        if(gnss_position.flag && 1== localization_type ){
            pose=gnss_position;
            gnss_position.flag=false;
            tfPub(pose);
        }
        else {
            if(laser_position.flag && 3== localization_type ){
                pose= LaserTognssTranslate(laser_position, map_num) ;
				pose.flag=true;
                laser_position.flag=false;
                tfPub(pose);
            }
			else pose_combine_pub_flag=false; //没有收到任何定位信息
        }
    }
 //  ROS_INFO("type=%d,combine_pose(%f,%f,%f) uwb_pose(%f,%f,%f)",localization_type,combine_pose.x,combine_pose.y,combine_pose.yaw,uwb_position.x,uwb_position.y,uwb_position.yaw);

	if(region_change_flag){

		if((localization_type == tmp_localization_type) && pose.flag ){//新的模式，收到数据
			region_change_time=ros::Time::now();
			region_change_flag=false;
			init_one_time_flag=false;
			ROS_INFO("Position_combine:: localization type change over , %d",localization_type);
			
			pose.flag=false;			
			
		}
	}

	if( !region_change_flag && ( ros::Time::now()-region_change_time).toSec() >5){ //5s后恢复导航
	
	  std_msgs::Int64 tmp_stop;
  	  tmp_stop.data=0;
	  navigate_stop=false;
  	  stop_pub.publish(tmp_stop);
	}
	
    tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(), ros::Time::now(),
                                      global_frame_id_, odom_frame_id_);
    this->tfb_->sendTransform(tmp_tf_stamped);
	

	tf2Pub(map_to_laser_pose);// 发布map -->map_laser坐标系
//ROS_INFO("map_to_laser_pose=(%f,%f)",map_to_laser_pose.x,map_to_laser_pose.y);
    //if(localization_type==3 && !region_change_flag){
    map_listener.waitForTransform(global_frame_id_,base_frame_id_, ros::Time(0), ros::Duration(10.0));
	
    try{
       map_listener.lookupTransform(global_frame_id_,base_frame_id_,ros::Time(0), map_transform);
	}
       catch(tf::TransformException &ex){
           ROS_INFO("PositionCombine::map_laser to base_footprint tf error!");
	   	   ROS_ERROR("%s",ex.what());
	   	   return;
	   }
	
	//}   
	
	if(pose_combine_pub_flag ||localization_type==3){
		
		pose.flag=false;
		nav_msgs::Odometry tmp_data;

		combine_pose.x=map_transform.getOrigin().x();
		combine_pose.y=map_transform.getOrigin().y();
		combine_pose.yaw=tf::getYaw(map_transform.getRotation());
		
		combine_pose.frame_id=global_frame_id_;
		
		tmp_data=odom_position;
		
		tmp_data.header.frame_id = global_frame_id_;
		tmp_data.header.stamp = ros::Time::now();
	    tmp_data.pose.pose.position.x=combine_pose.x;
	    tmp_data.pose.pose.position.y=combine_pose.y;
	    tmp_data.pose.pose.orientation=tf::createQuaternionMsgFromYaw(combine_pose.yaw);
		combine_pose_pub.publish(tmp_data);
	}
	
}


/**
*@ brief :  判断是否进入相应定位区域
*@ parameter: position robot_pose  ,position  start_pose
*@ return bool
*/
bool PositionCombine::poseDecide(position robot_pose ,position start_pose)
{	
	double dis=(robot_pose.x-start_pose.x)*(robot_pose.x-start_pose.x)+(robot_pose.y-start_pose.y)*(robot_pose.y-start_pose.y);
	double dif_yaw=robot_pose.yaw-start_pose.yaw;
	
	if(dif_yaw>M_PI) dif_yaw-=2*M_PI;
	else if(dif_yaw<-M_PI) dif_yaw+=2*M_PI;
	//进入相应定位区域
	if(dis<RIGION_DECIDE_DISTANCE*RIGION_DECIDE_DISTANCE && fabs(dif_yaw)<RIGION_DECIDE_RADIAN)
		return true;
	else 
		return false;
		
}


/**
*@ brief :  判断定位区域,选择相应的定位方式
*@ parameter:
*@ return 
*/
int PositionCombine::localizationRegionDecide()
{

	for(int i=0;i<laser_start_pose.size();i++){
		if(poseDecide(combine_pose,laser_start_pose[i])){
			if(map_num!=laser_start_pose[i].map_num){//
			ROS_INFO("Position_combine:: map num change from %d to %d",map_num,laser_start_pose[i].map_num);	
			map_num=laser_start_pose[i].map_num;
			position tmp_pose;
			tmp_pose.x=0;
			tmp_pose.y=0;
			tmp_pose.yaw=0;
			map_to_laser_pose=gnssToLaserTranslate(tmp_pose,map_num);
			
			}
			return 3;
			
		}
	}

	for(int i=0;i<uwb_start_pose.size();i++){
		if(poseDecide(combine_pose,uwb_start_pose[i])){
			return 2;
			
		}
	}
	
	for(int i=0;i<gnss_start_pose.size();i++){
		if(poseDecide(combine_pose,gnss_start_pose[i])){
			return 1;

		}
	}
	
	return 0;

}



/**
*@ brief :  更新map -> map_laser 坐标系的对应关系
*@ parameter: map 坐标系下的全局位置（x,y,yaw） 
*@ return 
*/

bool PositionCombine::tf2Pub(position data)
{
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(data.yaw);
	//发布odom-->base_footprint坐标系转换
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id =laser_map_frame_id;
	odom_trans.child_frame_id = global_frame_id_;
	odom_trans.transform.translation.x = data.x;
	odom_trans.transform.translation.y =data.y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;
	map_to_laser_transform.sendTransform(odom_trans);
   return true;
}

/**
*@ brief :  更新odom -> map 坐标系的对应关系
*@ parameter: map 坐标系下的全局位置（x,y,yaw） 
*@ return 
*/

bool PositionCombine::tfPub(position data)
{
	double x=data.x;
	double y=data.y;
	double yaw=data.yaw;
    tf::Stamped<tf::Pose> odom_to_map;
    try
    {
    tf::Transform tmp_tf(tf::createQuaternionFromYaw(yaw), tf::Vector3(x,y, 0.0));
    tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),odom_position.header.stamp,base_frame_id_);
    this->tf_->transformPose(odom_frame_id_, tmp_tf_stamped, odom_to_map);
    }
    catch(tf::TransformException)
    {
     ROS_INFO("Position Combine:Failed to subtract base_link  to odom transform");
     return false;
    }

    latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                           tf::Point(odom_to_map.getOrigin()));

   return true;

}


void PositionCombine::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom_position=*msg;
}


void PositionCombine::gpsRawCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	if(( ros::Time::now()-msg->header.stamp).toSec() <1){
		gps_signal_flag=true;
		gps_signal_time=ros::Time::now();
	}
}


void PositionCombine::gpsPositionCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

	if(( ros::Time::now()-msg->header.stamp).toSec() <1){
		 gnss_position.x=msg->pose.pose.position.x;
		 gnss_position.y=msg->pose.pose.position.y;
		 gnss_position.yaw=tf::getYaw(msg->pose.pose.orientation);
		 gnss_position.frame_id=global_frame_id_;
		 if(gps_signal_flag){
		  	gnss_position.flag=true;
		  	gnss_position.time=ros::Time::now();
		 }
	}
}


void PositionCombine::uwbPositionCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	if(( ros::Time::now()-msg->header.stamp).toSec() <1){
	  uwb_position.x=msg->pose.pose.position.x;
	  uwb_position.y=msg->pose.pose.position.y;
	  uwb_position.yaw=tf::getYaw(msg->pose.pose.orientation);
	  uwb_position.frame_id=global_frame_id_;
	  if(uwb_signal_flag){
	  	uwb_position.flag=true;
	  	uwb_position.time=ros::Time::now();
	  }

	}
}


void PositionCombine::laserPositionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{

  laser_position.x=msg->pose.pose.position.x;
  laser_position.y=msg->pose.pose.position.y;
  laser_position.yaw=tf::getYaw(msg->pose.pose.orientation);
  laser_position.flag=true;
  laser_position.time=ros::Time::now();

}


void PositionCombine::uwbSignalCallback(const mrpt_msgs::ObservationRangeBeacon::ConstPtr& msg)
{	
	
	if(( ros::Time::now()-msg->header.stamp).toSec() <1){
		int beacon_num=0;
		for(int i=0;i<msg->sensed_data.size();i++ ){
			if(msg->sensed_data[i].range >msg->min_sensor_distance)
			beacon_num++;	
				
		}
		//ROS_INFO("beacon_num=%d",beacon_num);
		if(beacon_num>=3){
			uwb_signal_flag=true;
			uwb_signal_time=ros::Time::now();
			
			
		}
			
	}
	
}


void PositionCombine::chargeSignalCallback(const std_msgs::Int64::ConstPtr& msg)
{
	if(msg->data==1){
		charge_signal_flag=true;
		charge_signal_time=ros::Time::now();
	}
}


void PositionCombine::uwbRawPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	if(( ros::Time::now()-msg->header.stamp).toSec() <1){

		uwb_raw_pose.x=msg->pose.pose.position.x;
		uwb_raw_pose.y=msg->pose.pose.position.y;
		uwb_raw_pose.frame_id=global_frame_id_;
		if(imu_flag){
			uwb_raw_pose.yaw=imu_yaw-M_PI;
			if(uwb_raw_pose.yaw>M_PI) uwb_raw_pose.yaw=uwb_raw_pose.yaw-2*M_PI;
			else if(uwb_raw_pose.yaw<-M_PI) uwb_raw_pose.yaw=uwb_raw_pose.yaw+2*M_PI;
			imu_flag=false;
			uwb_raw_pose.flag=true;
		}
	}
	
}


void PositionCombine::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
{
	if(( ros::Time::now()-msg->header.stamp).toSec() <0.5){
		imu_yaw = tf::getYaw(msg->orientation);
		imu_flag=true;
	}
}


void PositionCombine::initPoseCallback(const geometry_msgs :: PoseWithCovarianceStamped :: ConstPtr & msg)
{
	//if(( ros::Time::now()-msg->header.stamp).toSec() <1)
{		position tmp;
		tmp.x=msg->pose.pose.position.x;
		tmp.y=msg->pose.pose.position.y;
		tmp.yaw=tf::getYaw(msg->pose.pose.orientation);
		tmp.frame_id=msg->header.frame_id;
		initPosePub(tmp);
		ROS_INFO("init localization_type=%d",localization_type);
		
		if(msg->header.frame_id==laser_map_frame_id){ //激光地图坐标系下的初始化	
		
			if(map_num<0) map_num=0;
			init_pose=LaserTognssTranslate(tmp, map_num);
			init_pose.frame_id=global_frame_id_;
			init_pose.flag= true;
			ROS_INFO("map pose(%f,%f,%f),laser pose(%f,%f,%f)",init_pose.x,init_pose.y,init_pose.yaw,tmp.x,tmp.y,tmp.yaw);
			tfPub(init_pose);
			ROS_INFO("Position_combine:: receive init pose with laser map!");
		}	
		
		else if(msg->header.frame_id==global_frame_id_){
			init_pose=tmp;
			tfPub(init_pose); //更新tf
			ROS_INFO("Position_combine:: receive init pose with gnss map!");
		}
			
	}
}

void PositionCombine::mapNumCallback(const std_msgs :: Int64 :: ConstPtr & msg)
{
	map_num=msg->data;
ROS_INFO("call_back manual map num=%d",map_num);
}


void PositionCombine::localizationTypeCallback(const std_msgs :: Int64 :: ConstPtr & msg)
{
	localization_type=msg->data;
ROS_INFO("call_back manual localization type=%d",localization_type);
}

