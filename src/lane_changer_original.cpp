/*
 * lane_changer_original.cpp
 *
 * ORIGINAL VERSION — LiDAR-only obstacle detection (no sensor fusion).
 *
 * Known limitations that motivated the improved version:
 *   1. False positives from track cones and curbs detected as obstacles.
 *   2. No cut-in detection — caused a direct collision and disqualification
 *      in a previous competition when an opponent vehicle merged into our lane.
 *   3. Inspection test failures — sparse LiDAR returns on cones caused
 *      missed detections, leading to repeated cone collisions.
 *
 * See lane_changer.cpp for the improved version featuring:
 *   - PointPainting sensor fusion (camera + LiDAR)
 *   - TwinLiteNet lane segmentation + cut-in detection
 *   - Robust defensive logic against false detections
 */

#include "new_lane/lane_changer.h"
#include <std_msgs/Int32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


void LaneChanger::initSetup() {
	// morai_msgs::CtrlCmd ackermann_msg_;
	
	//publisher
	// ackermann_pub_ = nh_.advertise<morai_msgs::CtrlCmd>("ctrl_cmd", 1);

	pub_points_ = nh_.advertise<sensor_msgs::PointCloud2>("passed_points", 10);
	pub_center_ = nh_.advertise<visualization_msgs::Marker>("center_points", 10);
	pub_center_local = nh_.advertise<visualization_msgs::Marker>("center_points_local", 10);
	pub_marker_ = nh_.advertise<visualization_msgs::Marker>("wayPoint", 10);
	lidar_global_pub = nh_.advertise<geometry_msgs::PoseArray>("/lidar_global",1);
	frenet_path = nh_.advertise<nav_msgs::Path>("/frenet_path", 1);   // Publishes frenet path
	frenet_path_2 = nh_.advertise<nav_msgs::Path>("/frenet_path_2", 1);   // Publishes frenet path
	pub_offset_path1 = nh_.advertise<nav_msgs::Path>("/offset_path1", 1); // right
	pub_offset_path2 = nh_.advertise<nav_msgs::Path>("/offset_path2", 1); // left
	global_path = nh_.advertise<nav_msgs::Path>("/global_path", 1);   // Publishes global path
	pub_lane_ = nh_.advertise<waypoint_maker::State>("/lane_number_msg_", 1);
	pub_frenet_points = nh_.advertise<waypoint_maker::Lane>("frenet_points", 1);
	pub_lidar_topic = nh_.advertise<new_lane::lidar_topic_msg>("/lidar_topic", 1);
	pub_path_number_ = nh_.advertise<std_msgs::Int32>("/path_number", 1);e


	//subscriber
	sub_ = nh_.subscribe("/demo/nonground", 1, &LaneChanger::pointCallback, this);
	lane_sub_ = nh_.subscribe("/final_waypoint", 1, &LaneChanger::LaneCallback, this);
	// imu_sub_ = nh_.subscribe("/imu",1,&LaneChanger::ImuCallback,this);
	state_sub_ = nh_.subscribe("/gps_state", 1, &LaneChanger::state_Callback, this);
	odom_sub_ = nh_.subscribe("/odom", 1, &LaneChanger::OdomCallback,this);
	// course_sub_ = nh_.subscribe("course_path", 1, &LaneChanger::CourseCallback,this);
	// closest_index_sub_ = nh_.subscribe("/closest_index", 1, &LaneChanger::closest_index_Callback, this);

	
	//param for static 
	ros::param::get("~cluster_tolerance", cluster_tolerance_);
	ros::param::get("~cluster_minsize", cluster_minsize_);
	ros::param::get("~cluster_maxsize", cluster_maxsize_);
	ros::param::get("~str_",str_);// **' '** (Offset)
	ros::param::get("~sepe",sepe);// **' '** (Interval)
	
	
	ros::param::get("~x_min_st", x_min);
	ros::param::get("~x_max_st", x_max);
	ros::param::get("~y_min_st", y_min);
	ros::param::get("~y_max_st", y_max);
	ros::param::get("~z_min_st", z_min);
	ros::param::get("~z_max_st", z_max);

	//load parameters
	ros::param::get("/path/max_speed", MAX_SPEED);
	ros::param::get("/path/max_accel", MAX_ACCEL);
	ros::param::get("/path/max_curvature", MAX_CURVATURE);// [1/m]
	ros::param::get("/path/max_road_width", MAX_ROAD_WIDTH);// [m]
	ros::param::get("/path/d_road_w", D_ROAD_W);
	ros::param::get("/path/dt", DT);
	ros::param::get("/path/maxt", MAXT);
	ros::param::get("/path/mint", MINT);//''
	ros::param::get("/path/target_speed", TARGET_SPEED);
	ros::param::get("/path/d_t_s", D_T_S);
	ros::param::get("/path/n_s_sample", N_S_SAMPLE);//''
	ros::param::get("/path/robot_radius", ROBOT_RADIUS);
	ros::param::get("/path/max_lat_vel", MAX_LAT_VEL);
	ros::param::get("/path/min_lat_vel", MIN_LAT_VEL);//''
	ros::param::get("/path/d_d_ns", D_D_NS);//''
	ros::param::get("/path/max_shift_d", MAX_SHIFT_D);// m

	ros::param::get("/cost/kj", KJ);// (Jerk)
	ros::param::get("/cost/kt", KT);
	ros::param::get("/cost/kd", KD);
	ros::param::get("/cost/kd_v", KD_V);
	ros::param::get("/cost/klon", KLON);
	ros::param::get("/cost/klat", KLAT);
	
	ros::param::get("/frenet/d_offset", D_OFFSET);
	ros::param::get("/frenet/linear_speed", LINEAR_SPEED);
}

void LaneChanger::clustering(const pcl::PointCloud<PointType>::Ptr cloud_in) {
		
		//Voxelization -----------
		pcl::VoxelGrid<pcl::PointXYZ> vox;

		// sensor_msgs
		vox.setInputCloud(cloud_in);// vox
		vox.setLeafSize(0.15, 0.15, 0.15); // (m )
		// vox.setLeafSize(voxel_size, voxel_size, voxel_size);
		vox.filter(*cloud_in);

		pcl::PassThrough<pcl::PointXYZ> pass;//ROI

		pass.setInputCloud(cloud_in);//pass
		pass.setFilterFieldName("x"); // axis x
		pass.setFilterLimits(x_min,x_max);
		// pass.setFilterLimitsNegative(true);
		pass.setFilterLimitsNegative(false);//flase( )
		pass.filter(*cloud_in);

		pass.setInputCloud(cloud_in);
		pass.setFilterFieldName("y"); // axis y
		pass.setFilterLimits(y_min, y_max);
		// pass.setFilterLimitsNegative(true);
		pass.setFilterLimitsNegative(false);
		pass.filter(*cloud_in);
		
		pass.setInputCloud(cloud_in);
		pass.setFilterFieldName("z"); // axis z
		pass.setFilterLimits(z_min, z_max);
		// pass.setFilterLimitsNegative(true);
		pass.setFilterLimitsNegative(false);
		pass.filter(*cloud_in);

		sensor_msgs::PointCloud2 filteredOutput;// ( filteredOutput )
		pcl::toROSMsg(*cloud_in, filteredOutput);
		filteredOutput.header.frame_id = "os_sensor";
		pub_points_.publish(filteredOutput);

		// Populate point cloud...
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);//kd tree
		if(!cloud_in -> empty()) kdtree->setInputCloud(cloud_in);//kdtree zCloud ( )

		std::vector<pcl::PointIndices> clusterIndices;// (clusterindicies[0] [1])
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

		ec.setClusterTolerance(cluster_tolerance_);
		ec.setMinClusterSize(cluster_minsize_);    // set Minimum Cluster Size( )
		ec.setMaxClusterSize(cluster_maxsize_); // set Maximum Cluster Size

		ec.setSearchMethod(kdtree);
		ec.setInputCloud(cloud_in);

		ec.extract(clusterIndices);

		clustered_.clear();
		// pcl::PointCloud<pcl::PointXYZ> tmpCloud4;


		int cluster_idx = 0;
		center_.clear();

		for (vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
			pcl::PointCloud<PointType>::Ptr cluster_cloud (new pcl::PointCloud<PointType>);
			for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
				cluster_cloud->points.emplace_back(cloud_in->points[*pit]);
			}// cloud_in cluster_cloud
			PointXYZV P;
			pcl::PointXYZ min_pt, max_pt;
			pcl::getMinMax3D (*cluster_cloud, min_pt, max_pt);
			P.width = max_pt.x - min_pt.x;
			P.height = max_pt.y - min_pt.y;
			P.z_height = max_pt.z - min_pt.z;
			P.x = (max_pt.x + min_pt.x)/2;// x
			P.y = (max_pt.y + min_pt.y)/2;// y
			P.z = (max_pt.z + min_pt.z)/2;// z
			P.id = -1;
			P.vel = 0;
			P.dist = sqrt(pow(P.x,2)+pow(P.y,2));
			if(P.x > -2.0 && P.x < 0.5 && P.y > -0.85 && P.y < 0.85){} // x:1 4 y:-0 85~) 85 //
			else{
				if(P.z_height > 0.3){	// 0.3
					center_.emplace_back(P);
				}// 30cm center_
			}

			//static
			sort(center_.begin(),center_.end(),cmp);// cmp
		}
	
}


bool LaneChanger::cmp(const PointXYZV &v1, const PointXYZV &v2){
	return v1.dist < v2.dist;
}// true v1 v1 center_[0] [1]

void LaneChanger::pointCallback(const sensor_msgs::PointCloud2::ConstPtr &scan) {
		pcl::fromROSMsg(*scan, *msgCloud);//scan=ros msgcloud=pcl
}//ros pcl

void LaneChanger::state_Callback(const waypoint_maker::State::ConstPtr &msg){
	lane_number_ = msg->lane_number;
	current_state_ = msg->current_state;// (stop go)
	// cout<<"lane_number : "<<lane_number_<< endl;
}

// void LaneChanger::closest_index_Callback(const std_msgs::Int32::ConstPtr &msg){
// 	closest_index = msg->data;
// }

// void LaneChanger::CourseCallback(const std_msgs::Float64::ConstPtr &course_msg){
// 	cur_course_rad_ = course_msg -> data;
// }

void LaneChanger::LaneCallback(const waypoint_maker::Lane::ConstPtr &lane_msg)
{
	waypoints_.clear();
	W_X.clear();
	W_Y.clear();
	vector<waypoint_maker::Waypoint>().swap(waypoints_);
	waypoints_ = lane_msg->waypoints;
	waypoints_size_ = waypoints_.size();
	if (waypoints_size_ != 0) is_lane_ = true;
	// cout << "!!!!!!!!!! waypoint loaded !!!!!!!!!!" << endl;
	for(int i=0; i<waypoints_size_; i++){
		W_X.push_back(waypoints_[i].pose.pose.position.x);
		W_Y.push_back(waypoints_[i].pose.pose.position.y);
	}//waypoints_ x y w_x w_y
}

void LaneChanger::OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
	cur_pose_.header = odom_msg->header;//cur_pose=
	cur_pose_.pose.position = odom_msg->pose.pose.position;
    
	tf2::Quaternion q(
		odom_msg->pose.pose.orientation.x,
		odom_msg->pose.pose.orientation.y,
		odom_msg->pose.pose.orientation.z,
		odom_msg->pose.pose.orientation.w
	);
	double roll, pitch, yaw;
	tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
	cur_course_rad_ = yaw;// yaw ( )

	// cout << "position X: " << cur_pose_.pose.position.x << endl;
	// cout << "position Y: " << cur_pose_.pose.position.y << endl;
}

float LaneChanger::calcul_angle(float x, float y) {
	float ang;
	ang = atan2f(y, x)*180/M_PI;//from radian to degree
	return ang;
}

void LaneChanger::CircleDecision1(vecD x1, vecD y1, vecD x2, vecD y2,float r){	// my line
	is_obs1 = false;
	// cout << "CircleDecision1" << endl;

	if(x1.size() == 0 || x2.size() == 0){
		is_obs1 = false;
		lane_cnt1 = 0;
	}
	
	else if(x1.size() != 0 && x2.size() != 0){
		for(int j=0; j<x1.size(); j++){
			for(int i=0; i<x2.size(); i++){
				is_circle1 = sqrt(pow(x1[j]-x2[i], 2) + pow(y1[j]-y2[i], 2));
				if(is_circle1 < r) {
					cout << "!!!!!!!!! my line obstacle !!!!!!!!" << endl;
					is_obs1 = true;
					obs_index1 = j;
					lane_cnt1 ++;
					break;
				}
				else if(is_circle1 > r){
					is_obs1 = false;
				}
			}
			if(is_obs1) break;
		}
	}
	if(is_obs1){
		my_line_obs = true;
		cout << "[Obs] my lane obstacle confirmed" << endl;
	}
	else{
		my_line_obs = false;
		lane_cnt1 = 0;
	}
}

void LaneChanger::CircleDecision2(vecD x1, vecD y1, vecD x2, vecD y2,float r){
	is_obs2 = false;// false (is_obs1 2 )

	if(x1.size() == 0 || x2.size() == 0) {
		is_obs2 = false;
		lane_cnt2 = 0;
	}//x1 y1= x2 y2= circledecision2 x1 y1= x2 y2 (circledecision1 )

	else if(x1.size() != 0){
		for(int j=0; j<x1.size(); j++){
			for(int i=0; i<x2.size(); i++){
				is_circle2 = sqrt(pow(x1[j]-x2[i], 2) + pow(y1[j]-y2[i], 2));
				if(is_circle2 < r) {
					cout << "!!!!!!!!! other obstacle !!!!!!!!" << endl;
					is_obs2 = true;// true( )
					obs_index2 = j;
					lane_cnt2 ++;
					break;
				}
				else if(is_circle2 > r){
					is_obs2 = false;

				}
			}
			if(is_obs2) break;
		}
	}
	if(is_obs2){
		other_line_obs = true;
		cout << "[Obs] adjacent lane obstacle confirmed" << endl;
	}
	else{
		other_line_obs = false;
		lane_cnt2 = 0;
	}
}

bool LaneChanger::CircleDecision1_c(vecD x1, vecD y1, vecD x2, vecD y2,float r){
	is_obs1_c = false;

	if(x1.size() == 0 || x2.size() == 0) {
		is_obs1_c = false;
		// lane_cnt1_c = 0;
	}

	else if(x1.size() != 0){
		for(int j=0; j<x1.size(); j++){
			for(int i=0; i<x2.size(); i++){
				is_circle2 = sqrt(pow(x1[j]-x2[i], 2) + pow(y1[j]-y2[i], 2));
				if(is_circle2 < r) {
					is_obs1_c = true;
					obs_index1_c = j;
					// lane_cnt1_c ++;
					break;
				}
				else if(is_circle2 > r){
					is_obs1_c = false;

				}
			}
			if(is_obs1_c) break;
		}
	}
	return is_obs1_c;
}


bool LaneChanger::CircleDecision2_c(vecD x1, vecD y1, vecD x2, vecD y2,float r){
	is_obs2_c = false;

	if(x1.size() == 0 || x2.size() == 0) {
		is_obs2_c = false;
		// lane_cnt2_c = 0;
	}

	else if(x1.size() != 0){
		for(int j=0; j<x1.size(); j++){
			for(int i=0; i<x2.size(); i++){
				is_circle2 = sqrt(pow(x1[j]-x2[i], 2) + pow(y1[j]-y2[i], 2));
				if(is_circle2 < r) {
					is_obs2_c = true;
					// obs_index2_c = j;
					// lane_cnt2_c ++;
					break;
				}
				else if(is_circle2 > r){
					is_obs2_c = false;

				}
			}
			if(is_obs2_c) break;
		}
	}
	return is_obs2_c;
}

void LaneChanger::LaneChage_Decision(bool my_obs, bool other_obs){	// lane_num 0 : lane_num 1 : lane_num 2 :
	if(lane_num == 0){
		if(my_obs){
			if(other_obs){
				lane_num = lane_number_ ;// ( ) lane_num( )lane_number( ) !
				acc_index1 = find_closest_waypoint(global_obs, obs_index1, waypoints_);
				cout << "acc_index1 : " << acc_index1 << endl;
				acc_flag = true;
			}// acc_flag=true acc
			else{
				if(lane_cnt1 > 1){
					lane_cnt1 = 0;
					lane_cnt2 = 0;

					lane_num = 1;
					acc_flag = false;	
				}
				else {
					lane_num = 0;
				}	
			}// lane_num=1 ( ) acc_flag=false acc
		}
		else {
			lane_num = 0;
			acc_flag = false;

		}// my_obs=false lane_num=0 ( ) acc
	}
	else if (lane_num == 1){
		if(my_obs){
			if(other_obs){
				lane_num = lane_number_ ;
				acc_index1 = find_closest_waypoint(global_obs, obs_index1, waypoints_);
				cout << "acc_index1 : " << acc_index1 << endl;
				acc_flag = true;
			}// ( ) ( ) acc
			else{
				if(lane_cnt1 > 1){
					lane_cnt1 = 0;
					lane_cnt2 = 0;

					lane_num = 0;
					acc_flag = false;
				}
				else {
					lane_num = 1;
				}	
			}// ( ) (lane_num=0)
		}
		else {
			lane_num = 1;
			acc_flag = false;


		}

	}
	// cout << "lane_num : " << lane_num << endl;
	lidar_topic_msg.lane_num = lane_num;
	lidar_topic_msg.ACC_speed = 0; // no using
	lidar_topic_msg.ACC_flag = acc_flag;// or
	lidar_topic_msg.ACC_index_gap = acc_index1;// (index) ?
	pub_lidar_topic.publish(lidar_topic_msg);//lane_num ACC_speed ACC_flag ACC_index_gap

	std_msgs::Int32 path_msg;
	path_msg.data = lane_num;
	pub_path_number_.publish(path_msg);


}

void LaneChanger::only_ACC(bool my_obs){	// lane_num 0 : lane_num 2 :
	if(my_obs){
		if(lane_cnt1 > 2){
			lane_cnt1 = 0;
			// cout << "----ACC----" << endl;
			lane_num = lane_number_ ;
			acc_index1 = find_closest_waypoint(global_obs, obs_index1, waypoints_);
			cout << "acc_index1 : " << acc_index1 << endl;
			acc_flag = true;//acc ( )
		}			
	}
	else {
		acc_flag = false;// ACC
		lane_cnt1 = 0;
	}
	// cout << "lane_num : " << lane_num << endl;
	lidar_topic_msg.lane_num = lane_num;
	lidar_topic_msg.ACC_speed = 0; // no using
	lidar_topic_msg.ACC_flag = acc_flag;
	lidar_topic_msg.ACC_index_gap = acc_index1;
	pub_lidar_topic.publish(lidar_topic_msg);

}

void LaneChanger::only_ACC_2(bool my_obs){	// lane_num 0 : lane_num 2 :
	lane_cnt1 = 0;
	lane_cnt2 = 0;
	if(my_obs){
		// cout << "----ACC----" << endl;
		lane_num = lane_number_ ;
		acc_index1_c = find_closest_waypoint(global_obs, obs_index1_c, waypoints_);
		cout << "acc_index1_c : " << acc_index1_c << endl;
		acc_flag = true;
	}
	else {
		acc_flag = false;
	}
	// cout << "lane_num : " << lane_num << endl;
	lidar_topic_msg.lane_num = lane_num;
	lidar_topic_msg.ACC_speed = 0; // no using
	lidar_topic_msg.ACC_flag = acc_flag;
	lidar_topic_msg.ACC_index_gap = acc_index1_c;
	pub_lidar_topic.publish(lidar_topic_msg);

}

void LaneChanger::Local_to_Global(vector<PointXYZV> input_points){
	global_obs.clear();
	double yaw = cur_course_rad_;// x yaw
	pcl::PointXYZ obs;
	if(input_points.size() != 0){
		for(int i=0; i<input_points.size(); i++){
			geometry_msgs::Pose pose_global;
		
			pose_global.position.x = cur_pose_.pose.position.x +  ((input_points[i].x+1.04) * cos(yaw)) - ((input_points[i].y) * sin(yaw));
			pose_global.position.y = cur_pose_.pose.position.y +  ((input_points[i].x+1.04) * sin(yaw)) + ((input_points[i].y) * cos(yaw));
			pose_global.position.z = 0;
			global_x.emplace_back(pose_global.position.x);
			global_y.emplace_back(pose_global.position.y);
			global_z.emplace_back(pose_global.position.z);

			obs.x = pose_global.position.x;
			obs.y = pose_global.position.y;
			obs.z = pose_global.position.z;

			global_obs.emplace_back(obs);
			cur_pose_global.poses.emplace_back(pose_global);
		}
	}
	// cout << "global obs size : " << global_obs.size() << endl;
    lidar_global_pub.publish(cur_pose_global);
}

int LaneChanger::find_closest_waypoint(vector<pcl::PointXYZ> obs_points, int index, vector<waypoint_maker::Waypoint> path){
	// cout << "---find_closest_waypoint()---" << endl;
	float between_dist = 0.0;
	int tmp_index = 0;
	vector<float> all_dist;
	if(obs_points.size() != 0 && path.size() != 0){
		all_dist.clear();

		for(int i=0; i<path.size(); i++){
			between_dist = sqrt(pow(obs_points[index].x-path[i].pose.pose.position.x, 2) + pow(obs_points[index].y-path[i].pose.pose.position.y, 2));
			all_dist.push_back(between_dist);

		}// all_dist
		float min_dist = 9999.9;
		cout << "all_dist size : " << all_dist.size() << endl;
		for(int j=0; j < all_dist.size(); j++){
			if(min_dist > all_dist[j]){
				min_dist = all_dist[j];
				tmp_index = j;
			}// for (min_dist)
		}
		cout << "tmp_index : " << tmp_index << endl;
	}
	else{
		all_dist.clear();
	}
	return tmp_index;//acc_index1 (acc_index1 )
}


geometry_msgs::Point LaneChanger::calculateNormal(const waypoint_maker::Waypoint& p1, const waypoint_maker::Waypoint& p2) {
    geometry_msgs::Point normal;
    normal.x = p2.pose.pose.position.y - p1.pose.pose.position.y;
    normal.y = -(p2.pose.pose.position.x - p1.pose.pose.position.x);
    double length = sqrt(pow(p1.pose.pose.position.x - p2.pose.pose.position.x, 2) + pow(p1.pose.pose.position.y - p2.pose.pose.position.y, 2));
	if(length == 0) length = 1;
    normal.x /= length;
    normal.y /= length;
    return normal;
}

void LaneChanger::offsetPath(const vector<waypoint_maker::Waypoint>& before_path, float offset) {	// lane_num 0 = right, lane_num 1 = left
	offset_path1_x.clear();
	offset_path1_y.clear();
	offset_path1.clear();

	if(before_path.size() != 0){
		if(lane_number_ == 0){
			real_D_OFFSET = offset;//offset( )
			cout << "real_D_OFFSET : " << real_D_OFFSET << endl;
				int n = before_path.size();
				for (int i = 0; i < n-1; i++) {
					const waypoint_maker::Waypoint& p1 = before_path[i];
					const waypoint_maker::Waypoint& p2 = before_path[(i + 1)];
					geometry_msgs::Point normal = calculateNormal(p1, p2);

					waypoint_maker::Waypoint offsetPoint1;
					offsetPoint1.pose.pose.position.x = p1.pose.pose.position.x + normal.x * real_D_OFFSET;//offsetpoint1:
					offsetPoint1.pose.pose.position.y = p1.pose.pose.position.y + normal.y * real_D_OFFSET;

					offset_path1_x.push_back(offsetPoint1.pose.pose.position.x);//offset_path1_x/y:
					offset_path1_y.push_back(offsetPoint1.pose.pose.position.y);

					offset_path1.push_back(offsetPoint1);//offset_path1:
				}
				nav_msgs::Path offset_path_msg1;

				offset_path_msg1.header.frame_id = "map";
				offset_path_msg1.poses.resize(offset_path1.size());
                
				for (int i = 0; i < offset_path1.size(); i++){
					geometry_msgs::PoseStamped off;
					off.pose.position.x = offset_path1[i].pose.pose.position.x;
					off.pose.position.y = offset_path1[i].pose.pose.position.y;
					offset_path_msg1.poses[i] = off;
				}
				pub_offset_path1.publish(offset_path_msg1);
			}

		else if(lane_number_ == 1){
			real_D_OFFSET = -1*offset;//offset -1 ( )
			cout << "real_D_OFFSET : " << real_D_OFFSET << endl;
			int n = before_path.size();
			for (int i = 0; i < n-1; i++) {
				const waypoint_maker::Waypoint& p1 = before_path[i];
				const waypoint_maker::Waypoint& p2 = before_path[(i + 1)];
				geometry_msgs::Point normal = calculateNormal(p1, p2);

				waypoint_maker::Waypoint offsetPoint1;
				offsetPoint1.pose.pose.position.x = p1.pose.pose.position.x + normal.x * real_D_OFFSET;
				offsetPoint1.pose.pose.position.y = p1.pose.pose.position.y + normal.y * real_D_OFFSET;

				offset_path1_x.push_back(offsetPoint1.pose.pose.position.x);
				offset_path1_y.push_back(offsetPoint1.pose.pose.position.y);

				offset_path1.push_back(offsetPoint1);
			}
			nav_msgs::Path offset_path_msg1;

			offset_path_msg1.header.frame_id = "map";
			offset_path_msg1.poses.resize(offset_path1.size());

			for (int i = 0; i < offset_path1.size(); i++){
				geometry_msgs::PoseStamped off;
				off.pose.position.x = offset_path1[i].pose.pose.position.x;
				off.pose.position.y = offset_path1[i].pose.pose.position.y;
				offset_path_msg1.poses[i] = off;
			}
			pub_offset_path1.publish(offset_path_msg1);
		}

		//reference path visualization
		nav_msgs::Path global_path_msg;
		
		if(W_X.size() != 0){
			global_path_msg.header.frame_id = "map";
			global_path_msg.poses.resize(W_X.size());
			for (int i = 0; i < W_X.size(); i++)
			{
				geometry_msgs::PoseStamped loc;
				loc.pose.position.x = W_X[i];
				loc.pose.position.y = W_Y[i];
				global_path_msg.poses[i] = loc;
			}
			global_path.publish(global_path_msg);
		}
	}
}

void LaneChanger::run(){
	
	clustering(msgCloud);
	Local_to_Global(center_);// (center_)
	cout << "lidar_lane_num : " << lane_num << endl;
	cout << "gps_lane_number_ : " << lane_number_ << endl;//gps (0 1)
	cout << "current_state_ : " << current_state_ << endl;

	
	if(current_state_ == 1 || current_state_ == 2 || current_state_ == 3 || current_state_ == 4 || current_state_ == 5 || current_state_ == 6 || current_state_ == 8 || current_state_ == 9 || current_state_ == 10){
		offsetPath(waypoints_, D_OFFSET);// D_OFFSET (W_X W_Y)
		CircleDecision1(global_x, global_y, W_X, W_Y, 1.8);// 1 8m
		only_ACC(my_line_obs);
		
	}
	else {
		if(lane_num == lane_number_){
			float new_offset = 0.0;
			float r_dist = 0.0;
			if(current_state_ == 0 || current_state_ == 11){
				new_offset = 3.0;// ( 3m )
				r_dist = 1.0;// r_dist
			}
			else if(current_state_ == 7){
				new_offset = 2.7;
				r_dist = 1.15;
			}
			else{
				new_offset = 2.55;
				r_dist = 1.1;
			}

			offsetPath(waypoints_, new_offset);
			if(lane_number_ == 0){
						
				CircleDecision1(global_x, global_y, W_X, W_Y, r_dist);
				CircleDecision2(global_x, global_y, offset_path1_x, offset_path1_y,r_dist);
				
				
				// CircleDecision1_c(global_x, global_y, W_X, W_Y, 1.9);
				// CircleDecision2_c(global_x, global_y, offset_path1_x, offset_path1_y,1.9);

				// if(is_obs1_c && is_obs2_c){
				// 	only_ACC_2(true);
				// 	cout << "!!!center_ACC!!!" << endl;
				// }
				// else{
				LaneChage_Decision(my_line_obs, other_line_obs);// (lane_number=0) lanechange_decision
				// }
			}
			else if(lane_number_ == 1){
				
				CircleDecision1(global_x, global_y, W_X, W_Y, r_dist);
				CircleDecision2(global_x, global_y, offset_path1_x, offset_path1_y,r_dist);
				
				
				// CircleDecision1_c(global_x, global_y, W_X, W_Y, 1.9);
				// CircleDecision2_c(global_x, global_y, offset_path1_x, offset_path1_y,1.9);

				// if(is_obs1_c && is_obs2_c){
				// 	only_ACC_2(true);
				// 	cout << "!!!center_ACC!!!" << endl;
				// }
				// else{
				LaneChage_Decision(my_line_obs, other_line_obs);
				// }			
			}
		}
	}
	visualize_center(global_obs);// rviz
	visualize_center_local(center_);// (local)
	global_x.clear();
	global_y.clear();
	
}

void LaneChanger::visualize_center(vector<pcl::PointXYZ> input_points) {
		visualization_msgs::Marker centers;
		geometry_msgs::Point point;

		centers.header.frame_id = "map";
		centers.header.stamp = ros::Time::now();
		centers.ns = "centers";
		centers.action = visualization_msgs::Marker::ADD;
		centers.pose.orientation.w = 1.0;
		centers.id = 2;
		centers.type = visualization_msgs::Marker::POINTS;
		centers.scale.x = 0.8; 
		centers.scale.y = 0.8;
		centers.color.a = 1.0;
		centers.color.g = 1.0f;
		centers.color.r = 1.0f;

		for (int i = 0; i < input_points.size(); i++) {
			point.x = input_points[i].x;
			point.y = input_points[i].y;
			point.z = 0;

			centers.points.push_back(point);
		}
	    pub_center_.publish(centers);
}


void LaneChanger::visualize_center_local(vector<PointXYZV> input_points) {
		visualization_msgs::Marker centers;
		geometry_msgs::Point point;

		centers.header.frame_id = "map";
		centers.header.stamp = ros::Time::now();
		centers.ns = "centers_l";
		centers.action = visualization_msgs::Marker::ADD;
		centers.pose.orientation.w = 1.0;
		centers.id = 1;
		centers.type = visualization_msgs::Marker::POINTS;
		centers.scale.x = 0.3; 
		centers.scale.y = 0.3;
		centers.color.a = 1.0;
		centers.color.g = 1.0f;
		centers.color.r = 1.0f;

		for (int i = 0; i < input_points.size(); i++) {
			point.x = input_points[i].x;
			point.y = input_points[i].y;
			point.z = input_points[i].z;

			centers.points.push_back(point);
		}
	    pub_center_local.publish(centers);
}

void LaneChanger::visualize(geometry_msgs::Point input_point) {
		// visualize waypoint
		visualization_msgs::Marker way_point;
		geometry_msgs::Point point;

		way_point.header.frame_id = "map";
		way_point.header.stamp = ros::Time::now();
		way_point.ns = "points_and_lines";
		way_point.action = visualization_msgs::Marker::ADD;
		way_point.pose.orientation.w = 1.0;
		way_point.id = 1;
		way_point.type = visualization_msgs::Marker::POINTS;
		way_point.scale.x = 0.05; 
		way_point.scale.y = 0.05;
		way_point.color.a = 1.0;
		way_point.color.r = 1.0f;

		way_point.points.push_back(input_point);
	
		pub_marker_.publish(way_point);
    }

int main(int argc, char **argv) {	
    ros::init(argc, argv, "lane_changer_node");

    LaneChanger lc;
	ros::Rate loop_rate(10);


	while(ros::ok()){		
		ros::spinOnce();
		lc.run();
		loop_rate.sleep();
	}
	return 0;

}
