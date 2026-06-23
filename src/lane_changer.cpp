/*
 * lane_changer.cpp
 *
 * Autonomous driving perception and decision-making module
 * for 1/2-scale racing vehicle competition (MORAI Simulator, ROS1).
 *
 * This module incorporates the following third-party methods:
 *
 * [1] PointPainting: Sequential Fusion for 3D Object Detection
 *     Vora et al., CVPR 2020
 *     https://arxiv.org/abs/1911.10150
 *
 * [2] TwinLiteNet: An Efficient and Lightweight Model for
 *     Driveable Area and Lane Segmentation
 *     Chequang Huy et al.
 *     https://github.com/chequanghuy/TwinLiteNet
 *
 * [3] YOLOv8-seg: Real-time Instance Segmentation
 *     Ultralytics
 *     https://github.com/ultralytics/ultralytics
 */

#include "new_lane/lane_changer.h"
#include <std_msgs/Int32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <opencv2/opencv.hpp>


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
	pub_path_number_ = nh_.advertise<std_msgs::Int32>("/path_number", 1);


	//subscriber
	sub_ = nh_.subscribe("/demo/nonground", 1, &LaneChanger::pointCallback, this);
	lane_sub_ = nh_.subscribe("/final_waypoint", 1, &LaneChanger::LaneCallback, this);
	state_sub_ = nh_.subscribe("/gps_state", 1, &LaneChanger::state_Callback, this);
	odom_sub_ = nh_.subscribe("/odom", 1, &LaneChanger::OdomCallback, this);
	yolo_sub_ = nh_.subscribe("/yolo/detections", 1, &LaneChanger::yoloCallback, this);
	seg_mask_sub_ = nh_.subscribe("/yolo/seg_mask", 1, &LaneChanger::segMaskCallback, this);
	lane_mask_sub_ = nh_.subscribe("/twinlitenet/lane_mask", 1, &LaneChanger::laneMaskCallback, this);

	
	//param for static 
	ros::param::get("~cluster_tolerance", cluster_tolerance_); // max distance between points to be grouped into same cluster
	ros::param::get("~cluster_minsize", cluster_minsize_); // minimum number of points to form a valid cluster
	ros::param::get("~cluster_maxsize", cluster_maxsize_);
	ros::param::get("~str_",str_); // start offset: how far from ego vehicle to begin obstacle checking
	ros::param::get("~sepe",sepe); // interval: how densely to check along the path
	
	
	ros::param::get("~x_min_st", x_min); // ROI (Region of Interest) filter bounds
	ros::param::get("~x_max_st", x_max);
	ros::param::get("~y_min_st", y_min);
	ros::param::get("~y_max_st", y_max);
	ros::param::get("~z_min_st", z_min);
	ros::param::get("~z_max_st", z_max);

	//load parameters
	ros::param::get("/path/max_speed", MAX_SPEED); // maximum allowed vehicle speed
	ros::param::get("/path/max_accel", MAX_ACCEL); // maximum acceleration
	ros::param::get("/path/max_curvature", MAX_CURVATURE); // maximum curvature [1/m]
	ros::param::get("/path/max_road_width", MAX_ROAD_WIDTH); // total road width available for lateral movement [m]
	ros::param::get("/path/d_road_w", D_ROAD_W); // lateral step size when generating candidate lane change paths
	ros::param::get("/path/dt", DT); // time step for path planning
	ros::param::get("/path/maxt", MAXT); // maximum prediction horizon [s]
	ros::param::get("/path/mint", MINT); // minimum prediction horizon [s]
	ros::param::get("/path/target_speed", TARGET_SPEED); // desired cruising speed
	ros::param::get("/path/d_t_s", D_T_S); // speed sampling interval around target speed
	ros::param::get("/path/n_s_sample", N_S_SAMPLE); // number of speed samples
	ros::param::get("/path/robot_radius", ROBOT_RADIUS); // ego vehicle radius (modeled as circle)
	ros::param::get("/path/max_lat_vel", MAX_LAT_VEL); // max lateral velocity during lane change (higher = faster lane change)
	ros::param::get("/path/min_lat_vel", MIN_LAT_VEL); // min lateral velocity during lane change
	ros::param::get("/path/d_d_ns", D_D_NS); // lateral velocity sampling interval
	ros::param::get("/path/max_shift_d", MAX_SHIFT_D); // max lateral offset from lane center [m]

	ros::param::get("/cost/kj", KJ); // jerk cost weight: higher = smoother ride
	ros::param::get("/cost/kt", KT); // time cost weight: higher = faster arrival
	ros::param::get("/cost/kd", KD); // lateral deviation cost weight: higher = stays closer to lane center
	ros::param::get("/cost/kd_v", KD_V); // speed deviation cost weight
	ros::param::get("/cost/klon", KLON); // longitudinal overall cost weight
	ros::param::get("/cost/klat", KLAT); // lateral overall cost weight (lane change)
	
	ros::param::get("/frenet/d_offset", D_OFFSET);
	ros::param::get("/frenet/linear_speed", LINEAR_SPEED);

	// ── Sensor fusion parameters (camera intrinsics/extrinsics, drive mode) ──
	ros::param::get("~drive_mode",   drive_mode_);
	ros::param::get("~miss_cnt_max", miss_cnt_max_);
	ros::param::get("~cam/fx",    cam_.fx);
	ros::param::get("~cam/fy",    cam_.fy);
	ros::param::get("~cam/cx",    cam_.cx);
	ros::param::get("~cam/cy",    cam_.cy);
	ros::param::get("~cam/tx",    cam_.tx);
	ros::param::get("~cam/ty",    cam_.ty);
	ros::param::get("~cam/tz",    cam_.tz);
	ros::param::get("~cam/pitch", cam_.pitch);
	ros::param::get("~cam/yaw",   cam_.yaw);
	ros::param::get("~cam/roll",  cam_.roll);
	ROS_INFO("[LaneChanger] drive_mode=%d", drive_mode_);
}

void LaneChanger::clustering(const pcl::PointCloud<PointType>::Ptr cloud_in) {

    // ── Common: Voxel downsampling + PassThrough ROI filter ─────────────
    // Reduce point density and remove points outside the region of interest
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(cloud_in);
    vox.setLeafSize(0.15, 0.15, 0.15);
    vox.filter(*cloud_in);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_min, x_max);
    pass.setFilterLimitsNegative(false);
    pass.filter(*cloud_in);

    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_min, y_max);
    pass.setFilterLimitsNegative(false);
    pass.filter(*cloud_in);

    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min, z_max);
    pass.setFilterLimitsNegative(false);
    pass.filter(*cloud_in);

    sensor_msgs::PointCloud2 filteredOutput;
    pcl::toROSMsg(*cloud_in, filteredOutput);
    filteredOutput.header.frame_id = "map";
    pub_points_.publish(filteredOutput);

    center_.clear();

    // ── INSPECTION MODE ─────────────────────────────────────────────────────
    // Pre-competition vehicle inspection test: the vehicle must stop in front
    // of a traffic cone placed on the track.
    // Problem: traffic cones reflect very few LiDAR points (sometimes just 1~2),
    // so standard clustering often fails to detect them.
    // Solution: if even ONE LiDAR point falls inside a YOLO bounding box,
    // immediately register it as an obstacle — no clustering required.
    // This single-point detection strategy dramatically improves inspection pass rate.
    if (drive_mode_ == 0) {
        if (!is_yolo_ || cloud_in->empty()) return;

        vector<pair<int,int>> pixels;
        vector<int> valid_indices;
        projectLidarToImage(*cloud_in, pixels, valid_indices);

        for (const auto& box : yolo_boxes_) {
            float box_center_u = (box.u1 + box.u2) / 2.0f;

            float min_dist = FLT_MAX;
            PointXYZV best_P;
            bool has_lidar = false;

            for (int k = 0; k < (int)valid_indices.size(); k++) {
                int u = pixels[k].first;
                int v = pixels[k].second;
                if (u >= box.u1 && u <= box.u2 && v >= box.v1 && v <= box.v2) {
                    int ci = valid_indices[k];
                    float px = cloud_in->points[ci].x;
                    float py = cloud_in->points[ci].y;
                    float pz = cloud_in->points[ci].z;
                    float d  = sqrt(px*px + py*py);
                    if (d < min_dist) {
                        min_dist = d;
                        best_P.x = px; best_P.y = py; best_P.z = pz;
                        best_P.dist = d;
                        has_lidar = true;
                    }
                }
            }

            if (has_lidar) {
                if (best_P.x > -2.0f && best_P.x < 0.5f &&
                    best_P.y > -0.85f && best_P.y < 0.85f) continue;
                best_P.width = 0; best_P.height = 0;
                best_P.z_height = 0; best_P.id = -1; best_P.vel = 0;
                center_.emplace_back(best_P);
                printf("\033[1;33m[Inspection]\033[0m Cone in ROI! (u=%.1f) dist=%.2f\n",
                       box_center_u, min_dist);
                break;
            }
        }
    }
    // ── RACING MODE: PointPainting ──────────────────────────────────────────
    // In a race, cones and curbs must NOT be detected as obstacles.
    // Only competing vehicles should be detected.
    // PointPainting: project LiDAR points onto the YOLOv8-seg vehicle mask.
    // Only points landing inside the segmentation mask are kept,
    // then clustered — ensuring vehicle-only detection.
    else {
        if (cloud_in->empty()) return;

        // Project all LiDAR points onto the camera image plane
        vector<pair<int,int>> pixels;
        vector<int> valid_indices;
        projectLidarToImage(*cloud_in, pixels, valid_indices);

        pcl::PointCloud<PointType>::Ptr inbox_cloud(new pcl::PointCloud<PointType>());

        // PointPainting: keep only LiDAR points that land inside the vehicle seg mask
        if (!seg_mask_.empty()) {
            int seg_h = seg_mask_.rows;
            int seg_w = seg_mask_.cols;
            int inbox_cnt = 0;
            for (int k = 0; k < (int)valid_indices.size(); k++) {
                int u = pixels[k].first;
                int v = pixels[k].second;
                if (u >= 0 && u < seg_w && v >= 0 && v < seg_h) {
                    if (seg_mask_.at<uint8_t>(v, u) > 0) {
                        inbox_cloud->points.push_back(cloud_in->points[valid_indices[k]]);
                        inbox_cnt++;
                    }
                }
            }
            printf("[PointPainting] vehicle seg inbox=%d / total=%d\n", inbox_cnt, (int)valid_indices.size());
        }
        else {
            ROS_WARN_THROTTLE(5.0, "[LaneChanger] seg_mask empty, skip clustering");
            return;
        }
        inbox_cloud->width  = inbox_cloud->points.size();
        inbox_cloud->height = 1;
        inbox_cloud->is_dense = true;

        if (inbox_cloud->empty()) return;

        // Euclidean cluster extraction on the filtered (vehicle-only) point cloud
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
        if (!inbox_cloud->empty()) kdtree->setInputCloud(inbox_cloud);

        std::vector<pcl::PointIndices> clusterIndices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(cluster_minsize_);
        ec.setMaxClusterSize(cluster_maxsize_);
        ec.setSearchMethod(kdtree);
        ec.setInputCloud(inbox_cloud);
        ec.extract(clusterIndices);

        clustered_.clear();

        for (vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
            pcl::PointCloud<PointType>::Ptr cluster_cloud(new pcl::PointCloud<PointType>);
            for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
                cluster_cloud->points.emplace_back(inbox_cloud->points[*pit]);
            }
            PointXYZV P;
            pcl::PointXYZ min_pt, max_pt;
            pcl::getMinMax3D(*cluster_cloud, min_pt, max_pt);
            P.width    = max_pt.x - min_pt.x;
            P.height   = max_pt.y - min_pt.y;
            P.z_height = max_pt.z - min_pt.z;
            P.x    = (max_pt.x + min_pt.x) / 2;
            P.y    = (max_pt.y + min_pt.y) / 2;
            P.z    = (max_pt.z + min_pt.z) / 2;
            P.id   = -1;
            P.vel  = 0;
            P.dist = sqrt(pow(P.x, 2) + pow(P.y, 2));

            if (P.x > -2.0 && P.x < 0.5 && P.y > -0.85 && P.y < 0.85) {}
            else {
                center_.emplace_back(P);
                ROS_INFO("[Fusion] Obstacle at (%.2f, %.2f) dist=%.2f", P.x, P.y, P.dist);
            }

            sort(center_.begin(), center_.end(), cmp);
        }
    }
}


bool LaneChanger::cmp(const PointXYZV &v1, const PointXYZV &v2){
	return v1.dist < v2.dist;
} // Sort ascending by distance so center_[0] is always the closest obstacle

void LaneChanger::pointCallback(const sensor_msgs::PointCloud2::ConstPtr &scan) {
	pcl::fromROSMsg(*scan, *msgCloud);
}

void LaneChanger::yoloCallback(const vision_msgs::Detection2DArray::ConstPtr& msg) {
    yolo_boxes_.clear();
    for (const auto& det : msg->detections) {
        double cx = det.bbox.center.x;
        double cy = det.bbox.center.y;
        double hw = det.bbox.size_x / 2.0;
        double hh = det.bbox.size_y / 2.0;
        BBox b;
        b.u1 = cx - hw; b.u2 = cx + hw;
        b.v1 = cy - hh; b.v2 = cy + hh;
        yolo_boxes_.push_back(b);
    }
    is_yolo_ = !yolo_boxes_.empty();
}

void LaneChanger::segMaskCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // Store the latest vehicle segmentation mask (mono8) from YOLOv8-seg
    seg_mask_ = cv::Mat(msg->height, msg->width, CV_8UC1,
                        const_cast<uint8_t*>(msg->data.data())).clone();
}

void LaneChanger::laneMaskCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // Store the latest lane segmentation mask (mono8) from TwinLiteNet
    lane_mask_ = cv::Mat(msg->height, msg->width, CV_8UC1,
                         const_cast<uint8_t*>(msg->data.data())).clone();
}

// extractMyLane: Build a polygon representing the ego vehicle's current lane.
// Scans the TwinLiteNet lane mask from bottom to top.
// Starting from the image horizontal center, finds the first lane pixel
// to the left and right on each row, then connects them into a closed polygon.
// This polygon is used to check whether an obstacle has crossed into our lane.
bool LaneChanger::extractMyLane(std::vector<cv::Point>& my_lane_poly) {
    if (lane_mask_.empty()) return false;

    int h = lane_mask_.rows;
    int w = lane_mask_.cols;
    int cx = w / 2;

    std::vector<cv::Point> left_pts, right_pts;

    // Scan from bottom to top, finding the first lane pixel left and right of center
    for (int y = h - 1; y >= h / 2; y--) {
        // Left side: scan leftward from image center
        for (int x = cx; x >= 0; x--) {
            if (lane_mask_.at<uint8_t>(y, x) > 0) {
                left_pts.push_back(cv::Point(x, y));
                break;
            }
        }
        // Right side: scan rightward from image center
        for (int x = cx; x < w; x++) {
            if (lane_mask_.at<uint8_t>(y, x) > 0) {
                right_pts.push_back(cv::Point(x, y));
                break;
            }
        }
    }

    if (left_pts.size() < 5 || right_pts.size() < 5) return false;

    // Build closed polygon: left pts (top-to-bottom) + right pts (bottom-to-top)
    my_lane_poly.clear();
    for (auto& p : left_pts)  my_lane_poly.push_back(p);
    for (auto it = right_pts.rbegin(); it != right_pts.rend(); ++it)
        my_lane_poly.push_back(*it);

    return true;
}

// checkCutIn: Determine if an obstacle has invaded our lane polygon.
// Counts how many pixels of the vehicle segmentation mask fall inside
// the lane polygon built by extractMyLane().
// If overlap exceeds 50 pixels, a cut-in event is triggered.
// Threshold of 50px prevents false triggers from minor pixel-level noise.
bool LaneChanger::checkCutIn(const std::vector<cv::Point>& my_lane_poly) {
    if (seg_mask_.empty() || my_lane_poly.empty()) return false;

    int h = seg_mask_.rows;
    int w = seg_mask_.cols;

    // Collect all white pixels from the vehicle segmentation mask
    std::vector<cv::Point> vehicle_pts;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            if (seg_mask_.at<uint8_t>(y, x) > 0) {
                vehicle_pts.push_back(cv::Point(x, y));
            }
        }
    }
    if (vehicle_pts.empty()) return false;

    // Count vehicle pixels that fall inside the lane polygon
    int overlap_cnt = 0;
    for (const auto& pt : vehicle_pts) {
        double result = cv::pointPolygonTest(my_lane_poly, cv::Point2f(pt.x, pt.y), false);
        if (result >= 0) {
            overlap_cnt++;
            if (overlap_cnt > 50) return true;  // cut-in confirmed if > 50 pixels overlap
        }
    }
    return false;
}

// projectLidarToImage: Project 3D LiDAR points onto the 2D camera image plane.
// Applies camera extrinsic (pitch, tx, ty, tz) and intrinsic (fx, fy, cx, cy) parameters.
// Points behind the camera (cz <= 0.1) are discarded.
void LaneChanger::projectLidarToImage(
    const pcl::PointCloud<PointType>& cloud,
    vector<pair<int,int>>& pixels,
    vector<int>& valid_indices)
{
    pixels.clear();
    valid_indices.clear();
    double cp = cos(cam_.pitch), sp = sin(cam_.pitch);
    for (int i = 0; i < (int)cloud.points.size(); i++) {
        double lx = cloud.points[i].x;
        double ly = cloud.points[i].y;
        double lz = cloud.points[i].z;
        double cx_cam = -ly + cam_.ty;
        double cy_cam = -lz - cam_.tz;
        double cz_cam =  lx * cp + lz * sp + cam_.tx;
        if (cz_cam <= 0.1) continue;
        int u = static_cast<int>(cam_.fx * cx_cam / cz_cam + cam_.cx);
        int v = static_cast<int>(cam_.fy * cy_cam / cz_cam + cam_.cy);
        pixels.push_back({u, v});
        valid_indices.push_back(i);
    }
}

// state_Callback: Receives driving state info (current lane number, driving state)
void LaneChanger::state_Callback(const waypoint_maker::State::ConstPtr &msg){
	lane_number_ = msg->lane_number; // current GPS-based lane number (0 or 1)
	current_state_ = msg->current_state; // current driving state (e.g. intersection, normal)
	// cout<<"lane_number : "<<lane_number_<< endl;
}

// void LaneChanger::closest_index_Callback(const std_msgs::Int32::ConstPtr &msg){
// 	closest_index = msg->data;
// }

// void LaneChanger::CourseCallback(const std_msgs::Float64::ConstPtr &course_msg){
// 	cur_course_rad_ = course_msg -> data;
// }

// LaneCallback: Receives and stores the reference path (waypoints) to follow
void LaneChanger::LaneCallback(const waypoint_maker::Lane::ConstPtr &lane_msg)
{
	waypoints_.clear();
	W_X.clear();
	W_Y.clear();
	// Clear all existing waypoint data before loading new path
	vector<waypoint_maker::Waypoint>().swap(waypoints_);
	waypoints_ = lane_msg->waypoints; // store incoming waypoints
	waypoints_size_ = waypoints_.size(); // total number of waypoints
	if (waypoints_size_ != 0) is_lane_ = true;
	// cout << "!!!!!!!!!! waypoint loaded !!!!!!!!!!" << endl;
	for(int i=0; i<waypoints_size_; i++){
		W_X.push_back(waypoints_[i].pose.pose.position.x);
		W_Y.push_back(waypoints_[i].pose.pose.position.y);
	} // extract x,y coords into flat vectors for fast computation
}

// OdomCallback: Receives real-time vehicle position and heading from odometry
void LaneChanger::OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
	cur_pose_.header = odom_msg->header; // cur_pose_: stores the vehicle's current global position
	cur_pose_.pose.position = odom_msg->pose.pose.position;
	// Copy timestamp and frame_id; copy global x,y,z position
    
	// Convert quaternion orientation to roll/pitch/yaw using tf2
	tf2::Quaternion q(
		odom_msg->pose.pose.orientation.x,
		odom_msg->pose.pose.orientation.y,
		odom_msg->pose.pose.orientation.z,
		odom_msg->pose.pose.orientation.w
	);
	double roll, pitch, yaw;
	tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
	cur_course_rad_ = yaw; // store yaw as vehicle heading [rad]
	// yaw extracted from quaternion represents the vehicle heading direction

	// cout << "position X: " << cur_pose_.pose.position.x << endl;
	// cout << "position Y: " << cur_pose_.pose.position.y << endl;
}

// calcul_angle: Compute angle [deg] from ego vehicle to a given (x,y) point
float LaneChanger::calcul_angle(float x, float y) {
	float ang;
	ang = atan2f(y, x)*180/M_PI;//from radian to degree
	return ang;
}

// CircleDecision1: Check if any obstacle is within radius r of the EGO LANE path.
// Iterates over all waypoints (x1,y1) and all obstacle positions (x2,y2).
// If distance < r, sets my_line_obs = true (ego lane is blocked).
void LaneChanger::CircleDecision1(vecD x1, vecD y1, vecD x2, vecD y2,float r){	// my line
	is_obs1 = false;
	// cout << "CircleDecision1" << endl;

	if(x1.size() == 0 || x2.size() == 0){
		is_obs1 = false;
		lane_cnt1 = 0;
	} // guard: skip if either path or obstacle data is empty
	
    // Core collision check: compute distance between each path point and each obstacle
	else if(x1.size() != 0 && x2.size() != 0){
		for(int j=0; j<x1.size(); j++){
			for(int i=0; i<x2.size(); i++){
				is_circle1 = sqrt(pow(x1[j]-x2[i], 2) + pow(y1[j]-y2[i], 2)); // Euclidean dist between path point and obstacle
				if(is_circle1 < r) {
					cout << "!!!!!!!!! my line obstacle !!!!!!!!" << endl;
					is_obs1 = true; // obstacle within radius r detected
					obs_index1 = j;
					lane_cnt1 ++;
					break; // early exit once obstacle found
				}
				else if(is_circle1 > r){
					is_obs1 = false;
				}
			}
			if(is_obs1) break;
		}
	}
	if(is_obs1){
		my_line_obs = true; // ego lane is blocked 
		cout << "[Obs] my line obstacle confirmed" << endl;
	}
	else{
		my_line_obs = false; // ego lane is clear
		lane_cnt1 = 0;
	}
}

// CircleDecision2: Check if any obstacle is within radius r of the ADJACENT LANE path.
// Same logic as CircleDecision1, but applied to the offset (neighboring) lane path.
// Sets other_line_obs = true if the adjacent lane is blocked.
void LaneChanger::CircleDecision2(vecD x1, vecD y1, vecD x2, vecD y2,float r){ // adjacent lane
	is_obs2 = false; // reset adjacent lane obstacle flag at start of each call

	if(x1.size() == 0 || x2.size() == 0) {
		is_obs2 = false;
		lane_cnt2 = 0;
	} // x1,y1 = adjacent lane path points; x2,y2 = obstacle global positions

	else if(x1.size() != 0){
		for(int j=0; j<x1.size(); j++){
			for(int i=0; i<x2.size(); i++){
				is_circle2 = sqrt(pow(x1[j]-x2[i], 2) + pow(y1[j]-y2[i], 2));
				if(is_circle2 < r) {
					cout << "!!!!!!!!! other obstacle !!!!!!!!" << endl;
					is_obs2 = true; // adjacent lane is blocked
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
		cout << "[Obs] other obstacle confirmed" << endl;
	}
	else{
		other_line_obs = false;
		lane_cnt2 = 0;
	}
}

// CircleDecision1_c / 2_c: Return-value variants of CircleDecision1/2.
// Unlike the void versions that store results in member variables,
// these return bool directly for immediate conditional use.
bool LaneChanger::CircleDecision1_c(vecD x1, vecD y1, vecD x2, vecD y2,float r){ // return-value variant
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


bool LaneChanger::CircleDecision2_c(vecD x1, vecD y1, vecD x2, vecD y2,float r){ // return-value variant
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

// LaneChage_Decision: Core lane change / ACC decision logic.
// Based on my_line_obs (ego lane blocked) and other_line_obs (adjacent lane blocked):
//   my=false           → keep current lane, no ACC
//   my=true, other=false → change lane (lane_num flips)
//   my=true, other=true  → both blocked, activate ACC (stop/slow behind obstacle)
void LaneChanger::LaneChage_Decision(bool my_obs, bool other_obs){ // lane_num 0: default, 1: avoidance
	// lane_num 0 = default lane
	if(lane_num == 0){
		if(my_obs){
			if(other_obs){
				lane_num = lane_number_; // give up lane change, stay in current lane
				acc_index1 = find_closest_waypoint(global_obs, obs_index1, waypoints_); // find waypoint index closest to obstacle
				cout << "acc_index1 : " << acc_index1 << endl;
				acc_flag = true;
			} // both blocked: activate ACC
			else{
				if(lane_cnt1 > 1){
					lane_cnt1 = 0;
					lane_cnt2 = 0;

					lane_num = 1; // switch to lane 1 to avoid obstacle
					acc_flag = false;	
				}
				else {
					lane_num = 0; // obstacle detected once but not yet confirmed, hold current lane
				}	
			} // ego blocked, adjacent clear: change lane
		}
		else {
			lane_num = 0;
			acc_flag = false;

		} // ego lane clear: keep lane, no ACC
	}
	// lane_num==1: decision to use avoidance lane was already made in a previous cycle
	else if (lane_num == 1){
		// already in lane 1 (avoidance lane), but obstacle detected here too
		if(my_obs){
			if(other_obs){
				lane_num = lane_number_ ;
				acc_index1 = find_closest_waypoint(global_obs, obs_index1, waypoints_);
				cout << "acc_index1 : " << acc_index1 << endl;
				acc_flag = true;
			} // both lanes blocked: ACC
			else{
				if(lane_cnt1 > 1){
					lane_cnt1 = 0;
					lane_cnt2 = 0;

					lane_num = 0; // return to lane 0 to avoid obstacle
					acc_flag = false;
				}
				else {
					lane_num = 1;
				}	
			} // avoidance lane blocked, return to lane 0
		}
		else {
			lane_num = 1;
			acc_flag = false;


		} // ego lane clear: keep current lane, no ACC

	}
	// cout << "lane_num : " << lane_num << endl;
	lidar_topic_msg.lane_num = lane_num; // target lane to follow (0 or 1)
	lidar_topic_msg.ACC_speed = 0; // no using
	lidar_topic_msg.ACC_flag = acc_flag; // true = maintain safe distance behind obstacle
	lidar_topic_msg.ACC_index_gap = acc_index1; // waypoint index of the detected obstacle (virtual stop line)
	pub_lidar_topic.publish(lidar_topic_msg); // publish lane_num, ACC_flag, ACC_index_gap to control module

	std_msgs::Int32 path_msg;
	path_msg.data = lane_num;
	pub_path_number_.publish(path_msg);


}

// only_ACC: ACC-only mode — no lane change, just maintain safe following distance.
// Used in intersections, inspection mode, and cut-in mode.
// Includes a debounce mechanism: ACC flag is held for 7 frames after obstacle
// disappears to prevent flickering caused by brief sensor dropouts.
void LaneChanger::only_ACC(bool my_obs){
	// only_ACC mode
	// Activate ACC only when obstacle is detected in ego lane
	if(my_obs){
		if(lane_cnt1 > 2){
			lane_cnt1 = 0;
			lane_num = lane_number_ ;
			acc_index1 = find_closest_waypoint(global_obs, obs_index1, waypoints_);
			cout << "acc_index1 : " << acc_index1 << endl;
			acc_flag = true;
		}
		acc_hold_cnt_ = 0;  // reset hold counter while obstacle is actively detected
	}
	else {
		if (acc_flag) {
			acc_hold_cnt_++;
			if (acc_hold_cnt_ >= 7) {  // release ACC after 7 frames of no detection (debounce)
				acc_flag = false;
				lane_cnt1 = 0;
				acc_hold_cnt_ = 0;
			}
		} else {
			lane_cnt1 = 0;
			acc_hold_cnt_ = 0;
		}
	}
	// cout << "lane_num : " << lane_num << endl;
	lidar_topic_msg.lane_num = lane_num;
	lidar_topic_msg.ACC_speed = 0; // no using
	lidar_topic_msg.ACC_flag = acc_flag;
	lidar_topic_msg.ACC_index_gap = acc_index1;
	pub_lidar_topic.publish(lidar_topic_msg);

}

// only_ACC_2: Simplified ACC with no debounce. Responds instantly but vulnerable to sensor noise.
void LaneChanger::only_ACC_2(bool my_obs){
	lane_cnt1 = 0;
	lane_cnt2 = 0; // reset counters at every call (no debounce)
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

// Local_to_Global: Convert obstacle positions from vehicle-local frame to global map frame.
// Applies 2D rotation using current vehicle yaw and position.
// +1.04m offset compensates for the LiDAR sensor mounting position on the vehicle.
void LaneChanger::Local_to_Global(vector<PointXYZV> input_points){
	global_obs.clear(); // reset global obstacle list
	double yaw = cur_course_rad_; // vehicle heading [rad] from odometry
	pcl::PointXYZ obs;
	// Apply 2D rotation transform: local (sensor frame) -> global (map frame)
	if(input_points.size() != 0){
		for(int i=0; i<input_points.size(); i++){
			geometry_msgs::Pose pose_global;
		
			pose_global.position.x = cur_pose_.pose.position.x +  ((input_points[i].x+1.04) * cos(yaw)) - ((input_points[i].y) * sin(yaw));
			pose_global.position.y = cur_pose_.pose.position.y +  ((input_points[i].x+1.04) * sin(yaw)) + ((input_points[i].y) * cos(yaw));
			pose_global.position.z = 0;
			// +1.04: forward offset to account for LiDAR mounting position on vehicle
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
    cur_pose_global.header.frame_id = "map";
	lidar_global_pub.publish(cur_pose_global);
}

// find_closest_waypoint: Find the index of the waypoint closest to the detected obstacle.
// This index is sent to the control module as a "virtual stop line" — 
// the vehicle should decelerate to maintain safe distance at this waypoint.
int LaneChanger::find_closest_waypoint(vector<pcl::PointXYZ> obs_points, int index, vector<waypoint_maker::Waypoint> path){
	// obs_points: obstacle positions in global frame; index: obstacle index from CircleDecision; path: reference waypoints
	// cout << "---find_closest_waypoint()---" << endl;
	float between_dist = 0.0;
	int tmp_index = 0;
	// int now_index = closest_index;
	vector<float> all_dist;
	// Iterate through all waypoints and compute distance to the obstacle
	if(obs_points.size() != 0 && path.size() != 0){
		all_dist.clear();

		for(int i=0; i<path.size(); i++){
			between_dist = sqrt(pow(obs_points[index].x-path[i].pose.pose.position.x, 2) + pow(obs_points[index].y-path[i].pose.pose.position.y, 2));
			all_dist.push_back(between_dist);

		} // all distances from obstacle to each waypoint stored in all_dist
		float min_dist = 9999.9;
		cout << "all_dist size : " << all_dist.size() << endl;
		for(int j=0; j < all_dist.size(); j++){
			if(min_dist > all_dist[j]){
				min_dist = all_dist[j];
				tmp_index = j;
			} // find the minimum distance entry
		}
		cout << "tmp_index : " << tmp_index << endl;
	}
	else{
		all_dist.clear(); // reset distance vector
	}
	return tmp_index; // acc_index1 = virtual stop line index, sent to speed control module
}


// calculateNormal: Compute the unit normal vector perpendicular to a path segment.
// Used to laterally offset the reference path when generating adjacent lane candidates.
geometry_msgs::Point LaneChanger::calculateNormal(const waypoint_maker::Waypoint& p1, const waypoint_maker::Waypoint& p2) {
    geometry_msgs::Point normal;
    normal.x = p2.pose.pose.position.y - p1.pose.pose.position.y;
    normal.y = -(p2.pose.pose.position.x - p1.pose.pose.position.x);
    double length = sqrt(pow(p1.pose.pose.position.x - p2.pose.pose.position.x, 2) + pow(p1.pose.pose.position.y - p2.pose.pose.position.y, 2));
	if(length == 0) length = 1;
    normal.x /= length;
    normal.y /= length; // normalize to unit vector
    return normal;
}

// offsetPath: Generate a virtual adjacent lane path by laterally offsetting the reference path.
// Shifts each waypoint by 'offset' meters in the perpendicular direction.
// lane_number_=0: shift left (+offset); lane_number_=1: shift right (-offset)
// The result is used by CircleDecision2 to check if the adjacent lane is clear.
void LaneChanger::offsetPath(const vector<waypoint_maker::Waypoint>& before_path, float offset) {	// lane_num 0 = right, lane_num 1 = left
	offset_path1_x.clear();
	offset_path1_y.clear();
	offset_path1.clear();

	if(before_path.size() != 0){
		if(lane_number_ == 0){ // current lane is 0: shift path to the left
			real_D_OFFSET = offset; // positive offset = shift left
			cout << "real_D_OFFSET : " << real_D_OFFSET << endl;
				int n = before_path.size();
				for (int i = 0; i < n-1; i++) {
					const waypoint_maker::Waypoint& p1 = before_path[i];
					const waypoint_maker::Waypoint& p2 = before_path[(i + 1)];
					geometry_msgs::Point normal = calculateNormal(p1, p2);

					waypoint_maker::Waypoint offsetPoint1;
					offsetPoint1.pose.pose.position.x = p1.pose.pose.position.x + normal.x * real_D_OFFSET; // new waypoint shifted laterally
					offsetPoint1.pose.pose.position.y = p1.pose.pose.position.y + normal.y * real_D_OFFSET;

					offset_path1_x.push_back(offsetPoint1.pose.pose.position.x); // store for CircleDecision2
					offset_path1_y.push_back(offsetPoint1.pose.pose.position.y);

					offset_path1.push_back(offsetPoint1); // accumulate into offset path
				}
				nav_msgs::Path offset_path_msg1;

				offset_path_msg1.header.frame_id = "map";
				offset_path_msg1.poses.resize(offset_path1.size());
                
				// Pack computed offset path into ROS message and publish for visualization
				for (int i = 0; i < offset_path1.size(); i++){
					geometry_msgs::PoseStamped off;
					off.pose.position.x = offset_path1[i].pose.pose.position.x;
					off.pose.position.y = offset_path1[i].pose.pose.position.y;
					offset_path_msg1.poses[i] = off;
				}
				pub_offset_path1.publish(offset_path_msg1);
			}

		else if(lane_number_ == 1){
			real_D_OFFSET = -1*offset; // negative offset = shift right
			cout << "real_D_OFFSET : " << real_D_OFFSET << endl;
			int n = before_path.size();
			for (int i = 0; i < n-1; i++) { // shift path to the right
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
		
		// Publish global reference path for RViz visualization
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

// run(): Main loop executed at 10Hz. Orchestrates all perception and decision logic.
void LaneChanger::run(){
	
	clustering(msgCloud);
	Local_to_Global(center_);

	// ── CUT-IN DETECTION ────────────────────────────────────────────────────
	// Cooldown after lane change: when lane_num changes, set a 5-second cooldown.
	// Prevents false cut-in triggers during lane transitions, where the original
	// obstacle may appear to cross into the new lane from the updated perspective.
	if (lane_num != prev_lane_num_) {
		cut_in_cooldown_ = 50;
		ROS_INFO("[CutIn] lane changed %d->%d, cooldown=50", prev_lane_num_, lane_num);
	} else if (cut_in_cooldown_ > 0) {
		cut_in_cooldown_--;
	}
	prev_lane_num_ = lane_num;

	// Cut-in detection conditions (ALL must be true):
	//   1. other_line_obs=true  → obstacle was already in adjacent lane (approaching from side)
	//   2. my_line_obs=false    → ego lane not blocked (exclude double-block scenario)
	//   3. !center_.empty()     → obstacle is actively tracked (guards against dist=9999 ghost)
	//   4. cut_in_cooldown_==0  → no recent lane change
	if (!cut_in_mode_ && other_line_obs && !my_line_obs
	    && !center_.empty() && cut_in_cooldown_ == 0) {
		std::vector<cv::Point> my_lane_poly;
		if (extractMyLane(my_lane_poly)) {
			if (checkCutIn(my_lane_poly)) {
				ROS_WARN("[CutIn] Cut-in detected! only_ACC mode ON");
				cut_in_mode_ = true;
				cut_in_cnt_  = 0;
			}
		}
	}

	// ── CUT-IN MODE: active ACC until obstacle is sufficiently far ahead ────
	if (cut_in_mode_) {
		cut_in_cnt_++;
		acc_flag = true;

		// Run CircleDecision1 to update my_line_obs during cut-in mode
		float r_dist = 1.0;
		if (current_state_ == 7) r_dist = 1.15;
		else if (current_state_ != 0 && current_state_ != 11) r_dist = 1.1;
		offsetPath(waypoints_, 3.0);
		CircleDecision1(global_x, global_y, W_X, W_Y, r_dist);

		if (my_line_obs) {
			acc_index1 = find_closest_waypoint(global_obs, obs_index1, waypoints_);
		}

		lidar_topic_msg.lane_num      = lane_num;
		lidar_topic_msg.ACC_speed     = 0;
		lidar_topic_msg.ACC_flag      = true;
		lidar_topic_msg.ACC_index_gap = acc_index1;
		pub_lidar_topic.publish(lidar_topic_msg);
		std_msgs::Int32 path_msg;
		path_msg.data = lane_num;
		pub_path_number_.publish(path_msg);

		float nearest_dist = center_.empty() ? 9999.0f : center_[0].dist;
		ROS_INFO("[CutIn] mode ON cnt=%d dist=%.2f my_obs=%d", cut_in_cnt_, nearest_dist, (int)my_line_obs);

		// Exit cut-in mode when obstacle is > 15m ahead — it has merged and moved away
		if (nearest_dist > 15.0f) {
			ROS_INFO("[CutIn] dist > 15m, mode OFF -> resume normal logic");
			cut_in_mode_   = false;
			cut_in_cnt_    = 0;
			acc_flag       = false;
			acc_index1     = 0;
			other_line_obs = false;
		}
		visualize_center(global_obs);
		visualize_center_local(center_);
		global_x.clear();
		global_y.clear();
		return;
	}
	// ──────────────────────────────────────────────────────────

	// ── INSPECTION MODE: ACC-only (no lane change) ─────────────────────────
	if (drive_mode_ == 0) {
		offsetPath(waypoints_, D_OFFSET);
		CircleDecision1(global_x, global_y, W_X, W_Y, 1.8);
		only_ACC(my_line_obs);
		visualize_center(global_obs);
		visualize_center_local(center_);
		global_x.clear();
		global_y.clear();
		return;
	}
	cout << "lidar_lane_num : " << lane_num << endl; // target lane decided by perception logic
	cout << "gps_lane_number_ : " << lane_number_ << endl; // actual current lane from GPS
	cout << "current_state_ : " << current_state_ << endl; // driving state (0=normal, 7=narrow, etc.)

	
	if(current_state_ == 1 || current_state_ == 2 || current_state_ == 3 || current_state_ == 4 || current_state_ == 5 || current_state_ == 6 || current_state_ == 8 || current_state_ == 9 || current_state_ == 10){
		// Intersection / special state: no lane change, ACC only
		offsetPath(waypoints_, D_OFFSET); // generate reference path with D_OFFSET lateral shift
		CircleDecision1(global_x, global_y, W_X, W_Y, 1.8); // check obstacles within 1.8m of path
		only_ACC(my_line_obs); // maintain following distance, no lane change
		
	}
	else {
		// Normal road driving: evaluate lane change vs ACC
		if(lane_num == lane_number_){
			float new_offset = 0.0;
			float r_dist = 0.0;
			if(current_state_ == 0 || current_state_ == 11){
				new_offset = 3.0; // lane width offset [m]
				r_dist = 1.0; // obstacle detection radius [m]
			}
			else if(current_state_ == 7){
				new_offset = 2.7;
				r_dist = 1.15;
			}
			else{
				new_offset = 2.55;
				r_dist = 1.1;
			}

			offsetPath(waypoints_, new_offset); // generate virtual adjacent lane path
			if(lane_number_ == 0){
						
				CircleDecision1(global_x, global_y, W_X, W_Y, r_dist); // check ego lane for obstacles
				CircleDecision2(global_x, global_y, offset_path1_x, offset_path1_y,r_dist); // check adjacent lane (must be clear to change)
				
				
				// CircleDecision1_c(global_x, global_y, W_X, W_Y, 1.9);
				// CircleDecision2_c(global_x, global_y, offset_path1_x, offset_path1_y,1.9);

				// if(is_obs1_c && is_obs2_c){
				// 	only_ACC_2(true);
				// 	cout << "!!!center_ACC!!!" << endl;
				// }
				// else{
				LaneChage_Decision(my_line_obs, other_line_obs); // decide: change lane or ACC 
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
	visualize_center(global_obs); // publish obstacle markers in global frame
	visualize_center_local(center_); // publish obstacle markers in local frame
	global_x.clear(); // reset global obstacle coordinates for next cycle
	global_y.clear();
	
}

// visualize_center: Publish obstacle positions (global frame) as RViz markers
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