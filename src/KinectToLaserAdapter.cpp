

#include <map>
#include <cmath>
#include <string>
#include <cstring>
#include <iostream>
#include <algorithm>
#include "ros/ros.h"
#include "angles/angles.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "KinectToLaserAdapter.h"
#include "sensor_msgs/LaserScan.h"
#include "opencv2/highgui/highgui.hpp"
#include "sensor_msgs/image_encodings.h" // CHECK IF IT IS NECESSARY!!!!
#include "LinearAlgebraUtils.h"

using namespace cv;
using namespace std;
using namespace angles;

/**
 * CALIBRATION INFO:
 * LEFT KINECT IN RELATION TO RIGHT: 0 0.245 0 -20 13 40 (x,y,z,roll,p,y) em metros e graus
 * LEFT KINECT TO BASE_LINK: 0 -0.10 0.495 0 30 -32
 * RIGHT KINECT TO BASE_LINK: 0 0.105 0.5 0 30 30
 */

//void
//KinectToLaserAdapter::_transform_polar_coordinates_to_cartesian_coordinates(double radius, double angle, double *x, double *y)
//{
//	*x = radius * cos(angle);
//	*y = radius * sin(angle);
//}
//
//
//void
//KinectToLaserAdapter::_transform_cartesian_coordinates_to_polar_coordinates(double x, double y, double *radius, double *angle)
//{
//	*angle = atan2(y, x);
//	*radius = sqrt(pow(x, 2) + pow(y, 2));
//}


void
KinectToLaserAdapter::_measure_kinect_message_rate(const sensor_msgs::Image::ConstPtr& msg)
{
	static map<string, int> num_messages_per_second;
	static map<string, int> time_last_message;

	if (num_messages_per_second.find(msg->header.frame_id) == num_messages_per_second.end())
	{
		num_messages_per_second[msg->header.frame_id] = 1;
		time_last_message[msg->header.frame_id] = 0;
	}

	if (ros::Time::now().toSec() - time_last_message[msg->header.frame_id] > 1.0)
	{
		ROS_INFO("Message (%s) frequency: %d", msg->header.frame_id.c_str(), num_messages_per_second[msg->header.frame_id]);
		time_last_message[msg->header.frame_id] = ros::Time::now().toSec();
		num_messages_per_second[msg->header.frame_id] = 1;
	}

	num_messages_per_second[msg->header.frame_id]++;
}


void
KinectToLaserAdapter::_draw_point_in_the_map(Mat &m, SphericalPoint &s, double evidence)
{
	int i;
	double x;
	double y;

	// _transform_polar_coordinates_to_cartesian_coordinates(range, angle, &x, &y);
	CartesianPoint c = TransformUtil::toCartesian(s);

	x = c.x;
	y = c.y;

	int pixel_x = (int) (y / VIEWER_PIXELS_PER_METER_Y) + (m.rows / 2);
	int pixel_y = (int) (x / VIEWER_PIXELS_PER_METER_X) + 10;

	// As operacoes estao invertidas de proposito para melhorar a visulizacao
	pixel_y = m.rows - pixel_y - 1;
	pixel_x = m.cols - pixel_x - 1;

	// quanto mais azul menor a chance de ser obstaculo e
	// quanto mais vermelhor maior a chance de ser obstaculo.
	if (evidence > 1) evidence = 1;

	uchar blue = 0; // (uchar) ((1.0 - evidence) * 255);
	uchar red = (uchar) (evidence * 254.0);

	if (pixel_x < 0 || pixel_x >= m.cols || pixel_y < 0 || pixel_y >= m.rows)
	{
		// DEBUG:
		// printf("** px: %d py: %d\n", pixel_x, pixel_y);
		return;
	}

	m.data[pixel_y * m.step + 3 * pixel_x + 0] = blue;
	m.data[pixel_y * m.step + 3 * pixel_x + 1] = 0;
	m.data[pixel_y * m.step + 3 * pixel_x + 2] = red;
}


Mat
KinectToLaserAdapter::_msg_to_opencv(const sensor_msgs::Image::ConstPtr& cam_msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(cam_msg, sensor_msgs::image_encodings::TYPE_32FC1);
	return cv_ptr->image;
}


Mat
KinectToLaserAdapter::_normalize(const Mat &m)
{
	Mat out;
	double max_val;

	cv::minMaxIdx(m, 0, &max_val);
	out = m / max_val;

	return out;
}


void
KinectToLaserAdapter::_clean_auxiliar_images()
{
	scan_viewer = Scalar(255, 255, 255);
	obstacles_in_image_viewer = Scalar(255, 255, 255);
}


KinectToLaserAdapter::KinectToLaserAdapter()
{
	obstacles_in_image_viewer = Mat(KINECT_HEIGHT, KINECT_WIDTH, CV_8UC3);
	scan_viewer = Mat(MAP_VIEWER_HEIGHT, MAP_VIEWER_WIDTH, CV_8UC3);

	// ********************************************************************************************************
	// INFO ABOUT ANGLES IN THE TF (from http://www.bulletphysics.com/Bullet/BulletFull/classbtQuaternion.html#a8bd5d699377ba585749d325076616ffb)
	// yaw: Angle around Y unless BT_EULER_DEFAULT_ZYX defined then Z
	// pitch: Angle around X unless BT_EULER_DEFAULT_ZYX defined then Y
	// roll: Angle around Z unless BT_EULER_DEFAULT_ZYX defined then X
	//
	// OBS: In the ros reference system, the order of the angles in the Quaternion is PITCH, ROLL, YAW instead
	// of YAW, PITCH, ROLL as the documentation says.
	//
	// ********************************************************************************************************

	// 20 degrees = 0,3491 radians
	// 30 degrees = 0.5236 radians
	// 40 degrees = 0,6981 radians

	// CALIBRATION LEFT KINECT TO BASE_LINK: (x,y,z in meters) = 0 -0.10 0.495 (r,p,y in degrees) = 0 30 -32
	LeftCamToBaseLink = tf::Transform(tf::Quaternion(0.1236, 0, -0.6585), tf::Vector3(0, -0.10, 0.495));
	// CALIBRATION RIGHT KINECT TO BASE_LINK: (x,y,z in meters) = 0 0.105 0.5 (r,p,y in degrees) = 0 30 30
	RightCamToBaseLink = tf::Transform(tf::Quaternion(0.1236, 0, 0.5236), tf::Vector3(0, -0.10, 0.495));
}


KinectToLaserAdapter::~KinectToLaserAdapter()
{

}


double
KinectToLaserAdapter::_calulate_obstacle_evidence(double current_vertical_angle, double previous_vertical_angle, double range_current, double range_previous)
{
	double ray_z = range_current * sin(current_vertical_angle);

	if ((range_current >= KINECT_MAX_RANGE) || (range_current <= KINECT_MIN_RANGE) || (range_previous >= KINECT_MAX_RANGE) ||
			(range_previous <= KINECT_MIN_RANGE) || isnan(range_current) || isnan(range_previous) || (ray_z > 1.5))
	{
		return -1;
	}

	double cos_current = cos(current_vertical_angle);
	double cos_previous = cos(previous_vertical_angle);

	double range_current_on_the_ground = range_current * cos_current;
	double range_previous_on_the_ground = range_previous * cos_previous;

	double next_ray_angle = -normalize_angle(current_vertical_angle - previous_vertical_angle) + atan(KINECT_Z / range_previous_on_the_ground);

	double measured_difference = range_current_on_the_ground - range_previous_on_the_ground;
	double expected_difference = (KINECT_Z - range_previous_on_the_ground * tan(next_ray_angle)) / tan(next_ray_angle);

	double obstacle_evidence = 1 - (measured_difference / expected_difference);
	return obstacle_evidence;
}


void
KinectToLaserAdapter::_draw_point_in_image(Mat &n, int i, int j, uchar r, uchar g, uchar b)
{
	n.data[i * n.step + 3 * j + 0] = (uchar) b;
	n.data[i * n.step + 3 * j + 1] = (uchar) g;
	n.data[i * n.step + 3 * j + 2] = (uchar) r;
}


void
KinectToLaserAdapter::_add_point_to_scan_candidates(SphericalPoint point, double obstacle_evidence)
{
	SphericalPoint projection_2d;
	CartesianPoint c = TransformUtil::toCartesian(point);

	projection_2d.hangle = atan2(c.y, c.x);
	projection_2d.radius = sqrt(pow(c.x, 2) + pow(c.y, 2));
	projection_2d.vangle = 0;

	scan_candidates_and_obstacle_evidence.push_back(pair<double, SphericalPoint>(obstacle_evidence, projection_2d));
}


void
KinectToLaserAdapter::_build_scan_from_candidates(sensor_msgs::LaserScan &scan, vector<pair<double, SphericalPoint> > &candidates, ros::Time message_timestamp)
{
	scan.ranges.clear();
	scan.intensities.clear();

	scan.range_max = 10.0;
	scan.range_min = 0.4;

	sort(candidates.begin(), candidates.end(), SortByAngle());

	double min_angle = DBL_MAX, max_angle = -DBL_MAX;
	double min_range_in_section;
	double avg_range;
	double last_angle;
	int selected_ray;
	double nranges;

	last_angle = candidates[0].second.hangle;
	min_range_in_section = DBL_MAX;
	avg_range = 0; nranges = 0;
	selected_ray = 0;

	for (int i = 0; i < candidates.size(); i++)
	{
		if (candidates[i].second.hangle < min_angle)
			min_angle = candidates[i].second.hangle;

		if (candidates[i].second.hangle > max_angle)
			max_angle = candidates[i].second.hangle;

		if (candidates[i].first > OBSTACLE_EVIDENCE_THREASHOLD && candidates[i].first != -1)
		{
			avg_range += candidates[i].second.radius;
			nranges += 1;
		}

		// add a new ray every 0.5 degrees
		if ((candidates[i].second.hangle - last_angle) > from_degrees(0.5))
		{
			if (nranges == 0)
				scan.ranges.push_back(10.0);
			else
			{
				avg_range /= nranges;
				scan.ranges.push_back(avg_range);
			}

			scan.intensities.push_back(0);

			last_angle = candidates[i].second.hangle;
			min_range_in_section = DBL_MAX;
			avg_range = 0; nranges = 0;
		}
	}

	scan.angle_max = max_angle;
	scan.angle_min = min_angle;
	scan.angle_increment = from_degrees(0.5);
	scan.time_increment = 0; // CHECK IF IT IS NECESSARY
	scan.scan_time = 0; // CHECK IF IT IS NECESSARY
	scan.header.stamp = message_timestamp;
	scan.header.frame_id = "scan";
}


sensor_msgs::LaserScan
KinectToLaserAdapter::translate(const sensor_msgs::Image::ConstPtr& left_cam_msg, const sensor_msgs::Image::ConstPtr& right_cam_msg)
{
	int i, j, k;
	double obstacle_evidence;
	sensor_msgs::LaserScan scan;
	SphericalPoint corrected_spherical_coords;

	vector<Mat> depth_images;
	vector<tf::Transform> corrections;

	/*
	 * ********************************************************************************************************
	 * IMPORTANT NOTE: To add more kinects, add their depth images in the depth_images vector, and
	 * add the correction transform in the vector of transforms called corrections (check the code below).
	 * ********************************************************************************************************
	 */

	depth_images.push_back(_msg_to_opencv(left_cam_msg));
	depth_images.push_back(_msg_to_opencv(right_cam_msg));

	corrections.push_back(LeftCamToBaseLink);
	corrections.push_back(RightCamToBaseLink);

	normalized_depth_images.clear();

	for (i = 0; i < depth_images.size(); i++)
		normalized_depth_images.push_back(_normalize(depth_images[i]));

	_clean_auxiliar_images();

	for (j = 0; j < depth_images[0].cols; j++)
	{
		// angle horizontal. Lembre-se que o Y cresce para a esquerda.
		double horizontal_angle = from_degrees(INITIAL_ANGLE_HOR - j * ANGULAR_STEP_X);

		for (i = KINECT_LINES_TO_IGNORE_TOP; i < (depth_images[0].rows - KINECT_LINES_TO_IGNORE_BOTTOM - KINECT_LINE_STEP_BETWEEN_RAYS_TO_DETECT_OBSTACLES); i++)
		{
			// angle vertical. Lembre-se que o Z cresce para a cima.
			double current_vertical_angle = from_degrees(INITIAL_ANGLE_VER - i * ANGULAR_STEP_Y);
			double previous_vertical_angle = from_degrees(INITIAL_ANGLE_VER - (i + KINECT_LINE_STEP_BETWEEN_RAYS_TO_DETECT_OBSTACLES) * ANGULAR_STEP_Y);

			for (k = 0; k < depth_images.size(); k++)
			{
				// ranges
				double current_range = depth_images[k].at<float>(i, j);
				double previous_range = depth_images[k].at<float>(i + KINECT_LINE_STEP_BETWEEN_RAYS_TO_DETECT_OBSTACLES, j) ;

				// obstacle evidence
				double obstacle_evidence = _calulate_obstacle_evidence(current_vertical_angle, previous_vertical_angle, current_range, previous_range);

				if (obstacle_evidence == -1 || obstacle_evidence < OBSTACLE_EVIDENCE_THREASHOLD) // range max or not obstacle
				{
					corrected_spherical_coords = TransformUtil::TransformSpherical(SphericalPoint(current_vertical_angle, horizontal_angle, 10.0), corrections[k]);
					_add_point_to_scan_candidates(corrected_spherical_coords, -1);

					if (obstacle_evidence < OBSTACLE_EVIDENCE_THREASHOLD)
						_draw_point_in_image(obstacles_in_image_viewer, i, j, 0, 0, 255);
					else
						_draw_point_in_image(obstacles_in_image_viewer, i, j, 0, 255, 0);
				}
				else
				{
					corrected_spherical_coords = TransformUtil::TransformSpherical(SphericalPoint(current_vertical_angle, horizontal_angle, current_range), corrections[k]);

					_draw_point_in_the_map(scan_viewer, corrected_spherical_coords, obstacle_evidence);
					_draw_point_in_image(obstacles_in_image_viewer, i, j, 255, 0, 0);
					_add_point_to_scan_candidates(corrected_spherical_coords, obstacle_evidence);
				}
			}
		}
	}

	_build_scan_from_candidates(scan, scan_candidates_and_obstacle_evidence, left_cam_msg->header.stamp);
	return scan;
}


void
KinectToLaserAdapter::view() const
{
	static int first = 1;

	// in the first call of the method, the windows are created and moved to a default position.
	if (first)
	{
//		imshow("kmapper", scan_viewer);
//		imshow("obstacles", obstacles_in_image_viewer);
//
//		namedWindow("kmapper");
//		namedWindow("obstacles");
//
//		// TODO: essas janelas sao criadas dinamicamente no view.
//		namedWindow("depth_0");
//		namedWindow("depth_1");
//		moveWindow(const string& winname, int x, int y);
	}

	char name[32];
	imshow("kmapper", scan_viewer);
	imshow("obstacles", obstacles_in_image_viewer);

	for (int i = 0; i < normalized_depth_images.size(); i++)
	{
		sprintf(name, "depth_%d", i);
		imshow(name, normalized_depth_images[i]);
	}

	waitKey(1);
}
