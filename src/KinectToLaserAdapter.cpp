
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
#include "opencv2/imgproc/imgproc.hpp"
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

	int pixel_x = (int) (y / _VIEWER_PIXELS_PER_METER_Y) + (m.rows / 2);
	int pixel_y = (int) (x / _VIEWER_PIXELS_PER_METER_X) + 10;

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
	_scan_viewer = Scalar(255, 255, 255);
	_left_obstacles_in_image_viewer = Scalar(255, 255, 255);
	_right_obstacles_in_image_viewer = Scalar(255, 255, 255);
}


void
KinectToLaserAdapter::_clean_scan()
{
	for (int i = 0; i < _scan.size(); i++)
	{
		_scan[i] = _KINECT_MAX_RANGE;
		_intensities[i] = 0;
	}
}


KinectToLaserAdapter::KinectToLaserAdapter()
{
	_left_obstacles_in_image_viewer = Mat(_KINECT_HEIGHT, _KINECT_WIDTH, CV_8UC3);
	_right_obstacles_in_image_viewer = Mat(_KINECT_HEIGHT, _KINECT_WIDTH, CV_8UC3);

	_scan_viewer = Mat(_MAP_VIEWER_HEIGHT, _MAP_VIEWER_WIDTH, CV_8UC3);

	// ********************************************************************************************************
	// INFO ABOUT ANGLES IN THE TF (from http://www.bulletphysics.com/Bullet/BulletFull/classbtQuaternion.html#a8bd5d699377ba585749d325076616ffb)
	// yaw: Angle around Y unless BT_EULER_DEFAULT_ZYX defined then Z
	// pitch: Angle around X unless BT_EULER_DEFAULT_ZYX defined then Y
	// roll: Angle around Z unless BT_EULER_DEFAULT_ZYX defined then X
	//
	// OBS: In the ros reference system, the order of the angles in the Quaternion is PITCH, ROLL, YAW instead
	// of YAW, PITCH, ROLL as the documentation says. Observe also that the meaning of the angles are different
	// from what we are used to in carmen.
	//
	// ********************************************************************************************************

	// 20 degrees = 0,3491 radians
	// 30 degrees = 0.5236 radians
	// 40 degrees = 0,6981 radians
	// 6 degrees ~= 0.1 radians

	// CALIBRATION LEFT KINECT TO BASE_LINK: (x,y,z in meters) = 0 -0.10 0.495 (r,p,y in degrees) = 0 30 -32
	// LeftCamToBaseLink = tf::Transform(tf::Quaternion(0.1236, 0, -0.5585), tf::Vector3(0, -0.10, 0.495));
	// _LeftCamToBaseLink = tf::Transform(tf::Quaternion(0.1236, 0, 0.5585), tf::Vector3(0, -0.10, 0.495));
	_LeftCamToBaseLink = tf::Transform(tf::Quaternion(0.4536, -0.3836, 0.3885), tf::Vector3(0, -0.10, 0.495));

	// CALIBRATION RIGHT KINECT TO BASE_LINK: (x,y,z in meters) = 0 0.105 0.5 (r,p,y in degrees) = 0 30 30
	// RightCamToBaseLink = tf::Transform(tf::Quaternion(0.1236, 0, 0.5236), tf::Vector3(0, -0.105, 0.5));
	// _RightCamToBaseLink = tf::Transform(tf::Quaternion(0.1236, 0, -0.5236), tf::Vector3(0, -0.105, 0.5));
	_RightCamToBaseLink = tf::Transform(tf::Quaternion(0.5836, 0.3736, -0.3436), tf::Vector3(0, -0.105, 0.5));


	_STARTING_ANGLE = from_degrees(-50);
	_ENDING_ANGLE = from_degrees(50);
	_ANGULAR_STEP = from_degrees(1);
	_NUM_RAYS_IN_SCAN = (int) ((_ENDING_ANGLE - _STARTING_ANGLE) / _ANGULAR_STEP);

	_scan = vector<float>(_NUM_RAYS_IN_SCAN);
	_intensities = vector<float>(_NUM_RAYS_IN_SCAN);
}


KinectToLaserAdapter::~KinectToLaserAdapter()
{

}


double
KinectToLaserAdapter::_calulate_obstacle_evidence(double current_vertical_angle, double previous_vertical_angle, double range_current, double range_previous)
{
	double ray_z = range_current * sin(current_vertical_angle);

	if ((range_current >= _KINECT_MAX_RANGE) || (range_current <= _KINECT_MIN_RANGE) || (range_previous >= _KINECT_MAX_RANGE) ||
			(range_previous <= _KINECT_MIN_RANGE) || isnan(range_current) || isnan(range_previous) || (ray_z > 1.5))
	{
		return -1;
	}

	double cos_current = cos(current_vertical_angle);
	double cos_previous = cos(previous_vertical_angle);

	double range_current_on_the_ground = range_current * cos_current;
	double range_previous_on_the_ground = range_previous * cos_previous;

	double next_ray_angle = -normalize_angle(current_vertical_angle - previous_vertical_angle) + atan(_KINECT_Z / range_previous_on_the_ground);

	double measured_difference = range_current_on_the_ground - range_previous_on_the_ground;
	double expected_difference = (_KINECT_Z - range_previous_on_the_ground * tan(next_ray_angle)) / tan(next_ray_angle);

	double obstacle_evidence = 1 - (measured_difference / expected_difference);
	return obstacle_evidence;
}


void
KinectToLaserAdapter::_draw_point_in_image(Mat &n, int i, int j, uchar r, uchar g, uchar b)
{
	for (int k = 0; k < _KINECT_LINE_STEP_BETWEEN_RAYS_TO_DETECT_OBSTACLES; k++)
	{
		if ((i + k) >= 0 && (i + k) < n.rows && j >= 0 && j < n.cols)
		{
			n.data[(i + k) * n.step + 3 * j + 0] = (uchar) b;
			n.data[(i + k) * n.step + 3 * j + 1] = (uchar) g;
			n.data[(i + k) * n.step + 3 * j + 2] = (uchar) r;
		}
	}
}


void
KinectToLaserAdapter::_add_point_to_scan(SphericalPoint point, double obstacle_evidence)
{
	double hangle, radius;
	int angular_section;

	CartesianPoint c = TransformUtil::toCartesian(point);

	hangle = atan2(c.y, c.x);
	radius = sqrt(pow(c.x, 2) + pow(c.y, 2));

	if (radius >= _KINECT_MAX_RANGE || radius < _KINECT_MIN_RANGE)
		radius = _KINECT_MAX_RANGE;

	angular_section = (int) ((hangle + fabs(_STARTING_ANGLE)) / _ANGULAR_STEP);

	if ((angular_section >= 0) && (angular_section < _NUM_RAYS_IN_SCAN))
	{
		if (radius < _scan[angular_section])
		{
			_scan[angular_section] = (float) radius;
			_intensities[angular_section] = (float) obstacle_evidence;
		}
	}
}


void
KinectToLaserAdapter::_build_scan(sensor_msgs::LaserScan &scan, ros::Time message_timestamp)
{
	scan.ranges = _scan;
	scan.intensities.clear();

	scan.range_max = _KINECT_MAX_RANGE;
	scan.range_min = _KINECT_MIN_RANGE;

//	scan.range_max = 10.0;
//	scan.range_min = 0.4;

	scan.angle_max = _ENDING_ANGLE;
	scan.angle_min = _STARTING_ANGLE;
	scan.angle_increment = _ANGULAR_STEP;
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

	corrections.push_back(_LeftCamToBaseLink);
	corrections.push_back(_RightCamToBaseLink);

	_normalized_depth_images.clear();

	for (i = 0; i < depth_images.size(); i++)
		_normalized_depth_images.push_back(_normalize(depth_images[i]));

	_clean_scan();
	_clean_auxiliar_images();

	for (j = 0; j < depth_images[0].cols; j++)
	{
		// angle horizontal. Lembre-se que o Y cresce para a esquerda.
		double horizontal_angle = from_degrees(_INITIAL_ANGLE_HOR - j * _ANGULAR_STEP_X);

		for (i = _KINECT_LINES_TO_IGNORE_TOP; i < (depth_images[0].rows - _KINECT_LINES_TO_IGNORE_BOTTOM - _KINECT_LINE_STEP_BETWEEN_RAYS_TO_DETECT_OBSTACLES); i += _KINECT_VERTICAL_STEP)
		{
			// angle vertical. Lembre-se que o Z cresce para a cima.
			double current_vertical_angle = from_degrees(_INITIAL_ANGLE_VER - i * _ANGULAR_STEP_Y);
			double previous_vertical_angle = from_degrees(_INITIAL_ANGLE_VER - (i + _KINECT_LINE_STEP_BETWEEN_RAYS_TO_DETECT_OBSTACLES) * _ANGULAR_STEP_Y);

			for (k = 0; k < depth_images.size(); k++)
			{
				// ranges
				double current_range = depth_images[k].at<float>(i, j);
				double previous_range = depth_images[k].at<float>(i + _KINECT_LINE_STEP_BETWEEN_RAYS_TO_DETECT_OBSTACLES, j);

				// obstacle evidence
				double obstacle_evidence = _calulate_obstacle_evidence(current_vertical_angle, previous_vertical_angle, current_range, previous_range);

				if (obstacle_evidence == -1 || obstacle_evidence < _OBSTACLE_EVIDENCE_THREASHOLD) // range max or not obstacle
				{
					corrected_spherical_coords = TransformUtil::TransformSpherical(SphericalPoint(current_vertical_angle, horizontal_angle, 10.0), corrections[k]);
					_add_point_to_scan(corrected_spherical_coords, -1);

					if (obstacle_evidence == -1)
					{
						if (k == 0)
							_draw_point_in_image(_left_obstacles_in_image_viewer, i, j, 0, 255, 0);
						else
							_draw_point_in_image(_right_obstacles_in_image_viewer, i, j, 0, 255, 0);
					}
					else
					{
						if (k == 0)
							_draw_point_in_image(_left_obstacles_in_image_viewer, i, j, 0, 0, 255);
						else
							_draw_point_in_image(_right_obstacles_in_image_viewer, i, j, 0, 0, 255);
					}
				}
				else
				{
					corrected_spherical_coords = TransformUtil::TransformSpherical(SphericalPoint(current_vertical_angle, horizontal_angle, current_range), corrections[k]);

					_draw_point_in_the_map(_scan_viewer, corrected_spherical_coords, obstacle_evidence);

					if (k == 0)
						_draw_point_in_image(_left_obstacles_in_image_viewer, i, j, 255, 0, 0);
					else
						_draw_point_in_image(_right_obstacles_in_image_viewer, i, j, 255, 0, 0);

					_add_point_to_scan(corrected_spherical_coords, obstacle_evidence);
				}
			}
		}
	}

	_build_scan(scan, left_cam_msg->header.stamp);

	return scan;
}


void
KinectToLaserAdapter::view() const
{
	Mat mini = Mat(_left_obstacles_in_image_viewer.rows / 2, _left_obstacles_in_image_viewer.cols / 2, _left_obstacles_in_image_viewer.type());

	resize(_left_obstacles_in_image_viewer, mini, mini.size());
	imshow("L", mini);

	resize(_right_obstacles_in_image_viewer, mini, mini.size());
	imshow("R", mini);

	resize(_scan_viewer, mini, mini.size());
	imshow("scan", mini);

//	char name[32];
//	for (int i = 0; i < normalized_depth_images.size(); i++)
//	{
//		sprintf(name, "depth_%d", i);
//		imshow(name, normalized_depth_images[i]);
//	}

	waitKey(1);
}
