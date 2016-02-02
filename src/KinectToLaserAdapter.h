
#ifndef __KINECT_TO_LASER_ADAPTER_H_
#define __KINECT_TO_LASER_ADAPTER_H_

#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "opencv2/highgui/highgui.hpp"
#include "LinearAlgebraUtils.h"

using namespace cv;

class KinectToLaserAdapter
{
	public:

		KinectToLaserAdapter();
		~KinectToLaserAdapter();

		sensor_msgs::LaserScan translate(const sensor_msgs::Image::ConstPtr& left_cam_msg, const sensor_msgs::Image::ConstPtr& right_cam_msg);
		void view() const;

	protected:

		Mat _scan_viewer, _left_obstacles_in_image_viewer, _right_obstacles_in_image_viewer;
		vector<Mat> _normalized_depth_images;
		vector<pair<double, SphericalPoint> > _scan_candidates_and_obstacle_evidence;
		vector<float> _scan, _intensities; // i am storing the obstacle evidence in the intensities

		// initialized in the constructor. TODO: move the calibration values to a parameters file.
		tf::Transform _LeftCamToBaseLink;
		tf::Transform _RightCamToBaseLink;

		double _STARTING_ANGLE;
		double _ENDING_ANGLE;
		double _ANGULAR_STEP;
		int _NUM_RAYS_IN_SCAN;

		static const double _KINECT_HFV = 57.0; // horizontal field of view (degrees)
		static const double _KINECT_VFV = 43.0; // vertical field of view (degrees)

		static const double _KINECT_WIDTH = 640.0; // width of the kinect image
		static const double _KINECT_HEIGHT = 480.0; // height of the kinect image

		static const double _KINECT_CX = 640.0 / 2.0; // x-coord of the center of the image
		static const double _KINECT_CY = 480.0 / 2.0; // y-coord of the center of the image

		static const int _KINECT_LINES_TO_IGNORE_TOP = 0.0;
		static const int _KINECT_LINES_TO_IGNORE_BOTTOM = 0.0;

		static const int _KINECT_VERTICAL_STEP = 4; // for subsampling
		static const int _KINECT_LINE_STEP_BETWEEN_RAYS_TO_DETECT_OBSTACLES = 10; // the number of lines to jump when trying to detect obstacles
		static const double _OBSTACLE_EVIDENCE_THREASHOLD = 0.8; // threashold to assume a evidence is an obstacle

		static const double _KINECT_Z = 0.53; // height of the sensor in relation to the base

		static const double _KINECT_MAX_RANGE = 4.5; // max range in kinect depth to consider
		static const double _KINECT_MIN_RANGE = 0.4; // min range in kinect depth to consider

		static const double _VIEWER_PIXELS_PER_METER_X = 0.01;
		static const double _VIEWER_PIXELS_PER_METER_Y = 0.01;

		// Note: we assumed that the center of the image is the position of the angle 0 (both horizontal and vertical)
		static const double _INITIAL_ANGLE_HOR = (57.0 / 2.0); // initial angle x = kinect hfov / 2
		static const double _INITIAL_ANGLE_VER = (43.0 / 2.0); // initial angle y = kinect vfov / 2
		static const double _ANGULAR_STEP_X = 57.0 / 640.0; // angular step in x = kinect hfov / image_size
		static const double _ANGULAR_STEP_Y = 43.0 / 480.0; // angular step in y = kinect vfov / image_size

		static const int _MAP_VIEWER_HEIGHT = 600;
		static const int _MAP_VIEWER_WIDTH = 600;

		double _calulate_obstacle_evidence(double current_vertical_angle, double previous_vertical_angle, double range_current, double range_previous);
		void _measure_kinect_message_rate(const sensor_msgs::Image::ConstPtr& msg);
		Mat _msg_to_opencv(const sensor_msgs::Image::ConstPtr& cam_msg);
		Mat _normalize(const Mat &m);

		void _clean_scan();
		void _clean_auxiliar_images();

		static void _draw_point_in_image(Mat &n, int i, int j, uchar r, uchar g, uchar b);
		static void _draw_point_in_the_map(Mat &m, SphericalPoint &s, double evidence);

		void _add_point_to_scan(SphericalPoint point, double obstacle_evidence);
		void _build_scan(sensor_msgs::LaserScan &scan, ros::Time message_timestamp);
};

#endif
