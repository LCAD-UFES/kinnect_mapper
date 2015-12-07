
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

		Mat scan_viewer, obstacles_in_image_viewer;
		vector<Mat> normalized_depth_images;
		vector<pair<double, SphericalPoint> > scan_candidates_and_obstacle_evidence;

		// initialized in the constructor. TODO: move the calibration values to a parameters file.
		tf::Transform LeftCamToBaseLink;
		tf::Transform RightCamToBaseLink;

		static const double KINECT_HFV = 57.0; // horizontal field of view (degrees)
		static const double KINECT_VFV = 43.0; // vertical field of view (degrees)

		static const double KINECT_WIDTH = 640.0; // width of the kinect image
		static const double KINECT_HEIGHT = 480.0; // height of the kinect image

		static const double KINECT_CX = 640.0 / 2.0; // x-coord of the center of the image
		static const double KINECT_CY = 480.0 / 2.0; // y-coord of the center of the image

		static const int KINECT_LINES_TO_IGNORE_TOP = 0.0;
		static const int KINECT_LINES_TO_IGNORE_BOTTOM = 0.0;

		static const int KINECT_LINE_STEP_BETWEEN_RAYS_TO_DETECT_OBSTACLES = 10; // the number of lines to jump when trying to detect obstacles
		static const double OBSTACLE_EVIDENCE_THREASHOLD = 0.8; // threashold to assume a evidence is an obstacle

		static const double KINECT_Z = 0.53; // height of the sensor in relation to the base

		static const double KINECT_MAX_RANGE = 4.5; // max range in kinect depth to consider
		static const double KINECT_MIN_RANGE = 0.4; // min range in kinect depth to consider

		static const double VIEWER_PIXELS_PER_METER_X = 0.01;
		static const double VIEWER_PIXELS_PER_METER_Y = 0.01;

		// Note: we assumed that the center of the image is the position of the angle 0 (both horizontal and vertical)
		static const double INITIAL_ANGLE_HOR = (57.0 / 2.0); // initial angle x
		static const double INITIAL_ANGLE_VER = (43.0 / 2.0); // initial angle y
		static const double ANGULAR_STEP_X = 57.0 / 640.0; // angular step in x
		static const double ANGULAR_STEP_Y = 43.0 / 480.0; // angular step in y

		static const double LEFT_CAM_PITCH = -0.60650192; // -34.75 degrees
		static const double RIGHT_CAM_PITCH = -0.541052; //-31.0 degrees

		static const int MAP_VIEWER_HEIGHT = 600;
		static const int MAP_VIEWER_WIDTH = 600;

		double _calulate_obstacle_evidence(double current_vertical_angle, double previous_vertical_angle, double range_current, double range_previous);
		void _measure_kinect_message_rate(const sensor_msgs::Image::ConstPtr& msg);
		void _add_point_to_scan_candidates(SphericalPoint corrected_spherical_coords, double obstacle_evidence);
		Mat _msg_to_opencv(const sensor_msgs::Image::ConstPtr& cam_msg);
		Mat _normalize(const Mat &m);

		void _clean_auxiliar_images();

		static void _draw_point_in_image(Mat &n, int i, int j, uchar r, uchar g, uchar b);
		static void _draw_point_in_the_map(Mat &m, SphericalPoint &s, double evidence);
		static void _build_scan_from_candidates(sensor_msgs::LaserScan &scan,
			vector<pair<double, SphericalPoint> > &candidates, ros::Time message_timestamp);

		class SortByAngle
		{
			public:
				bool operator()(const pair<double, SphericalPoint> &a, const pair<double, SphericalPoint> &b)
				{
					return a.second.hangle < b.second.hangle;
				}
		};
};

#endif
