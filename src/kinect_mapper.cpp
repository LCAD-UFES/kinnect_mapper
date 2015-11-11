#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/highgui/highgui.hpp"
#include "angles/angles.h" // from ros-indigo-geometry
#include <map>

using namespace cv;
using namespace std;
using namespace angles;

/**
 * INFORMACOES IMPORTANTISSIMAS:
 * - A mensagem de depth do ros usa nan p/ representar oclusoes!
 * - P/ acessar os elementos da imagem de depth eh necessario fazer img.at<float>(i, j) pq isso pode retornar nan. O acesso com img.data[i * step + j] nao funciona!
 * - Aparentemente a mensagem de depth retorna a profundidade em metros. P/ mais info veja a documentacao em http://www.ros.org/reps/rep-0118.html.
 */

// *********************************************************
// PARAMETERS OF THE PACKAGE
// *********************************************************

double KINECT_HFV = 57; // horizontal field of view (degrees)
double KINECT_VFV = 43; // vertical field of view (degrees)

double KINECT_WIDTH = 640.0; // width of the kinect image
double KINECT_HEIGHT = 480.0; // height of the kinect image

double KINECT_CX = KINECT_WIDTH / 2.0; // x-coord of the center of the image
double KINECT_CY = KINECT_HEIGHT / 2.0; // y-coord of the center of the image

int KINECT_LINES_TO_IGNORE_TOP = 120;
int KINECT_LINES_TO_IGNORE_BOTTOM = 0;

int KINECT_LINE_STEP_BETWEEN_RAYS_TO_DETECT_OBSTACLES = 10; // the number of lines to jump when trying to detect obstacles
double OBSTACLE_EVIDENCE_THREASHOLD = 0.3; // threashold to assume a evidence is an obstacle

double KINECT_Z = 1.2; // height of the sensor in relation to the base

double KINECT_MAX_RANGE = 4.5; // max range in kinect depth to consider
double KINECT_MIN_RANGE = 0.4; // min range in kinect depth to consider

double VIEWER_PIXELS_PER_METER_X = 0.01;
double VIEWER_PIXELS_PER_METER_Y = 0.01;


void
transform_polar_coordinates_to_cartesian_coordinates(double radius, double angle, double *x, double *y)
{
	*x = radius * cos(angle);
	*y = radius * sin(angle);
}


void
transform_cartesian_coordinates_to_polar_coordinates(double x, double y, double *radius, double *angle)
{
	*angle = atan2(y, x);
	*radius = sqrt(pow(x, 2) + pow(y, 2));
}


void
measure_kinect_message_rate(const sensor_msgs::Image::ConstPtr& msg)
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
draw_point(Mat &m, double range, double angle, double evidence)
{
	int i;
	double x;
	double y;

	transform_polar_coordinates_to_cartesian_coordinates(range, angle, &x, &y);

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


void
convert_to_laser_scan(const sensor_msgs::Image::ConstPtr& msg)
{
	double max_val;

	// convert the message to opencv
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    cv::minMaxIdx(cv_ptr->image, 0, &max_val);
    Mat o = cv_ptr->image / max_val;

	Mat n = Mat(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC3);
	Mat m = Mat(600, 600, CV_8UC3);

	m = Scalar(255, 255, 255);
	n = Scalar(255, 255, 255);

	double oe; // obstacle evidence

	// Note: we assumed that the center of the image is the position of the angle 0 (both horizontal and vertical)
	double iax = -(KINECT_HFV / 2.0); // initial angle x
	double iay = -(KINECT_VFV / 2.0); // initial angle y

	double dx = KINECT_HFV / (double) cv_ptr->image.cols; // angular step in x
	double dy = KINECT_VFV / (double) cv_ptr->image.rows; // angular step in y

	for (int j = 0; j < cv_ptr->image.cols; j++)
	{
		double ax = from_degrees(iax + j * dx); // angle horizontal

		for (int i = KINECT_LINES_TO_IGNORE_TOP; i < (cv_ptr->image.rows - KINECT_LINES_TO_IGNORE_BOTTOM - KINECT_LINE_STEP_BETWEEN_RAYS_TO_DETECT_OBSTACLES); i++)
		{
			double current_vertical_angle = from_degrees(iay + i * dy); // angle vertical
			double previous_vertical_angle = from_degrees(iay + (i + KINECT_LINE_STEP_BETWEEN_RAYS_TO_DETECT_OBSTACLES) * dy); // angle of the previous ray (we sum 1 because the 0 is in the top of the image) <- Check!!!

			double range_current = cv_ptr->image.at<float>(i, j) ;
			double range_previous = cv_ptr->image.at<float>(i + KINECT_LINE_STEP_BETWEEN_RAYS_TO_DETECT_OBSTACLES, j) ;

			double ray_z = range_current * sin(current_vertical_angle);

			// TODO: checar se essa eh a melhor forma de tratar max_range
			if ((range_current >= KINECT_MAX_RANGE) || (range_current <= KINECT_MIN_RANGE) ||
				(range_previous >= KINECT_MAX_RANGE) || (range_previous <= KINECT_MIN_RANGE) ||
				isnan(range_current) || isnan(range_previous) || (ray_z > 1.0))
			{
				n.data[i * n.step + 3 * j + 0] = (uchar) 0;
				n.data[i * n.step + 3 * j + 1] = (uchar) 255;
				n.data[i * n.step + 3 * j + 2] = (uchar) 0;

				continue;
			}
			// DEBUG:
//			else
//			{
//				printf("image: %d %d %d %d\n", cv_ptr->image.rows, cv_ptr->image.cols, cv_ptr->image.channels(), cv_ptr->image.type());
//				printf("%d %d curr %lf prev %lf\n", i, j, range_current, range_previous);
//				char c = getchar();
//				if (c == 'q') exit(0);
//			}

			double cos_current = cos(current_vertical_angle);
			double cos_previous = cos(previous_vertical_angle);

			double range_current_on_the_ground = range_current * cos_current;
			double range_previous_on_the_ground = range_previous * cos_previous;

			double next_ray_angle = -normalize_angle(current_vertical_angle - previous_vertical_angle) + atan(KINECT_Z / range_previous_on_the_ground);

			double measured_difference = range_current_on_the_ground - range_previous_on_the_ground;
			double expected_difference = (KINECT_Z - range_previous_on_the_ground * tan(next_ray_angle)) / tan(next_ray_angle);

			double obstacle_evidence = measured_difference / expected_difference;

			if (obstacle_evidence > OBSTACLE_EVIDENCE_THREASHOLD)
				draw_point(m, range_current_on_the_ground, ax, obstacle_evidence);

			if (obstacle_evidence > OBSTACLE_EVIDENCE_THREASHOLD)
			{
				n.data[i * n.step + 3 * j + 0] = (uchar) 0;
				n.data[i * n.step + 3 * j + 2] = (uchar) 255;
			}
			else
			{
				n.data[i * n.step + 3 * j + 0] = (uchar) 255;
				n.data[i * n.step + 3 * j + 2] = (uchar) 0;
			}

			n.data[i * n.step + 3 * j + 1] = (uchar) 0;
		}
	}

	imshow((string("kmapper") + "_" + msg->header.frame_id).c_str(), m);
	imshow((string("obstacles") + "_" + msg->header.frame_id).c_str(), n);
	//imshow("depth", o);

	waitKey(1);
}


void
kinnect_callback(const sensor_msgs::Image::ConstPtr& msg)
{
	//measure_kinect_message_rate(msg);
	convert_to_laser_scan(msg);
}


int
main(int argc, char **argv)
{
	printf("Start...\n");

	ros::init(argc, argv, "kinnect_mapper");
	ros::NodeHandle n;

	// queue = 1 will drop a lot of messages (good for online running), while queue = 1000 is good for offline mapping
	// ros::Subscriber sub = n.subscribe("/camera/depth/image", 1, kinnect_callback);
	// ros::Subscriber sub1 = n.subscribe("/camera1/depth/image", 1, measure_kinect_message_rate);
	ros::Subscriber sub1 = n.subscribe("/camera1/depth/image", 1, kinnect_callback);
	ros::Subscriber sub2 = n.subscribe("/camera2/depth/image", 1, kinnect_callback);

	ros::spin();

	return 0;
}
