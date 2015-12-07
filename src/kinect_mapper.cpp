
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "KinectToLaserAdapter.h"
#include <tf/transform_broadcaster.h>
#include <iostream>


/**
 * *********************************************************
 *
 * INFORMACOES IMPORTANTISSIMAS:
 * - A mensagem de depth do ros usa nan p/ representar oclusoes!
 * - P/ acessar os elementos da imagem de depth eh necessario fazer img.at<float>(i, j) pq isso pode retornar nan. O acesso com img.data[i * step + j] nao funciona!
 * - Aparentemente a mensagem de depth retorna a profundidade em metros. P/ mais info veja a documentacao em http://www.ros.org/reps/rep-0118.html.
 *
 * *********************************************************
 */

int VIEW_ACTIVE = 1;
ros::Publisher g_laser_publisher;


void
measure_message_rate(const sensor_msgs::Image::ConstPtr& cam1_msg, const sensor_msgs::Image::ConstPtr& cam2_msg)
{
	static int n = 0;
	static double t = 0;

	if (ros::Time::now().toSec() - t > 1.0)
	{
		std::cout << "rate: " << n << std::endl;

		t = ros::Time::now().toSec();
		n = 0;
	}
	else
		n++;
}


void
cams_to_laser_handler(const sensor_msgs::Image::ConstPtr& cam1_msg, const sensor_msgs::Image::ConstPtr& cam2_msg)
{
	static tf::TransformBroadcaster br;
	tf::Transform transform(tf::Quaternion(0, 0, 0), tf::Vector3(0, 0, 0));
	br.sendTransform(tf::StampedTransform(transform, cam1_msg->header.stamp, "base_link", "scan"));

	KinectToLaserAdapter adapter;
	sensor_msgs::LaserScan scan;

	scan = adapter.translate(cam1_msg, cam2_msg);

	if (VIEW_ACTIVE)
		adapter.view();

	g_laser_publisher.publish(scan);
}


int
main(int argc, char **argv)
{
	ros::init(argc, argv, "kinnect_mapper");
	ros::NodeHandle n;

	std::cout << "Node started...\n" << std::endl;

	g_laser_publisher = n.advertise<sensor_msgs::LaserScan>("/scan", 30);

	message_filters::Subscriber<sensor_msgs::Image> right_cam_subscriber(n, "/camera1/depth/image", 1);
	message_filters::Subscriber<sensor_msgs::Image> left_cam_subscriber(n, "/camera2/depth/image", 1);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

	// Note: ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), left_cam_subscriber, right_cam_subscriber);

	// uncomment the first to measure the rate when the synchronization is used, and the second to
	// transform the cams messages in a laser message.
	// sync.registerCallback(boost::bind(&measure_message_rate, _1, _2));
	sync.registerCallback(boost::bind(&cams_to_laser_handler, _1, _2));

	ros::spin();
	return 0;
}
