
#ifndef __LINEAR_ALGEBRA_UTILS_H__
#define __LINEAR_ALGEBRA_UTILS_H__

#include "tf/transform_datatypes.h"
#include <string>

using namespace std;

class SphericalPoint
{
	public:

		SphericalPoint();
		SphericalPoint(double v, double h, double r);

		double vangle;
		double hangle;
		double radius;
};


class CartesianPoint
{
	public:

		CartesianPoint();
		CartesianPoint(double x_in, double y_in, double z_in);

		double x;
		double y;
		double z;
};


class TransformUtil
{
	public:

		static SphericalPoint toSpherical(CartesianPoint p);
		static CartesianPoint toCartesian(SphericalPoint p);

		static SphericalPoint TransformSpherical(SphericalPoint p, tf::Transform transform);
		static CartesianPoint TransformCartesian(CartesianPoint p, tf::Transform transform);

};


#endif
