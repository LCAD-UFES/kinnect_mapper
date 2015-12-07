
#include "LinearAlgebraUtils.h"
#include "tf/transform_datatypes.h"
#include <string>
#include <cmath>

using namespace std;


SphericalPoint::SphericalPoint()
{
	 vangle = hangle = radius = 0;
}


SphericalPoint::SphericalPoint(double v, double h, double r)
{
	vangle = v;
	hangle = h;
	radius = r;
}


CartesianPoint::CartesianPoint()
{
	x = y = z = 0;
}


CartesianPoint::CartesianPoint(double x_in, double y_in, double z_in)
{
	x = x_in;
	y = y_in;
	z = z_in;
}


SphericalPoint
TransformUtil::toSpherical(CartesianPoint p)
{
	SphericalPoint s;

	s.radius = sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));
	s.vangle = atan2(p.z, sqrt(pow(p.x, 2) + pow(p.y, 2)));
	s.hangle = atan2(p.y, p.x);

	return s;
}


CartesianPoint
TransformUtil::toCartesian(SphericalPoint p)
{
	double rxy;

	CartesianPoint c;

	rxy = p.radius * cos(p.vangle);

	c.x = rxy * cos(p.hangle);
	c.y = rxy * sin(p.hangle);
	c.z = p.radius * sin(p.vangle);

	return c;
}


SphericalPoint
TransformUtil::TransformSpherical(SphericalPoint p, tf::Transform transform)
{
	CartesianPoint q = toCartesian(p);
	CartesianPoint r = TransformCartesian(q, transform);
	SphericalPoint s = toSpherical(r);

	// DEBUG:
//	printf("p: %lf %lf %lf\n", p.hangle, p.vangle, p.radius);
//	printf("cart q: %lf %lf %lf\n", q.x, q.y, q.z);
//	printf("cart transf r: %lf %lf %lf\n", r.x, r.y, r.z);
//	printf("spher transf s: %lf %lf %lf\n", s.hangle, s.vangle, s.radius);
//	printf("\n");

	return s;
}


CartesianPoint
TransformUtil::TransformCartesian(CartesianPoint p, tf::Transform transform)
{
	CartesianPoint c;

	tf::Vector3 tf_p(p.x, p.y, p.z);
	tf::Vector3 tf_c = transform * tf_p;

	c = CartesianPoint(tf_c.x(), tf_c.y(), tf_c.z());

	return c;
}
