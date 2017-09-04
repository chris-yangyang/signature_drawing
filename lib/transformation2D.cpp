#include "../include/transformation2D.h"

/** constuctor
_t: 2d translation
_scale: scale
_rotation: rotation in degrees
*/
transformation2D::transformation2D(Point _t, double _scale, double _rotation)
{
  t=_t;
  scale=_scale;
  rotation=_rotation;
}

Point2d transformation2D::doTransformation(Point2d p1)
{
   //get the 2*3 transformation matrix
   Mat transformation=cv::getRotationMatrix2D(Point2f(p1.x, p1.y), rotation, scale);
   cv::Mat_<double>p13(3,1);
   p13(0,0)=p1.x;
   p13(1,0)=p1.y;
   p13(2,0)=1.0;
   cv::Mat_<double> p2=transformation*p13;
   return Point2d(p2(0,0),p2(1,0));
}
