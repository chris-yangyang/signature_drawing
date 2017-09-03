#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <exception>
#include <vector>
#include <opencv2/opencv.hpp>

class transformation2D
{
   public:
      transformation2D();
      ~transformation2D();
      transformation2D(Point _t, double _scale, double _rotation);

};
