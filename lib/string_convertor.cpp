/*************************************************************************
	> File Name: triangular_dist.cpp
	> Jun Jin.:
	> Mail:jjin5@ualberta.ca
    > ---- triangular distribution functions.----
	> Created Time: Mon 20 Feb 2017 09:23:14 PM MST
 ************************************************************************/

#include "../include/string_convertor.h"

string string_convertor::d2s(double d)
{
  std::ostringstream d2str;
  d2str<<d;
  return d2str.str();
}


std::vector<float> string_convertor::convert2Float(std::vector<double> v)
{
  std::vector<float> t;
  for(int i=0;i<v.size();i++)
    t.push_back((float)v[i]);
  return t;
}

void string_convertor::printOutStdVector(std::vector<double> v)
{
 //cout<<"print out v:"<<endl;
 for(int i=0;i<v.size();i++)
 {
   cout<<v.at(i)<<"  ";
 }
 cout<<endl;
}

/**1 2 3 4 5 splitor: space
* input string 1 2 3 4 5 6
output array [1,2,3,4,5,6]
*/
std::vector< double > string_convertor::fromString2Array(string inStr)
{
   std::vector< double > vd;
   string buf; // Have a buffer string
   stringstream ss(inStr); // Insert the string into a stream
   while (ss >> buf)
       vd.push_back(atof(buf.c_str()));
   return vd;
}

/**
input string 1 2 3 4 5 6
output string array ["1","2","3","4","5","6"]
*/
std::vector< string > string_convertor::fromString2ArrayStr(string inStr)
{
   std::vector< string > vd;
   string buf; // Have a buffer string
   stringstream ss(inStr); // Insert the string into a stream
   while (ss >> buf)
       vd.push_back((buf.c_str()));
   return vd;
}

string string_convertor::constructPubStr(vector< vector<Point> > vps)
{
   string rtStr="";
   size_t strokesNum=vps.size();
   for(int i=0;i<strokesNum;i++)
   {
      size_t pointsNum=vps[i].size();
      if(pointsNum>0)
      {
        string thisLineStr="";
        for(int j=0;j<pointsNum-1;j++)
          thisLineStr += d2s(vps[i][j].x)+" "+d2s(vps[i][j].y)+" ";
        thisLineStr+=d2s(vps[i][pointsNum-1].x)+" "+d2s(vps[i][pointsNum-1].y);
        if(i==strokesNum-1)
          rtStr+=thisLineStr;
        else
          rtStr+=thisLineStr+";";
      }
   }
   return rtStr;
}
double string_convertor::pointDistance(Point2f p1, Point2f p2)
{
  double deltax=p1.x-p2.x;
  double deltay=p1.y-p2.y;
  return sqrt(deltax*deltax+deltay*deltay);
}

string string_convertor::constructPubStr2(vector< vector<Point> > vps, int gap)
{
  string rtStr="";
  size_t strokesNum=vps.size();

  for(int i=0;i<strokesNum;i++)
  {
    size_t pointsNum = vps[i].size();

    //down sampling points by straight line approximation
    string thisLineStr="";
    if(1 == pointsNum){//strokes with only one pixel

      thisLineStr +=d2s(vps[i][0].x)+" "+d2s(vps[i][0].y);

    }else if(pointsNum < 5 && pointsNum > 1){//stroks with less than 5 pixels and more than 1

      thisLineStr +=d2s(vps[i][0].x)+" "+d2s(vps[i][0].y)+" "+
      d2s(vps[i][round((pointsNum-1)/2)].x)+" "+d2s(vps[i][round((pointsNum-1)/2)].y)+" "+
      d2s(vps[i][pointsNum-1].x)+" "+d2s(vps[i][pointsNum-1].y);

    }else if(pointsNum > 4){//strokes with more than 4 pixels

      double sample_dist_tmp = 0;
      for(int j = 0; j < pointsNum; j++){//count the distances from each pixel to the first one
        if(0==j || sample_dist_tmp > 5){//store the first point//every 5 pixels do the downsampling
          thisLineStr +=d2s(vps[i][j].x)+" "+d2s(vps[i][j].y)+" ";
          sample_dist_tmp = 0;
        }else{
          sample_dist_tmp += sqrt(pow(double(vps[i][j].x)-double(vps[i][j-1].x),2)+
          pow(double(vps[i][j].y)-double(vps[i][j-1].y),2));
        }//end if 0==j
      }//end for j
      thisLineStr +=d2s(vps[i][pointsNum-1].x)+" "+d2s(vps[i][pointsNum-1].y);
    }//end if pointsNum>4

    if(i==strokesNum-1)
      rtStr+=thisLineStr;
    else
      rtStr+=thisLineStr+";";


/*    if(pointsNum > 2){
      string thisLineStr="";
      thisLineStr +=d2s(vps[i][0].x)+" "+d2s(vps[i][0].y)+" "+
      d2s(vps[i][round(pointsNum/4)].x)+" "+d2s(vps[i][round(pointsNum/4)].y)+" "+
      d2s(vps[i][round(pointsNum/2)].x)+" "+d2s(vps[i][round(pointsNum/2)].y)+" "+
      d2s(vps[i][round(pointsNum*3/4)].x)+" "+d2s(vps[i][round(pointsNum*3/4)].y)+" "+
      d2s(vps[i][pointsNum-1].x)+" "+d2s(vps[i][pointsNum-1].y);
      if(i==strokesNum-1)
        rtStr+=thisLineStr;
      else
        rtStr+=thisLineStr+";";
    }*/
    /* size_t pointsNum=vps[i].size();
     if(pointsNum>0)
     {
       string thisLineStr="";
       for(int j=0;j<pointsNum-1;j++)
         if(j%gap==0||pointsNum<gap)
              thisLineStr += d2s(vps[i][j].x)+" "+d2s(vps[i][j].y)+" ";
       thisLineStr+=d2s(vps[i][pointsNum-1].x)+" "+d2s(vps[i][pointsNum-1].y);
       if(i==strokesNum-1)
         rtStr+=thisLineStr;
       else
         rtStr+=thisLineStr+";";
     }*/
  }
  return rtStr;
}
