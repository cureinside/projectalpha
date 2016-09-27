/*********************************************************************
Dhimas Prabudi Wicaksa
Closed Environment Localization System - variables & voids
*********************************************************************/
#include <string>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <ctime>
#include <cmath>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

using namespace cv;
using namespace std;

//.:.interfacing------------------------------------------------------
bool showimages = true;
bool saveimage 	= true;
bool deb 	= false;
bool initial;
//:.:interfacing------------------------------------------------------

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int stop, pxlx[400], pxly[400], a, fixedsize, setpixelX, setpixelY,
    datasize = 0, cnumber = 0, initcenterx, initcentery, countstep=0;

int stepx[7],stepy[7];

string filepath,ops;

ifstream file;

bool dataerror = false, scanningerror = false;

double degree[400], range [400], quality[400];

char num = 'A';

float f_angle [10][10][10], f_range [10][10], lastanglerotation;

int centerpointx, centerpointy;
float rotdegree;
int midcy,midcx;
Mat lidarused,iposition,cposition,tempmat,lastmatrotation,savedinit;
Mat initpos (501,501,CV_8UC3,Scalar(0,0,0));
Mat rawpost1 (501,501,CV_8UC1,Scalar(0));


typedef struct {char point; int pointx; int pointy; float angle;}
circles;
circles fixedcorners[10], corner[10],
	icorner[10], lcorner[10], tempc[10];
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void centerpoint();
void read_data(int count);
void save_data();
void convertdata(int x, int y, float ps, int maxrange);
void drawmap (Mat lidar, int w);
void erode_ (Mat lidar, int val1, int val2);
void dilate_ (Mat lidar, int val1, int val2);
void cross (Mat lidar, int val1, int val2);
void harriscorner();
void checkpoint();
void writeimage(string a,Mat aa);
void showimage(string a,Mat aa);
void drawfixedcorners(char aa, Mat b);
void drawcorner(char a, Mat b);
void debg(string a);
bool avg(int a,int b, int c);
double sin_(double par);
double asin_(double par);
double cos_(double par);
double acos_(double par);
float midpoint(int ax, int bx, int cx, int ay, int by, int cy);
int sqr(int a);
string cvt2str(int a);
void rotation();
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void debg(string a) {if (deb) cout<<a<<endl;}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool avg(int a,int b, int c)
  {
  if (abs(a-b)<=c) return true; else return false;
  }
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
string cvt2str(int a)
{
stringstream instr;
instr<<a;
return instr.str();
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
float midpoint(int ax, int bx, int cx, int ay, int by, int cy)
 {
 float temp,angle;
 float r1,r2,r3 = 0;

  r1 = sqrt (sqr(ax-bx)+sqr(ay-by));
  r2 = sqrt (sqr(bx-cx)+sqr(by-cy));
  r3 = sqrt (sqr(ax-cx)+sqr(ay-cy));

  temp = ((sqr(r1)+sqr(r2)-sqr(r3))/(2*r1*r2));
  angle = acos_(temp);
 return angle;}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int sqr(int a) {return (a*a);}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void writeimage(string a,Mat aa)
{
 if (saveimage)
  {
  string b,c,d,z;
  z.append(a);
  b="imageresult/"+ops;
  c=a+".jpg";
  d.append(b);
  d.append(c);
  imwrite (d,aa);
  if (deb) cout<<a<<" saved"<<endl;
  }
 else
  {
  if (deb) cout<<a<<" not saved"<<endl;
  }
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void showimage(string a,Mat aa)
{
  if (showimages && !scanningerror)  imshow (a,aa);
  else if (deb) cout<<a<<" not showed"<<endl;
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void read_data(int count)
{
  std::string a;
  dataerror=false;
    if (!file) {std::cout<<"error"<<std::endl;} else
    {
    file>>a;
      if (a!="")
        {
         degree[count]=::atof(a.c_str());
         file>>a;
         range[count]=::atof(a.c_str());
         file>>a;
         quality[count]=::atof(a.c_str());
        }
      else
        {
         dataerror=true;
        }
    }
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void save_data()
{
   std::string a;
   int i=0;
   file>>a; file>>a; file>>a; file>>a;
   file>>a; file>>a; file>>a;
    while (!file.eof())
        {
          read_data(i);
          if (!dataerror) i++;
        }
   datasize = i;
}

void save_data2()
{
   std::string a;
  // if (file.eof())
  //	{cout<<"error"<<endl; file.close();
  //	file.open (filepath.c_str()); cout<<"error"<<endl;}
  // cout<<a<<endl;
   int i=0;
   file>>a;
    while (!file.eof())
        {
          read_data(i);
          if (!dataerror) i++;
        }
   datasize = i;
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void convertdata(int x, int y, float ps, int maxrange)
{
   setpixelX = x; setpixelY = y;
   float a,b;
   for (int i=0; i<datasize;i++)
        {
         if (range[i]<=maxrange) //1 meter
          {
           a = (sin_(degree[i])*range[i]*ps);
            pxlx[i]=setpixelX+ static_cast<int>(a);
           b = (cos_(degree[i])*range[i]*ps);
            pxly[i]=setpixelY+ static_cast<int>(b);
          }
        }
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void drawmap(Mat lidar, int w)
{
   for (int i=0; i<datasize; i++)
    {
        line(lidar,Point(pxlx[i],pxly[i]),
                   Point(pxlx[i],pxly[i]), Scalar(255),w);
    }
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void erode_ (Mat lidar, int val1,int val2)
{
   Mat erodeelement = getStructuringElement (MORPH_RECT,
                                             Size(val1,val1),
                                             Point(val2,val2));
   erode (lidar,lidar,erodeelement);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void dilate_ (Mat lidar, int val1,int val2)
{
   Mat dilateelement = getStructuringElement (MORPH_RECT,
                                             Size(val1,val1),
                                             Point(val2,val2));
   dilate (lidar,lidar,dilateelement);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void checkpoint()
{
int temp[2],count = 0; bool end = false;
int thresh = 250; int ercount = 0; int descount = 2;

if(initial)
{while(!end)
{

for (int x = 0; x<fixedsize; x++)
 {
ercount++;
  if (fixedcorners[x].pointy<=thresh)
	{temp[count]=x; count++;}
 }
 if (count==descount) end=true; else
	{
	if (count<descount) {thresh++;} else
		     {thresh--;}
	count=0;
	}
 if (ercount>500 || fixedsize==0)
  {
  descount++;
  ercount=0;
  scanningerror=true;
  cout<<"BAD DATA"<<endl; end=true;
  }
}

 if (fixedcorners[temp[0]].pointx<fixedcorners[temp[1]].pointx)
	{
	corner[0]=fixedcorners[temp[0]]; corner[0].point = 'A';
	corner[1]=fixedcorners[temp[1]]; corner[1].point = 'B';
	} else
	{
	corner[1]=fixedcorners[temp[0]]; corner[1].point = 'B';
	corner[0]=fixedcorners[temp[1]]; corner[0].point = 'A';
	}
count = 0;
end = false;
thresh = 250;
while(!end)
{
for (int x = 0; x<fixedsize; x++)
 {
  if (fixedcorners[x].pointy>thresh)
	{temp[count]=x; count++;}
 }
 if (count==2) end=true; else
	{
	if (count<2) {thresh--;} else
		     {thresh++;}
	count=0;
	}
}
 if (fixedcorners[temp[0]].pointx<fixedcorners[temp[1]].pointx)
	{
	corner[3]=fixedcorners[temp[0]]; corner[2].point = 'C';
	corner[2]=fixedcorners[temp[1]]; corner[3].point = 'D';
	} else
	{
	corner[2]=fixedcorners[temp[0]];
	corner[3]=fixedcorners[temp[1]]; corner[2].point = 'C';
	corner[3].point = 'D';
	}

}
else
{
for (int i=0; i<fixedsize; i++)
 corner[i].point = 'X';
//.:.ROI alg----------------------------------------------------------
int par = 20;
int erh = 0;
int erh1 = -1;
int erh2 = 100;
if (!initial && (fixedsize==4))
 {
 for (int i=0; i<fixedsize; i++)
  {
  bool end2 = false;
  int j = 0;
  while (!end2)
    {
    if ((fixedcorners[i].pointx<(lcorner[j].pointx+par)) &&
	(fixedcorners[i].pointx>(lcorner[j].pointx-par)) &&
	(fixedcorners[i].pointy<(lcorner[j].pointy+par)) &&
    	(fixedcorners[i].pointy>(lcorner[j].pointy-par))
	 && (corner[j].point == 'X') && (erh1!=j))
	{
	 end2 = true;
	 corner[j]=fixedcorners[i];
	 corner[j].point = lcorner[j].point;
	} else
	{
	 j++;
	 if (j==fixedsize) {j=0; par++;}
	}
    }
  }
 }

}
//:.:ROI alg----------------------------------------------------------
for (int i = 0; i<fixedsize; i++)
 lcorner[i] = corner[i];

}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//.:.corner-classification-output-------------------------------------
void drawcorner(char a, Mat b)
{
for (int i = 0; i<fixedsize; i++)
{
//  corner[i].point = a;
  circle(b, Point(corner[i].pointx,corner[i].pointy),
	5, Scalar(130,20,190), 2, 8, 0 );
  string stemp (1,a);
  putText(b,stemp,
	Point(corner[i].pointx,corner[i].pointy),
	FONT_HERSHEY_PLAIN,2.2,Scalar(0,255,0),1,CV_AA);
  char ctemp[100];
  sprintf (ctemp,"%c(%d,%d)",corner[i].point,corner[i].pointx,
	corner[i].pointy);
  string stemp2(ctemp);
  putText(b,stemp2,Point(10,(20+(i*18))),
	FONT_HERSHEY_SIMPLEX,0.35,Scalar(255,255,0),1,CV_AA);

/*  if (deb) printf ("%c\t%d\t%d\n",corner[i].point,
	  		          corner[i].pointx,
			          corner[i].pointy);
*/
  a++;
  }
}
//:.:corner-classification-output-------------------------------------

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//.:.fixedcorner-classification-output--------------------------------
void drawfixedcorners(char aa, Mat b)
{
a=aa;
for (int i = 0; i<fixedsize; i++)
{
  fixedcorners[i].point = a;
  circle(b, Point(fixedcorners[i].pointx,fixedcorners[i].pointy),
	5, Scalar(130,20,190), 2, 8, 0 );
  string stemp (1,fixedcorners[i].point);
  putText(b,stemp,
	Point(fixedcorners[i].pointx,fixedcorners[i].pointy),
	FONT_HERSHEY_PLAIN,2.2,Scalar(0,255,0),1,CV_AA);
  char ctemp[100];
  sprintf (ctemp,"%c(%d,%d)",a,fixedcorners[i].pointx,
	fixedcorners[i].pointy);
  string stemp2(ctemp);
  putText(b,stemp2,Point(10,(20+(i*18))),
	FONT_HERSHEY_SIMPLEX,0.35,Scalar(255,255,0),1,CV_AA);

/*  if (deb) printf ("%c\t%d\t%d\n",fixedcorners[i].point,
	  		          fixedcorners[i].pointx,
			          fixedcorners[i].pointy);
*/
  a++;
  }
}
//:.:fixedcorner-classification-output--------------------------------

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
double sin_(double par)
  {double temp = sin (par*CV_PI/180); return temp;}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
double cos_(double par)
  {double temp = cos (par*CV_PI/180); return temp;}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
double asin_(double par)
  {double temp;
	  temp = asin(par);
	  temp = ((temp*180)/CV_PI); return temp;}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
double acos_(double par)
  {double temp;
	  temp = acos(par);
	  temp = ((temp*180)/CV_PI); return temp;}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void centerpoint()
{
//.:.range------------------------------------------------------------
float max1=0,max2=0;
int pa1,pa2,pb1,pb2;
  for (int i=0; i<fixedsize-1; i++)
   {
    for (int j=i+1; j<fixedsize; j++)
    {
     float range; int pos1,pos2;
     range = sqrt(sqr(corner[i].pointx-corner[j].pointx)
                 +sqr(corner[i].pointy-corner[j].pointy));
     pos1 = i; pos2 = j;
     if (range>max1) {max1 = range; pa1=pos1; pa2 = pos2;}
    }
   }
//:.:range------------------------------------------------------------

if (pa1==0) pb1=1; else pb1=0;
if (pa2==2) pb2=3; else pb2=2;

//.:.line equation----------------------------------------------------
int x1a = corner[pa1].pointx;
int x2a = corner[pa2].pointx;
int y1a = corner[pa1].pointy;
int y2a = corner[pa2].pointy;

int x1b = corner[pb1].pointx;
int x2b = corner[pb2].pointx;
int y1b = corner[pb1].pointy;
int y2b = corner[pb2].pointy;

int cya = x2a-x1a;
int cxa = y2a-y1a;
int cca = ((cya*y1a)-(cxa*x1a));

int cyb = x2b-x1b;
int cxb = y2b-y1b;
int ccb = ((cyb*y1b)-(cxb*x1b));

float ty3a = cca- (ccb*cxa/cxb);
float ty3b = cya-(cxa*cyb/cxb);
float cyf = ty3a/ty3b;
float cxf = ((cya*cyf)-cca)/cxa;

centerpointx = cxf;
centerpointy = cyf;
//:.:line equation----------------------------------------------------

float rangec = sqrt(sqr(corner[pa1].pointx-corner[pb1].pointx)
                      +sqr(corner[pa1].pointy-corner[pb1].pointy));
float rangeb = sqrt(sqr(corner[pa1].pointx-centerpointx)
                   +sqr(corner[pa1].pointy-centerpointy));
float rangea = sqrt(sqr(corner[pa1].pointx-centerpointx)
                   +sqr(corner[pa1].pointy-centerpointy));

float midangle = acos_((sqr(rangea)+sqr(rangeb)-
			sqr(rangec))/(2*rangea*rangeb));


midcx = (corner[pa1].pointx+corner[pb1].pointx)/2;
midcy = (corner[pa1].pointy+corner[pb1].pointy)/2;


if ((midcx-centerpointx)!=0)
 {
  rotdegree = asin_((midcx-centerpointx)/
		    (sqrt(sqr(midcx-centerpointx)+
			  sqr(midcy-centerpointy))));
  if (midcy>centerpointy) {rotdegree = 180-rotdegree;}
 }
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void cross (Mat lidar, int val1, int val2)
{
 line(lidar,Point(val1+4,val2-4),
            Point(val1-4,val2+4), Scalar(255,255,255),2);
 line(lidar,Point(val1+4,val2+4),
            Point(val1-4,val2-4), Scalar(255,255,255),2);
 circle(lidar, Point(val1,val2),
        4, Scalar(0,0,255), 2, 8, 0 );

}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
