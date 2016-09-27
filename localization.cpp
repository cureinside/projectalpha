/*********************************************************************
Dhimas Prabudi Wicaksa
Closed Environment Localization System - localization algorithm
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
#include "var.cpp"

using namespace cv;
using namespace std;

void localization()
{
scanningerror = false;
Mat lidarscan1 (501,501,CV_8UC1,Scalar(0));
Mat lidarscan2,lidarscan3,lidarscan4,lidarscan5,lidarscan6;
//.:.flush data-------------------------------------------------------
 for (int i = 0; i<10; i++)
 {
 fixedcorners[i].pointx	=  0;
 fixedcorners[i].pointy	=  0;
 fixedcorners[i].point	='X';
 fixedcorners[i].angle	=  0;
 corner[i].pointx	=  0;
 corner[i].pointy	=  0;
 corner[i].point	='X';
 corner[i].angle	=  0;
 fixedsize = 0; cnumber = 0; num = 'A';
  for (int j = 0; j<10; j++)
   f_range[i][j] = 0;
 }
 dataerror = false;
 datasize = 0;
//:.:flush data-------------------------------------------------------

//.:.save & convert data----------------------------------------------
save_data2();
convertdata(250,250,0.249,1000);

while (datasize==0)
{
 file.close();
 cout<<"closed"<<endl;
 waitKey(100);
 file.open (filepath.c_str());
 cout<<"opened"<<endl;
 save_data2();
 cout<<"datasize "<<datasize<<endl; }

drawmap (lidarscan1,2);
//:.:save & convert data----------------------------------------------

cvtColor (lidarscan1,lidarscan1,CV_GRAY2RGB);

lidarscan2=lidarscan1.clone();

//.:.corner-raw-------------------------------------------------------
int m,n;
m = 8;
n = 4;
int savedx[400],savedy[400], savedx2[400], savedy2[400];
int cornercount = 0, cornercount2 = 0;;
float amin = 1000, amax = 0;
for (int i=0; i<datasize; i++)
{
 float angletemp = midpoint(pxlx[i],pxlx[i+n],pxlx[i+m],
			     pxly[i],pxly[i+n],pxly[i+m]);
 if ((i+m)<=datasize)
 {
  if (midpoint(pxlx[i],pxlx[i+n],pxlx[i+m],
               pxly[i],pxly[i+n],pxly[i+m])<180)
  {
  circle(lidarscan2, Point(pxlx[i+n],pxly[i+n]),
	 3, Scalar(0,255,0), 2, 8, 0 );
  savedx[cornercount] = pxlx[i+n];
  savedy[cornercount] = pxly[i+n];
  cornercount++;
  }
 }
}
//:.:corner-raw-------------------------------------------------------

//.:.corner-adv-------------------------------------------------------
lidarscan3=lidarscan1.clone();

for (int i=0; i<cornercount; i++)
{
 float angletemp = midpoint(savedx[i],savedx[i+n],savedx[i+m],
			    savedy[i],savedy[i+n],savedy[i+m]);
 if ((i+m)<=cornercount)
 {
  if (angletemp<135)
  {
  circle(lidarscan3, Point(savedx[i+n],savedy[i+n]),
	 3, Scalar(0,0,255), 2, 8, 0 );
  savedx2[cornercount2] = savedx[i+n];
  savedy2[cornercount2] = savedy[i+n];
  cornercount2++;
  }
 }
}
//:.:corner-adv-------------------------------------------------------

//.:.corner point classification--------------------------------------
int ii = 0, loop = 0;
bool astop = false;
fixedcorners[0].pointx = savedx2[0];
fixedcorners[0].pointy = savedy2[0];

for (int i=1; i<cornercount2; i++)
{ astop = true; loop = 0;
  while (astop)
  {
    if ((avg(savedy2[i],fixedcorners[loop].pointy,10)) &&
        (avg(savedx2[i],fixedcorners[loop].pointx,10)))
	  {
	  astop=false;
   	     fixedcorners[loop].pointx =
	   ((fixedcorners[loop].pointx+savedx2[i])/2);
   	     fixedcorners[loop].pointy =
	   ((fixedcorners[loop].pointy+savedy2[i])/2);
          }
    else
    {
    loop++;
    if (loop-1==ii)
	 {
	 ii++;
	 astop=false;
	 fixedcorners[loop].pointy=savedy2[i];
	 fixedcorners[loop].pointx=savedx2[i];
	 }
    }
  }
}

lidarscan4 = lidarscan1.clone();
fixedsize = ii+1;

for (int i=0; i<fixedsize; i++)
{
  circle(lidarscan4,
         Point(fixedcorners[i].pointx,fixedcorners[i].pointy),
	 3, Scalar(0,255,0), 2, 8, 0 );
}
//:.:corner point classification--------------------------------------

//.:.angle examination------------------------------------------------
for (int a1=0; a1<10; a1++)
  for (int b1=0; b1<10; b1++)
    for (int c1=0; c1<10; c1++)
	f_angle[a1][b1][c1] = -1000;

for (int x = 0; x<fixedsize; x++)
{
int a1,b1,c1;
 a1=x;
 b1=x+1; if (b1>fixedsize-1) {b1=b1-fixedsize;}
 c1=x+2; if (c1>fixedsize-1) {c1=c1-fixedsize;}

 float angletemp = midpoint(fixedcorners[a1].pointx,
			    fixedcorners[b1].pointx,
			    fixedcorners[c1].pointx,
			    fixedcorners[a1].pointy,
			    fixedcorners[b1].pointy,
			    fixedcorners[c1].pointy);
 f_angle[a1][b1][c1] = angletemp;
 fixedcorners[b1].angle = angletemp;
}
//:.:angle examination------------------------------------------------

//.:.corner detection positive error handler--------------------------
int countmin = 0;
for (int x = 0; x<fixedsize; x++)
{
int a1,b1,c1;
 a1=x;
 b1=x+1; if (b1>fixedsize-1) b1=b1-fixedsize;
 c1=x+2; if (c1>fixedsize-1) c1=c1-fixedsize;

	if (f_angle[a1][b1][c1]>110 ||
	    isnan(f_angle[a1][b1][c1]) ||
	    f_angle[a1][b1][c1]<25)
	{
	for (int y=x+1; y<fixedsize-1; y++)
	 {
	 fixedcorners[y].pointx = fixedcorners[y+1].pointx;
	 fixedcorners[y].pointy = fixedcorners[y+1].pointy;
	 fixedcorners[y].angle  = fixedcorners[y+1].angle;
	 }
	countmin++;
	}
}
fixedsize=fixedsize-countmin;
//:.:corner detection positive error handler--------------------------


//.:.corner detection negative error handler--------------------------
countmin=0;
if (fixedsize==3)
{
 cout<<"MISSING POINT PREDICTION -->  ";
 int a2,b2,c0,x3,y3;
//.:.locate normal point----------------------------------------------
 for (int i=0; i<fixedsize; i++)
 {
  if (fixedcorners[i].angle>80 && fixedcorners[i].angle<100)
	{
	 c0 = i;
	 a2 = i+1; if (a2>fixedsize-1) a2=a2-fixedsize;
	 b2 = i-1; if (b2<0) b2=fixedsize-b2;
	}
 }
//:.:locate normal point----------------------------------------------
 for (x3=0; x3<501; x3++)
 {
  for (y3=0; y3<501; y3++)
  {
    float angletemp1 = midpoint(fixedcorners[a2].pointx,
                                fixedcorners[c0].pointx,
                                x3,
                                fixedcorners[a2].pointy,
                                fixedcorners[c0].pointy,
                                y3);
    float angletemp2 = midpoint(fixedcorners[b2].pointx,
                                fixedcorners[c0].pointx,
                                x3,
                                fixedcorners[b2].pointy,
                                fixedcorners[c0].pointy,
                                y3);

    if (angletemp1>85 && angletemp1<95 &&
	angletemp2>85 && angletemp2<95)
    {
     fixedcorners[3].pointx = x3;
     fixedcorners[3].pointy = y3;
     fixedcorners[3].angle  = ((angletemp1+angletemp2)/2);
     x3 = 1000; y3 =1000; fixedsize ++;
    }
  }
 }
 if (x3 ==501 && y3 ==501) {
			    cout<<"RECONSTRUCTION FAILED"<<endl;
			    return;
			   }
else
 if (x3>=1000 && y3>=1000) {cout<<"RECONSTRUCTION SUCCESS"<<endl;}
}
//:.:corner detection negative error handler--------------------------

//.:.range examination------------------------------------------------
for (int xx=0; xx<fixedsize; xx++)
{
  for (int yy=0; yy<fixedsize; yy++)
  {
    if (xx!=yy)
    {
	float jarak = sqrt
			(sqr(fixedcorners[xx].pointx-
			     fixedcorners[yy].pointx)+
			 sqr(fixedcorners[xx].pointy-
			     fixedcorners[yy].pointy));
	f_range[xx][yy] = jarak;
    }
  }
}
//:.:range examination------------------------------------------------

//.:.corners location drawing-----------------------------------------
lidarscan5 = lidarscan1.clone();

drawfixedcorners('1',lidarscan5);
//:.:corners location drawing-----------------------------------------

//.:.error trap-------------------------------------------------------
if (fixedsize !=4)
{
cout << "detected corner : "<<fixedsize<<endl;
cout << "error handling required "<<endl;

return;
}

checkpoint();

if (scanningerror) {cout<<"BAD DATA"<<endl; return;}
//.:.error trap-------------------------------------------------------

lidarscan6 = lidarscan1.clone();

drawcorner('A',lidarscan6);

//writeimage ("1",lidarscan1);
//writeimage ("1",lidarscan1);
//writeimage ("1",lidarscan1);
//writeimage ("1",lidarscan1);
//writeimage ("1",lidarscan1);

//iposition = lidarscan6.clone();
/*
lidarscan1 --> raw data
lidarscan2 --> raw corner detection
lidarscan3 --> advanced corner detection
lidarscan4 --> classified corner
lidarscan5 --> filtered corner
lidarscan6 --> localized corner
*/


//writeimage ("1",lidarscan1);
//writeimage ("2",lidarscan2);
//writeimage ("3",lidarscan3);
//writeimage ("4",lidarscan4);
//writeimage ("5",lidarscan5);
//writeimage ("6",lidarscan6);
//showimage  ("1",lidarscan1);
//showimage  ("2",lidarscan2);
//showimage  ("3",lidarscan3);
//showimage  ("4",lidarscan4);
//showimage  ("5",lidarscan5);
//showimage  ("6",lidarscan6);

Mat cpos (501,501,CV_8UC3,Scalar(0,0,0));
Mat cpos2 (501,501,CV_8UC3,Scalar(0,0,0));

//.:.save initial information-----------------------------------------
if (initial)
{
 for (int x=0; x<fixedsize; x++)
 {
 icorner[x] = corner[x];
 cout<<"initial point ";
 cout<<icorner[x].point<<" - ";
 cout<<icorner[x].pointx<<" - ";
 cout<<icorner[x].pointy<<endl;
// cout<<icorner[x].angle<<endl;
 lcorner[x] = corner[x];
 }

 for (int x=0; x<fixedsize; x++)
 {
 circle(initpos, Point(icorner[x].pointx,icorner[x].pointy),
        5, Scalar(255,255,255), 2, 8, 0 );
 string sst (1,icorner[x].point);
 putText(initpos,sst,
        Point(icorner[x].pointx,icorner[x].pointy),
        FONT_HERSHEY_PLAIN,2.2,Scalar(27,110,201),1,CV_AA);
 if (x<fixedsize-1)
   line(initpos,Point(icorner[x].pointx,icorner[x].pointy),
                Point(icorner[x+1].pointx,icorner[x+1].pointy),
		Scalar(224,193,56),2);
 else
   line(initpos,Point(icorner[x].pointx,icorner[x].pointy),
                Point(icorner[0].pointx,icorner[0].pointy),
		Scalar(224,193,56),2);
 }
}
//:.:save initial information-----------------------------------------

//.:.current information drawing--------------------------------------
else
if (!initial)
{
for (int x=0; x<fixedsize; x++)
 {
 circle(cpos, Point(corner[x].pointx,corner[x].pointy),
	5, Scalar(200,20,190), 2, 8, 0 );
 string sst (1,corner[x].point);
 putText(cpos,sst,
	Point(corner[x].pointx,corner[x].pointy),
	FONT_HERSHEY_PLAIN,2.2,Scalar(0,255,0),1,CV_AA);
 if (x<fixedsize-1)
   line(cpos,Point(corner[x].pointx,corner[x].pointy),
                Point(corner[x+1].pointx,corner[x+1].pointy),
		Scalar(100,24,200),2);
 else
   line(cpos,Point(corner[x].pointx,corner[x].pointy),
                Point(corner[0].pointx,corner[0].pointy),
		Scalar(100,24,200),2);
 }
cross (cpos,250,250);
}
//:.:current information drawing--------------------------------------

//.:.data normalization-----------------------------------------------
centerpoint();
int dx,dy;

if (initial) {
  	      cross(initpos,(int)centerpointx,(int)centerpointy);
              savedinit=initpos.clone();
              showimage("pos",initpos);

              initcenterx = centerpointx;
	      initcentery = centerpointy;
             } else

if (!initial)
{
 /*line(cpos,Point(centerpointx,0),
              Point(centerpointx,centerpointy), Scalar(25,100,240),2);
 line(cpos,Point(centerpointx,centerpointy),
              Point(midcx,midcy), Scalar(250,100,240),2);
 line(cpos,Point(midcx,midcy),
              Point(centerpointx,midcy), Scalar(25,200,24),2);
*/
 circle(cpos, Point(centerpointx,centerpointy),
	         5, Scalar(255,255,255), 2, 8, 0 );

 showimage ("currentpost",cpos);

 int cx2 = (centerpointx*cos_(-rotdegree))-
	   (centerpointy*sin_(-rotdegree));
 int cy2 = (centerpointx*sin_(-rotdegree))+
	   (centerpointy*cos_(-rotdegree));

 for (int x=0; x<fixedsize; x++)
 {
  int tx2 = corner[x].pointx;
  int ty2 = corner[x].pointy;
  corner[x].pointx = (tx2*cos_(-rotdegree))-(ty2*sin_(-rotdegree));
  corner[x].pointy = (tx2*sin_(-rotdegree))+(ty2*cos_(-rotdegree));
  int tx = corner[x].pointx;
  int ty = corner[x].pointy;

  dx = initcenterx-cx2;
  dy = initcentery-cy2;

  corner[x].pointx = tx+(dx);
  corner[x].pointy = ty+(dy);
 }
 centerpointx = centerpointx + (initcenterx-centerpointx);
 centerpointy = centerpointy + (initcentery-centerpointy);

cout<<endl;

 for (int x=0; x<fixedsize; x++)
 {
 circle(cpos2, Point(corner[x].pointx,corner[x].pointy),
        5, Scalar(200,20,190), 2, 8, 0 );
 circle(cpos2, Point(centerpointx,centerpointy),
        5, Scalar(200,20,190), 2, 8, 0 );
 string sst (1,corner[x].point);
 putText(cpos2,sst,
        Point(corner[x].pointx,corner[x].pointy),
        FONT_HERSHEY_PLAIN,2.2,Scalar(0,255,0),1,CV_AA);
 if (x<fixedsize-1)
   line(cpos2,Point(corner[x].pointx,corner[x].pointy),
                Point(corner[x+1].pointx,corner[x+1].pointy),
		Scalar(100,24,200),2);
 else
   line(cpos2,Point(corner[x].pointx,corner[x].pointy),
                Point(corner[0].pointx,corner[0].pointy),
		Scalar(100,24,200),2);

//.:.normalized data debugging----------------------------------------
 cout<<"current point ";
 cout<<corner[x].point<<" - ";
 cout<<corner[x].pointx<<" - ";
 cout<<corner[x].pointy<<endl;
// cout<<corner[x].angle<<endl;

//:.:normalized data debugging----------------------------------------
 }
}
//:.:data normalization-----------------------------------------------

//.:.scan point rotation----------------------------------------------
int newx2 = (250*cos_(-rotdegree))-(250*sin_(-rotdegree));
int newy2 = (250*sin_(-rotdegree))+(250*cos_(-rotdegree));

int newx3 = (250*cos_(-rotdegree))-(265*sin_(-rotdegree));
int newy3 = (250*sin_(-rotdegree))+(265*cos_(-rotdegree));

  newx2 = newx2+(dx);
  newy2 = newy2+(dy);

  newx3 = newx3+(dx);
  newy3 = newy3+(dy);
//:.:scan point rotation----------------------------------------------

  initpos=savedinit.clone();

//.:.step alg---------------------------------------------------------
  stepx[countstep] = newx2;
  stepy[countstep] = newy2;
  countstep++;
  if (countstep==6)
    {
    countstep = 5;
    stepx[0] = stepx[1]; stepx[1] = stepx[2]; stepx[2] = stepx[3];
    stepx[3] = stepx[4]; stepx[4] = stepx[5]; stepx[5] = stepx[6];

    stepy[0] = stepy[1]; stepy[1] = stepy[2]; stepy[2] = stepy[3];
    stepy[3] = stepy[4]; stepy[4] = stepy[5]; stepy[5] = stepy[6];
    }
  for (int i=0; i<6; i++)
	{
	if (stepx[i] != 0 && stepy[i] !=0)
	  line(initpos, Point(stepx[i],stepy[i]),
			Point(stepx[i],stepy[i]),
	        	Scalar(255,255,255), 2);
	}
//:.:step alg---------------------------------------------------------

//.:.scan point drawing-----------------------------------------------
  circle(initpos, Point(newx2,newy2),
	         5, Scalar(0,85,28), 2, 8, 0 );

  line(initpos, Point(newx3,newy3), Point(newx2,newy2),
	        Scalar(62,255,126), 2);
  showimage ("pos",initpos);
  if (!initial) showimage ("normalized",cpos2);
//:.:scan point drawing-----------------------------------------------

a=0;
while (a==0) {a=waitKey(100);}
}
