/*********************************************************************
Dhimas Prabudi Wicaksa
Closed Environment Localization System - main
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
#include "localization.cpp"

using namespace cv;
using namespace std;

int mode = 0;

/*******************************************
 mode 1 = real-time localization
 mode 2 = step-simulation localization
*******************************************/

int main(int arr, char** loww)
{
int rat = atoi(loww[1]);

if (rat==0) {mode=1; filepath = "network/datalidar.txt";} else
	    {mode=2; filepath = "datalidar/sequential1/d";}
string savedfilepath=filepath;


cout<<"running program mode : "<<mode<<endl;

if (mode==1)
{
 file.open (filepath.c_str());
 initial = true;
 localization();
 initial = false;
 file.close();

 while (true)
 {
 file.open(filepath.c_str());
 if (!file.eof()) localization; else cout<<"FETCH DATA ERROR"<<endl;
 file.close();
 waitKey(100);
 }
}
else
if (mode==2)
{
string path_temp;
path_temp.append(cvt2str(rat));
filepath.append(path_temp);
ops.append(path_temp);
ops.append("-");

file.open (filepath.c_str());
initial = true;
localization();
initial = false;
file.close();

 while (rat<10)
 {
 int b=0;

 if (showimages)
  while ((char) b != 'a' && (char) b != 'x' && (char) b != 's')
	{b=waitKey(30);}
 if ((char) b == 'x') return 1;
 if ((char) b == 's') rat=rat-2;

 rat++;
 path_temp.clear();
 path_temp.append(cvt2str(rat));
 filepath.clear();
 filepath=savedfilepath;
 filepath.append(path_temp);
 ops.clear();
 ops.append(path_temp);
 ops.append("-");

 file.open(filepath.c_str());
 if (!file.eof()) localization(); else cout<<"FETCH DATA ERROR"<<endl;
 file.close();

 }
 int stopkey=-1;
 while (stopkey==-1){stopkey=waitKey(30);}
}

return 0;
}
