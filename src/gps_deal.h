#ifndef GPS_DEAL_H_   
#define GPS_DEAL_H_
#include <stdio.h>
#include <math.h>
#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)*M_PI/180)
#define R 6378137

//已知两点经纬度求距离
double distance(double rear_lat,double rear_lon,double front_lat,double front_lon);
//已知两点经纬度求两点连线与正北方向夹角,[0,2pi]
double bearing(double rear_lat,double rear_lon,double front_lat,double front_lon);


#endif
