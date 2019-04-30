#include "gps_deal.h"

//已知两点经纬度求距离
double distance(double rear_lat,double rear_lon,double front_lat,double front_lon)
{
	double dis = 0;
	double Front_Lat = DEG2RAD(front_lat);
	double Front_Lon = DEG2RAD(front_lon);
	double Rear_Lat = DEG2RAD(rear_lat);
	double Rear_Lon = DEG2RAD(rear_lon);
	double a = Rear_Lat-Front_Lat;
	double b = Rear_Lon-Front_Lon;
	dis = 2*R*asin(sqrt(pow(sin(a/2),2)+cos(Front_Lat)*cos(Rear_Lat)*pow(sin(b/2),2)));
	return dis;
}

//已知两点经纬度求两点连线与正北方向夹角,[0,2*pi]
double bearing(double rear_lat,double rear_lon,double front_lat,double front_lon)
{
	double bear = 0;
	double Front_Lat = DEG2RAD(front_lat);
	double Front_Lon = DEG2RAD(front_lon);
	double Rear_Lat = DEG2RAD(rear_lat);
	double Rear_Lon = DEG2RAD(rear_lon);
	double x = sin(Front_Lon-Rear_Lon)*cos(Front_Lat);
	double y = cos(Rear_Lat)*sin(Front_Lat)-sin(Rear_Lat)*cos(Front_Lat)*cos(Front_Lon-Rear_Lon);
	bear = atan2(x,y);
	if(bear>=-M_PI && bear<0)
		bear += 2*M_PI;
	return bear;
}

