#pragma once
// #include "bezier.h"
#include<vector>
#include <pangolin/pangolin.h>

constexpr auto Swidth = 1000;
constexpr auto Sheight = 1200;
constexpr auto deltaTIME = 50000;

 //点
class Point	{
public:
	double x;
	double y;
};

enum DriveStatus {
	state1 = 0,
	state2 = 1,
	ahead = 2,
	turning = 3,
	exiting = 4
};

enum showMode {
	staticShow,
	staticOfficial,
	dynamic
};
//车辆基类
class CarBase {
public:
	virtual void showCar(Point p) = 0;//绘制车辆，纯虚函数
public:
	double length;//长度
	double width;//宽度

	Point p0;//中心点
	Point p1;//终点位置
};

//车辆
class Car :public CarBase {
public:
	Car(double Rw);
	void showCar(Point p);//绘制车辆
};

//障碍车辆
class CarObs :public CarBase {
public:
	CarObs(double Rw);
	void showCar(Point p);//绘制障碍车辆
};

//道路
class Road {
public:
	Road();
	~Road();
	void showRoad(Point p);//绘制道路
public:
	double RWidth = 100.0;
	double dot_distance = 8.0; // 虚线点的间隔
	CarBase* car = new Car(RWidth);//多态：父类指针指向子类对象
	CarBase* carObs = new CarObs(RWidth);//多态：父类指针指向子类对象
};

//贝塞尔曲线
class Bezier {
public:
	Bezier();
	~Bezier();
	void SetPangolinParams(pangolin::OpenGlRenderState *camera, pangolin::View *view);
	void CarGoStraight();//直行
	void showAheadTrack();//绘制直行规划线轨迹
	void setControlPoint();//设置控制点
	void showBezierTrack(int trackIndex);//绘制贝塞尔曲线轨迹
	// void showBezierTrackOfficial();//用官方函数绘制贝塞尔曲线轨迹
	void CarChangeLane();//车辆换道移动
	void process();//整个过程
public:
	Road road;

	Point p0;
	Point p1;
	Point p2;
	Point p3;//4个控制点

	double disS = road.car->length * 5;//纵向换道距离
	double safeDis = 500.0;//纵向安全距离
	int disAhead = safeDis / 50;//直行规划线的间隔
	double speed = 5.0;//直行速度
	double delta_t = 0.02;//时间间隔
	std::vector<Point> trackPoints;//存储轨迹点
	int curTrackIndex = -1;
	int showMode = dynamic;
	DriveStatus state = state1;
	pangolin::OpenGlRenderState *camera_p = nullptr;
	pangolin::View *view_p = nullptr;
};
