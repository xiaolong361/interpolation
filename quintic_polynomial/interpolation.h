#pragma once

#include <iostream>
#include <cmath>
#include <ctime>
#include <vector>
#include <pangolin/pangolin.h>

using namespace std;

constexpr auto N = 10;					//矩阵的最大行(列)数
constexpr auto Swidth = 600;
constexpr auto Sheight = 1200;
constexpr auto deltaTIME = 50000; 		// us

enum DriveStatus {
	state1 = 0,
	state2 = 1,
	ahead = 2,
	turning = 3,
	exiting = 4
};

class Point {
public:
	double x;
	double y;
};

//车辆基类
class CarBase {
public:
	virtual void showCar(Point p) = 0;//绘制车辆，纯虚函数

public:
	double length;//长度
	double width;//宽度

	Point p0;//起始位置
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
	double RWidth = 100.0;   // 路宽
	double dot_distance = 8.0; // 虚线点的间隔
	// 道路上的控制车辆
	CarBase *car = new Car(RWidth);//多态：父类指针指向子类对象
	// 道路上的障碍车辆
	CarBase *carObs = new CarObs(RWidth);//多态：父类指针指向子类对象
};

//矩阵
class Mat {
public:
#if 0
	Mat();//存在有参构造时，要手写默认构造，才能创建不带参对象
	Mat(int mm, int nn);//手动创建矩阵时用
	void createMat();//手动创建矩阵
#endif

	void createMatT(double t0, double t1);//创建矩阵T
	void createVector(double tmp1, double tmp2, double tmp3, double tmp4, double tmp5, double tmp6);//创建向量
	void PrintMat();//打印矩阵

	void augMat(Mat a, Mat b);//求矩阵 a 和向量 b 的增广矩阵
	bool solve(Mat a, Mat b); //求线性方程组的解(ax=b ,求 x)，a 为方阵 ，b 为列向量

public:
	int m;
	int	n; //行数和列数
	double mat[N][N] = { 0 };  //矩阵元素初始化
};

//多项式求解曲线
class Polynomial {
public:
	Polynomial();
	void SetPangolinParams(pangolin::OpenGlRenderState *camera, pangolin::View *view);
	void process();							//整个过程
	void CarGoStraight();					//直行
	void showAheadTrack();					//绘制直行规划线轨迹
	void calMat();							//计算矩阵
	void showTurningTrack(int trackIndex);	//绘制多项式规划线轨迹
	void CarChangeLane();					//车辆换道移动
	void ShowResult();						//显示速度与加速度关于时间的曲线
public:
	Road road;

	double t0 = 0.0;
	double t1 = 3.0;
	double x0 = road.car->p0.x;
	double y0 = road.car->p0.y;
	double x1 = road.car->p1.x;
	double y1 = road.car->p1.y;
	double vx0 = 0.0;
	double vy0 = 120.0; // t0时y方向速度为120
	double vx1 = 0.0;
	double vy1 = 120.0; // t1时y方向速度为120
	double ax0 = 0.0;
	double ay0 = 0.0;
	double ax1 = 0.0;
	double ay1 = 0.0;

	Mat X;
	Mat Y;
	Mat T;
	Mat A;
	Mat B;

	double disS = road.car->length * 4;	//纵向换道距离
	double safeDis = 400.0;				//纵向安全距离
	int disAhead = safeDis / 50;		//直行规划线的间隔
	double speed = 5.0;   				//直行速度
	double delta_t = 0.05;				//时间间隔, s
	vector<Point> trackPoints;			//存储规划轨迹点
	int curTrackIndex = -1;
	DriveStatus state = state1;
	pangolin::OpenGlRenderState *camera_p = nullptr;
	pangolin::View *view_p = nullptr;
};
