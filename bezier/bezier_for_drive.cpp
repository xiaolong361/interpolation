#include "bezier_for_drive.h"
#include <unistd.h>
#include <pangolin/pangolin.h>

static void DrawLine2D(double x1, double y1, double x2, double y2, float r, float g, float b) {
	glColor3f(r, g, b);
	glBegin(GL_LINES);
	glVertex3f(x1, y1, 0);
	glVertex3f(x2, y2, 0);
	glEnd();
}

static void DrawSimpleRectangle(double left, double top, double right, double bottom, float r, float g, float b) {
	glColor3f(r, g, b);
	glBegin(GL_LINES);
	glVertex3f(left, bottom, 0);
	glVertex3f(right, bottom, 0);

	glVertex3f(right, bottom, 0);
	glVertex3f(right, top, 0);

	glVertex3f(right, top, 0);
	glVertex3f(left, top, 0);

	glVertex3f(left, top, 0);
	glVertex3f(left, bottom, 0);
	glEnd();
}

/**
 * @brief Construct a new Car:: Car object
 *
 * @param Rw car所在的宽度位置（直行时不变）
 */
Car::Car(double Rw) {
	length = 100.0;
	width = 60.0;

	p0.x = Rw / 2;
	p0.y = length;

	p1 = p0;

	std::cout << "car loc: " << p0.x << ", " << p0.y << std::endl;
	std::cout << "car target loc: " << p1.x << ", " << p1.y << std::endl;
}

CarObs::CarObs(double Rw) {
	length = 100.0;
	width = 60.0;

	p0.x = Rw / 2;
	p0.y = Sheight / 2 + length * 4;

	std::cout << "carObs loc: " << p0.x << ", " << p0.y << std::endl;
}

//绘制车辆
void Car::showCar(Point p) {
	double left = p.x - width / 2;
	double right = p.x + width / 2;
	double top = p.y + length / 2;
	double bottom = p.y - length / 2;
	DrawSimpleRectangle(left, top, right, bottom, 1.0, 0.0, 0.0);
}

//绘制障碍车辆
void CarObs::showCar(Point p) {
	double left = p.x - width / 2;
	double right = p.x + width / 2;
	double top = p.y + length / 2;
	double bottom = p.y - length / 2;
	DrawSimpleRectangle(left, top, right, bottom, 1.0, 0.0, 0.0);
}

Road::Road() {
	showRoad(car->p0);
}

Road::~Road() {
	if (car != nullptr) {
		delete car;
		car = nullptr;
	}

	if (carObs != nullptr) {
		delete car;
		carObs = nullptr;
	}
}

/**
 * @brief 绘制道路
 *
 * @param p 道路上car的位置
 */
void Road::showRoad(Point p) {
	//绘制道路中心
	glBegin(GL_POINTS);
	glColor3f(0.f, 0.8f, 0.f); // R,G,B
	int point_num = Sheight / dot_distance + 1;
	double start_y = 0;
	for (int i = 0; i < point_num; i++) {
		start_y += dot_distance;
		glVertex3f(0, start_y, 3);
	}
	glEnd();	// 结束绘图

	//绘制左右边界
	glBegin(GL_LINES);
	glLineWidth(3);//设置线宽
	DrawLine2D(0. - RWidth, 0.0,
			 0. - RWidth, Sheight,
			 0.f,0.8f,0.f);
	DrawLine2D(RWidth, 0.0,
			 RWidth, Sheight,
			 0.f,0.8f,0.f);

	car->showCar(p);
	carObs->showCar(carObs->p0);
}

Bezier::Bezier() {
	//如果是行进过程中换道，就在决定要换道的时候再调用贝塞尔函数
	showAheadTrack();
}

Bezier::~Bezier() {
	trackPoints.clear();
}

void Bezier::SetPangolinParams(pangolin::OpenGlRenderState *camera, pangolin::View *view) {
	camera_p = camera;
	view_p = view;
}

//车辆直行，当status为DriveStatus::ahead时执行
void Bezier::CarGoStraight() {
	if (state != DriveStatus::ahead || camera_p == nullptr || view_p == nullptr) {
		return;
	}
	std::cout << "直行中" << std::endl;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	view_p->Activate(*camera_p);
	// 更新道路上car的位置并显示
	road.car->p0.y += speed;
	// road.car->p1.y += speed;
	showAheadTrack();
	road.showRoad(road.car->p0);
	glEnd();	// 结束绘图

	if (abs(road.car->p0.x - road.carObs->p0.x) <= road.RWidth/2.0 &&
			abs(road.car->p0.y - road.carObs->p0.y) <= safeDis) {
		this->state = DriveStatus::turning;
	}
	if (road.car->p0.y > Sheight) {
		this->state = DriveStatus::exiting;
	}
	usleep(deltaTIME);
}

//绘制直行规划线轨迹
void Bezier::showAheadTrack() {
	glBegin(GL_POINTS);
	glColor3f(1.0, 1.0f, 0.f); // R,G,B
	Point pt = road.car->p0;
	for (int i = 0; i < 50; i++) {
		pt.y += disAhead;
		glVertex3f(pt.x, pt.y, 5);
	}
	glEnd();	// 结束绘图
}

//设置控制点
void Bezier::setControlPoint() {
	// 曲线起点
	p0 = road.car->p0;
	// 曲线终点
	p3.x = -1 * road.RWidth / 2;
	p3.y = p0.y + disS;

	// p1控制点：从起点沿车道方向向前，距离为起点终点纵向距离的1/2
	p1.x = p0.x;
	p1.y = p0.y + disS / 2;
	// p2控制点：从终点沿车道方向向后，距离为起点终点纵向距离的1/2
	p2.x = p3.x;
	p2.y = p1.y;
}

//绘制贝塞尔曲线轨迹
void Bezier::showBezierTrack(int trackIndex) {
	if (trackIndex < 0 || trackIndex >= trackPoints.size()) {
		return;
	}
	glBegin(GL_POINTS);
	glColor3f(1.0f, 1.0f, 0.f); // R,G,B
	for (int i = trackIndex; i < trackPoints.size(); ++i) {
		glVertex3f(trackPoints.at(i).x, trackPoints.at(i).y, 5);
	}
	glEnd();	// 结束绘图
}

//车辆换道移动，只在DriveStatus::turning状态运行
void Bezier::CarChangeLane() {
	if (state != DriveStatus::turning || camera_p == nullptr || view_p == nullptr) {
		return;
	}
	std::cout << "变道中" << std::endl;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	view_p->Activate(*camera_p);

	if (curTrackIndex == -1) { // 刚开始进入转弯状态时，计算轨迹
		setControlPoint();
		for (double t = 0.0; t <= 1.0; t += delta_t) {
			//这里由于已经确定了用三阶贝塞尔，所以直接套公式，不用递归计算，节省资源；
			Point ptmp;
			ptmp.x = pow(1 - t, 3) * p0.x + 3 * t * pow(1 - t, 2) * p1.x + 3 * pow(t, 2) * (1 - t) * p2.x + pow(t, 3) * p3.x;
			ptmp.y = pow(1 - t, 3) * p0.y + 3 * t * pow(1 - t, 2) * p1.y + 3 * pow(t, 2) * (1 - t) * p2.y + pow(t, 3) * p3.y;
			trackPoints.push_back(ptmp);
		}
	}
	curTrackIndex++;
	if (curTrackIndex == trackPoints.size()) {
		state = DriveStatus::ahead;
	}
	else {
		// 绘制转弯轨迹
		showBezierTrack(curTrackIndex);
		auto &curLocation = trackPoints.at(curTrackIndex);
		road.showRoad(curLocation);
		//实时更新p0位置
		road.car->p0.x = curLocation.x;
		road.car->p0.y = curLocation.y;
	}
	usleep(deltaTIME);
}

//整个过程
void Bezier::process() {
	if (this->state == DriveStatus::ahead) {
		CarGoStraight();
	}
	if (this->state == DriveStatus::turning) {
		CarChangeLane();
	}
	if (this->state == DriveStatus::exiting) {
		// ShowResult();
	}
}

/**
 * @brief 贝塞尔曲线例程
 * 参考博客：https://zhuanlan.zhihu.com/p/548820614
 * 图形化工具从easyX改为pangolin,在Ubuntu系统下开发、验证, 采用状态机控制整个仿真过程
 * 对于无人驾驶，规划的轨迹应满足以下要求：
 * (1)轨迹连续；
 * (2)轨迹曲率连续；
 * (3)轨迹容易被车辆跟随，较贴合车辆运动学约束；
 * (4)轨迹容易生成；
 *
 * 通常无人驾驶在换道、绕障时的局部路径规划中，可以使用贝塞尔曲线，且为了能满足上述要求，通常是三阶贝塞尔曲线；
 * 贝塞尔曲线主要性质：
 * 性质1：P0和Pn分别位于贝塞尔曲线的起点和终点；
 * 性质2：几何特性不随坐标系的变换而变化；
 * 性质3：起点和终点处的切线方向与和特征多边形的第一条边及最后一条边分别重合；
 * 性质4：至少需要三阶贝塞尔曲线（四个控制点）才能生成曲率连续的轨迹。
 *
 */
int main() {
	// 初始化图形窗口
    pangolin::CreateWindowAndBind("bezier_for_drive", Swidth, Sheight);
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState camera(
			// 图像宽度、高度、4个内参fx fy cx cy以及最近、最远视距
			pangolin::ProjectionMatrix(Swidth, Sheight, 1500, 1500, Swidth/2, Sheight/2, 0.1, 2500),
			/*相机的初始坐标，相机视点的初始位置（即相机光轴朝向），相机轴方向*/
        	pangolin::ModelViewLookAt(0, Sheight/2, 1800, 0, Sheight/2, 0, pangolin::AxisY));

    // Create Interactive View in window
    pangolin::Handler3D handler(camera);
    pangolin::View& view = pangolin::CreateDisplay()
			/*底、顶、左、右、显示长宽比//视图在视窗中的范围*/
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1.f * Swidth/Sheight)
			/*handler绑定渲染状态*/
            .SetHandler(&handler);

	Bezier bezier;
	bezier.SetPangolinParams(&camera, &view);
	bezier.state = DriveStatus::ahead;
	while(!pangolin::ShouldQuit()) {
		// Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        view.Activate(camera);

		bezier.process();

        // draw the original axis
		glLineWidth(3);
		glBegin(GL_LINES);
		// x
		glColor3f(0.8f, 0.f, 0.f); // R,G,B
		glVertex3f(0, 0, 0);
		glVertex3f(50, 0, 0);
		// y
		glColor3f(0.f, 0.8f, 0.f);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 50, 0);
		// z
		glColor3f(0.2f, 0.2f, 1.f);
		glVertex3f(0., 0., 0.);
		glVertex3f(0., 0., 50.);
		glEnd();

		// Swap frames and Process Events
        pangolin::FinishFrame();
    }
	return 0;
}
