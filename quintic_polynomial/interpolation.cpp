#include "interpolation.h"
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

	cout << "car loc: " << p0.x << ", " << p0.y << endl;
	cout << "car target loc: " << p1.x << ", " << p1.y << endl;
}

CarObs::CarObs(double Rw) {
	length = 100.0;
	width = 60.0;

	p0.x = Rw / 2;
	p0.y = Sheight / 2 + length * 4;

	p1 = p0;

	cout << "carObs loc: " << p0.x << ", " << p0.y << endl;
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

Road::~Road()
{
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

//创建矩阵T
void Mat::createMatT(double t0, double t1) {
	m = n = 6;
	for (int j = 1; j <= n; j++) {
		mat[1][j] = pow(t0, n - j);
		mat[2][j] = (n - j) * pow(t0, n - j - 1);
		mat[3][j] = (n - j) * (n - j - 1) * pow(t0, n - j - 2);

		if (n - j - 1 < 0) {
			mat[2][j] = 0.0;
		}

		if (n - j - 2 < 0) {
			mat[3][j] = 0.0;
		}
	}

	for (int j = 1; j <= n; j++) {
		mat[4][j] = pow(t1, n - j);
		mat[5][j] = (n - j) * pow(t1, n - j - 1);
		mat[6][j] = (n - j) * (n - j - 1) * pow(t1, n - j - 2);

		if (n - j - 1 < 0) {
			mat[5][j] = 0.0;
		}

		if (n - j - 2 < 0) {
			mat[6][j] = 0.0;
		}
	}
}

//创建向量
void Mat::createVector(double tmp1, double tmp2, double tmp3, double tmp4, double tmp5, double tmp6) {
	m = 6;
	n = 1;

	mat[1][1] = tmp1;
	mat[2][1] = tmp2;
	mat[3][1] = tmp3;
	mat[4][1] = tmp4;
	mat[5][1] = tmp5;
	mat[6][1] = tmp6;
}

//打印矩阵
void Mat::PrintMat() {
	for (int i = 1; i <= m; i++) {
		for (int j = 1; j <= n; j++) {
			cout << mat[i][j] << "\t";
		}
		cout << endl;
	}
	cout << endl;
}

//求矩阵 a 和向量 b 的增广矩阵
void Mat::augMat(Mat a, Mat b) {
	m = a.m;
	n = a.n + 1;						//列数+1
	for (int i = 1; i <= a.m; i++) {
		for (int j = 1; j <= a.n; j++) {
			mat[i][j] = a.mat[i][j];
		}
		mat[i][n] = b.mat[i][1];		//每行的最后一列，赋值为向量b在这一行的元素
	}
}

//求线性方程组的解(ax=b ,求 x)，a 为方阵 ，b 为列向量 //矩阵 a 为方阵并且方程组有唯一解时返回 true
bool Mat::solve(Mat a, Mat b) {
	if (a.n != a.m)	{	//只求解是方阵时的情形
		cout << "系数矩阵不是方阵" << endl;
		return false;
	}

	m = a.n;
	n = 1; //解向量中必定有 a.n（ a.m ）个分量,是 a.n * 1 的列向量

	Mat aa;
	aa.augMat(a, b); //求增广矩阵
	cout << "增广矩阵是：" << endl;
	aa.PrintMat();

	//下面代码将增广矩阵化为上三角矩阵，并判断增广矩阵秩是否为 n
	for (int i = 1; i <= aa.m; i++) {
		//寻找第 i 列不为零的元素
		int k;
		for (k = i; k <= aa.m; k++) {
			if (fabs(aa.mat[k][i]) > 1e-10) {  //满足这个条件时，认为这个元素不为0
				break;
			}
		}

		if (k <= aa.m) {  //说明第 i 列有不为0的元素
			//交换第 i 行和第 k 行所有元素
			for (int j = i; j <= aa.n; j++) { //从第 i 个元素交换即可，因为前面的元素都为0
				aa.mat[0][j] = aa.mat[i][j]; aa.mat[i][j] = aa.mat[k][j]; aa.mat[k][j] = aa.mat[0][j];//使用aa.mat[0][j]作为中间变量交换元素
			}
			double c;//倍数
			for (int j = i + 1; j <= aa.m; j++) {
				c = -aa.mat[j][i] / aa.mat[i][i];
				for (k = i; k <= aa.n; k++) {
					aa.mat[j][k] += c * aa.mat[i][k];//第 i 行 a 倍加到第 j 行
				}
			}
		}
		else {   //没有找到则说明系数矩阵秩不为 n ，说明方程组中有效方程的个数小于 n
			cout << "系数矩阵奇异，线性方程组无解或有无数解" << endl;
			return false;
		}
	}

	//自下而上求解
	for (int i = a.m; i >= 1; i--) {
		mat[i][1] = aa.mat[i][aa.n];
		for (int j = a.m; j > i; j--) {
			mat[i][1] -= mat[j][1] * aa.mat[i][j];
		}
		mat[i][1] /= aa.mat[i][i];
	}
	return true;
}

Polynomial::Polynomial() {
	//如果是行进过程中换道，就在决定要换道的时候再计算矩阵；
	showAheadTrack();
}

//车辆直行，当status为DriveStatus::ahead时执行
void Polynomial::CarGoStraight() {
	if (state != DriveStatus::ahead || camera_p == nullptr || view_p == nullptr) {
		return;
	}
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	view_p->Activate(*camera_p);
	// 更新道路上car的位置并显示
	road.car->p0.y += speed;
	road.car->p1.y += speed;
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
void Polynomial::showAheadTrack() {
	glBegin(GL_POINTS);
	glColor3f(0.f, 0.f, 0.8f); // R,G,B
	Point pt = road.car->p0;
	for (int i = 0; i < 50; i++) {
		pt.y += disAhead;
		glVertex3f(pt.x, pt.y, 3);
	}
	glEnd();	// 结束绘图
}

//计算矩阵
void Polynomial::calMat() {
	x0 = road.car->p0.x;
	y0 = road.car->p0.y;

	road.car->p1.x = 0. - road.RWidth / 2;
	road.car->p1.y = road.car->p0.y + disS;

	x1 = road.car->p1.x;
	y1 = road.car->p1.y;
	cout << x0 << ", " << y0 << ", " << x1 << ", " << y1 << endl;

	X.createVector(x0, vx0, ax0, x1, vx1, ax1);
	Y.createVector(y0, vy0, ay0, y1, vy1, ay1);
	T.createMatT(t0, t1);
	cout << endl;
	cout << "X向量：" << endl;
	X.PrintMat();
	cout << "Y向量：" << endl;
	Y.PrintMat();
	cout << "T矩阵：" << endl;
	T.PrintMat();

	if (A.solve(T, X)) {   // T * A = X
		cout << "求得的A向量如下：" << endl;
		A.PrintMat();
	}

	if (B.solve(T, Y)) {   // T * B = Y
		cout << "求得的B向量如下：" << endl;
		B.PrintMat();
	}
}

//绘制多项式轨迹
void Polynomial::showTurningTrack(int trackIndex) {
	if (trackIndex < 0 || trackIndex >= trackPoints.size()) {
		return;
	}
	glBegin(GL_POINTS);
	glColor3f(0.f, 0.f, 1.0f); // R,G,B
	for (int i = trackIndex; i < trackPoints.size(); ++i) {
		glVertex3f(trackPoints.at(i).x, trackPoints.at(i).y, 3);
	}
	glEnd();	// 结束绘图
}

//车辆换道移动，只在DriveStatus::turning状态运行
void Polynomial::CarChangeLane() {
	if (state != DriveStatus::turning || camera_p == nullptr || view_p == nullptr) {
		return;
	}
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	view_p->Activate(*camera_p);

	if (curTrackIndex == -1) { // 刚开始进入转弯状态时，计算轨迹
		calMat();//计算矩阵
		for (double t = t0; t <= t1; t += delta_t) {    // 根据五次多项式计算轨迹
			double x_tmp = A.mat[1][1] * pow(t, 5) + A.mat[2][1] * pow(t, 4) + A.mat[3][1] * pow(t, 3) + A.mat[4][1] * pow(t, 2) + A.mat[5][1] * pow(t, 1) + A.mat[6][1];
			double y_tmp = B.mat[1][1] * pow(t, 5) + B.mat[2][1] * pow(t, 4) + B.mat[3][1] * pow(t, 3) + B.mat[4][1] * pow(t, 2) + B.mat[5][1] * pow(t, 1) + B.mat[6][1];
			Point ptmp;
			ptmp.x = x_tmp;
			ptmp.y = y_tmp;
			trackPoints.push_back(ptmp);
		}
	}
	curTrackIndex++;
	// cout << "curTrackIndex = " << curTrackIndex << ", trackPoints.size() = " << trackPoints.size() << endl;
	if (curTrackIndex == trackPoints.size()) {
		state = DriveStatus::ahead;
	}
	else {
		// 绘制转弯轨迹
		showTurningTrack(curTrackIndex);
		auto &curLocation = trackPoints.at(curTrackIndex);
		road.showRoad(curLocation);
		//实时更新p0位置
		road.car->p0.x = curLocation.x;
		road.car->p0.y = curLocation.y;
	}
	usleep(deltaTIME);
}

//显示速度与加速度关于时间的曲线
void Polynomial::ShowResult() {
	if (state != DriveStatus::exiting || camera_p == nullptr || view_p == nullptr) {
		return;
	}
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	pangolin::OpenGlRenderState camera(
			// 图像宽度、高度、4个内参fx fy cx cy以及最近、最远视距
			pangolin::ProjectionMatrix(Swidth, Sheight, 1500, 1500, 0, 0, 0.1, 2500),
			/*相机的初始坐标，相机视点的初始位置（即相机光轴朝向），相机轴方向*/
        	pangolin::ModelViewLookAt(Swidth/2.0, 0.0, 1800, 0, Sheight/2, 0, pangolin::AxisY));
	view_p->Activate(camera);
	DrawLine2D(0, 0, 0,1000,1,1,1);
	DrawLine2D(0, 0, 1000,0,1,1,1);

	// 同时绘制x-t和y-t速度曲线
	for (double t = t0; t <= t1; t += delta_t) {
		double vx_tmp = 5 * A.mat[1][1] * pow(t, 4) + 4 * A.mat[2][1] * pow(t, 3) + 3 * A.mat[3][1] * pow(t, 2) + 2 * A.mat[4][1] * pow(t, 1) + A.mat[5][1];
		double vy_tmp = 5 * B.mat[1][1] * pow(t, 4) + 4 * B.mat[2][1] * pow(t, 3) + 3 * B.mat[3][1] * pow(t, 2) + 2 * B.mat[4][1] * pow(t, 1) + B.mat[5][1];
		glColor3f(1.0, 0., 0.);
		glPointSize(5.0);
		glBegin(GL_POINTS);
		glVertex3f(t * 100, vx_tmp, 10);
		glVertex3f(t * 100, vy_tmp, 10);
		glEnd();
	}
	// 同时绘制x-t和y-t加速度曲线
	for (double t = t0; t <= t1; t += delta_t) {
		double ax_tmp = 20 * A.mat[1][1] * pow(t, 3) + 12 * A.mat[2][1] * pow(t, 2) + 6 * A.mat[3][1] * pow(t, 1) + 2 * A.mat[4][1];
		double ay_tmp = 20 * B.mat[1][1] * pow(t, 3) + 12 * B.mat[2][1] * pow(t, 2) + 6 * B.mat[3][1] * pow(t, 1) + 2 * B.mat[4][1];
		glColor3f(0., 1., 0.);
		glPointSize(5.0);
		glBegin(GL_POINTS);
		glVertex3f(t * 100 + 350, ax_tmp, 10);
		glVertex3f(t * 100 + 350, ay_tmp, 10);
		glEnd();
	}
	pangolin::FinishFrame();
}

// 整个过程
void Polynomial::process() {
	if (this->state == DriveStatus::ahead) {
		CarGoStraight();
	}
	if (this->state == DriveStatus::turning) {
		CarChangeLane();
	}
	if (this->state == DriveStatus::exiting) {
		ShowResult();
	}
}

void Polynomial::SetPangolinParams(pangolin::OpenGlRenderState *camera, pangolin::View *view) {
	camera_p = camera;
	view_p = view;
}

/**
 * @brief 五次多项式例程
 * 参考博客：https://zhuanlan.zhihu.com/p/546222927
 * 图形化工具从easyX改为pangolin,在Ubuntu系统下开发、验证, 采用状态机控制整个仿真过程
 * 五次多项式，最多能确定每个期望点的位置、速度和加速度；
 * 五次多项式是指横向/纵向位置关于时间t的五次多项式，而不是横向位置y关于纵向位置x的五次多项式！
 *
 * x(t) = a0 + a1t + a2t^2 + a3t^3 + a4t^4 + a5t^5; //纵向位置
 * y(t) = b0 + b1t + b2t^2 + b3t^3 + b4t^4 + b5t^5; //横向位置
 *
 * 假设起点、终点的位置、速度（一阶导）、加速度（二阶导）均已知，有已下方程：
 *
 * x(t0) = a0 + t0a1 + t0^2 a2 + t0^3 a3 + t0^4 a4 + t0^5 a5;
 * y(t0) = b0 + t0b1 + t0^2 b2 + t0^3 b3 + t0^4 b4 + t0^5 b5;
 * x'(t0) = a1 + 2t0a2 + 3t0^2 a3 + 4t0^3 a4 + 5t0^4 a5;
 * y'(t0) = b1 + 2t0b2 + 3t0^2 b3 + 4t0^3 b4 + 5t0^4 b5;
 * x''(t0) = 2a0 + 6t0a3 + 12t0^2 a4 + 20t0^3 a5;
 * y''(t0) = 2b0 + 6t0b3 + 12t0^2 b4 + 20t0^3 b5;
 * x(t1) = a0 + t1a1 + t1^2 a2 + t1^3 a3 + t1^4 a4 + t1^5 a5;
 * y(t1) = b0 + t1b1 + t1^2 b2 + t1^3 b3 + t1^4 b4 + t1^5 b5;
 * x'(t1) = a1 + 2t1a2 + 3t1^2 a3 + 4t1^3 a4 + 5t1^4 a5;
 * y'(t1) = b1 + 2t1b2 + 3t1^2 b3 + 4t1^3 b4 + 5t1^4 b5;
 * x''(t1) = 2a0 + 6t1a3 + 12t1^2 a4 + 20t1^3 a5;
 * y''(t1) = 2b0 + 6t1b3 + 12t1^2 b4 + 20t1^3 b5;
 *
 * 共12个方程，其中12个未知数（a0 - a5, b0 - b5），可以通过线性方程组求出唯一解，
 * 即： X = A * T，Y = B * T； 求解A、B。
 * 注意：五次多项式插值速度曲线并没有匀速段，因此这种规划方式不适用于笛卡尔空间的要求匀速的场合的规划。
 *
 */
int main() {
	// 初始化图形窗口
    pangolin::CreateWindowAndBind("interpolation", Swidth, Sheight);
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

	Polynomial poly;
	poly.SetPangolinParams(&camera, &view);
	poly.state = DriveStatus::ahead;
    while(!pangolin::ShouldQuit()) {
		// Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        view.Activate(camera);

        // draw the original axis
		glLineWidth(3);
		glColor3f(0.8f, 0.f, 0.f); // R,G,B
		glBegin(GL_LINES);
		// x
		glVertex3f(0, 0, 0);
		glVertex3f(10, 0, 0);
		glColor3f(0.f, 0.8f, 0.f);
		// y
		glVertex3f(0, 0, 0);
		glVertex3f(0, 10, 0);
		glColor3f(0.2f, 0.2f, 1.f);
		// z
		glVertex3f(0., 0., 0.);
		glVertex3f(0., 0., 10.);
		glEnd();

		poly.process();

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }
	return 0;
}
