#include<iostream>
#include<opencv2/opencv.hpp>
#include<OsqpEigen/OsqpEigen.h>
#include<Eigen/Core>

#ifdef _DEBUG
#pragma comment(lib,"opencv_world348d.lib")
#pragma comment(lib,"../lib/Debug/OsqpEigend.lib")
#pragma comment(lib,"../lib/Debug/osqp.lib")
#else
#pragma comment(lib,"opencv_world348.lib")
#pragma comment(lib,"../lib/Release/OsqpEigen.lib")
#pragma comment(lib,"../lib/Release/osqp.lib")
#endif
using namespace std;
using namespace cv;
float g_speed, g_acc, g_aacc;
vector<Point>path;
Mat show;
int thresholds = 10;
int thresholds1 = 50;
int thresholds2 = 50;
void onMouse(int event, int x, int y, int flags, void* param)
{
	Mat* im = reinterpret_cast<Mat*>(param);
	switch (event)
	{
	case CV_EVENT_LBUTTONDOWN:     //鼠标左键按下响应：返回坐标和灰度
		circle(show, Point(x, y), 2, Scalar(255, 255, 255));
 		path.push_back(Point(x, y));
		break;
	}
}
void GetPoint(Point from, Point to, vector<Point>& dst) {
	float dx = to.x - from.x;
	float dy = to.y - from.y;
	if (fabsf(dx) <= 1) {
		if (dst.size() > 0 && from.x == dst[dst.size() - 1].x)return;
		dst.push_back(from);
		return;
	}
	int step = fabsf(dx);
	float stepx = dx / step;
	float stepy = dy / step;
	for (int i = 0; i < step; i++) {
		int x = from.x + i * stepx;
		int y = from.y + i * stepy + 0.5;
		if (dst.size() > 0 && x == dst[dst.size() - 1].x)continue;
		dst.push_back(Point(x, y));
	}
}
static float Square(float f) { return (float)f * f; };
static float Three(float f) { return (float)f * f * f; };
void Insert_Square_Bezier(vector<Point2f>& q, Point2f p0, Point2f p1, Point2f p2, float n) {
	float  f = fabs(1.0 / n);
	for (float t = f; t <= 1.0; t += f) {
		float x1 = Square(1 - t) * p0.x + 2 * t * (1 - t) * p1.x + Square(t) * p2.x;
		float y1 = Square(1 - t) * p0.y + 2 * t * (1 - t) * p1.y + Square(t) * p2.y;
		if (q.size() > 0 && x1 == q[q.size() - 1].x && y1 == q[q.size() - 1].y)continue;
		q.push_back(Point2f(x1, y1));
	}
}
void Insert_Three_Bezier(vector<Point2f>& q, Point2f p0, Point2f p1, Point2f p2,Point2f p3, float n) {
    float  f = fabs(1.0 / n);
	for (float t = f; t <= 1.0; t += f) {
		float x1 = Three(1 - t) * p0.x + 3 * t * Square(1 - t) * p1.x + 3 * (1 - t)* Square(t) * p2.x + Three(t) * p3.x;
		float y1 = Three(1 - t) * p0.y + 3 * t * Square(1 - t) * p1.y + 3 * (1 - t) * Square(t)*  p2.y + Three(t) * p3.y;
		if (q.size() > 0 && x1 == q[q.size() - 1].x && y1 == q[q.size() - 1].y)continue;
		q.push_back(Point2f(x1, y1));
	}
}
void DealWithPathBySquareBezier(vector<Point>Org, vector<Point>& dst) {
	if (Org.size() == 0)return;
	vector<Point2f>arrPoint(110);
	Point2f lastPoint;
	arrPoint.clear();
	if (Org.size() <= 1) {
		for (int i = 0; i < Org.size(); i++) {
			arrPoint.push_back(Org[i]);
		}
	}
	else {
		Point p0, p1, p2;
		p0 = Org[0];
		p1 = Org[0] + Point(thresholds, 0);
		float f;
		for (int i = 1; i < Org.size() - 1; i ++) {
			p2 = (Org[i] + p1) / 2;
			f = (p2.x - p0.x) / 2;
			Insert_Square_Bezier(arrPoint, p0, p1, p2, f);
			p0 = p2;
			p1 = Org[i];
		}
		p2 = Org[Org.size() - 1];
		f = (p2.x - p0.x) / 2;
		Insert_Square_Bezier(arrPoint, p0, p1, p2, f);
	}
	for (int i = 1; i < arrPoint.size(); i++) {
		Point from, end;
		from.x = arrPoint[i - 1].x;
		from.y = arrPoint[i - 1].y + 0.5;
		end.x = arrPoint[i].x;
		end.y = arrPoint[i].y + 0.5;
		GetPoint(from, end, dst);
	}
	for (int i = 1; i < int(dst.size()) - 2; i++) {
		if (dst[i - 1].y == dst[i + 1].y) {
			dst[i].y = dst[i - 1].y;
		}
	}
}
void DealWithPathByThreeBezier(vector<Point>Org, vector<Point>& dst) {
	if (Org.size() == 0)return;
	vector<Point2f>arrPoint(110);
	Point2f lastPoint;
	arrPoint.clear();
	if (Org.size() <= 1) {
		for (int i = 0; i < Org.size(); i++) {
			arrPoint.push_back(Org[i]);
		}
	}
	else if (Org.size() == 2) {
		float f = (Org[1].x - Org[0].x)/2;
		Insert_Square_Bezier(arrPoint, Org[0], Org[0] + Point(10,0), Org[1], f);
	}
	else {
		Point p0, p1, p2, p3;
		p0 = Org[0];
		p1 = Org[0] + Point(thresholds, 0);
		p2 = Org[1];
		float f;
		for (int i = 2; i < Org.size(); i+=2) {
			p2 = Org[i - 1];
			p3 = (Org[i] + p2) / 2;
			f = (p3.x - p0.x) / 2;
			Insert_Three_Bezier(arrPoint, p0, p1, p2, p3, f);
			p0 = p3;
			p1 = Org[i];
		}
		p2 = Org[Org.size() - 1];
		f = (p2.x - p0.x)/2;
		Insert_Square_Bezier(arrPoint, p0, p1, p2, f);
	}
	for (int i = 1; i < arrPoint.size(); i++) {
		Point from, end;
		from.x = arrPoint[i - 1].x;
		from.y = arrPoint[i - 1].y + 0.5;
		end.x = arrPoint[i].x;
		end.y = arrPoint[i].y + 0.5;
		GetPoint(from, end, dst);
	}
	for (int i = 1; i < int(dst.size()) - 2; i++) {
		if (dst[i - 1].y == dst[i + 1].y) {
			dst[i].y = dst[i - 1].y;
		}
	}
}
Mat polyfit(vector<Point>& in_point, int n)
{
	int size = in_point.size();
	//所求未知数个数
	int x_num = n + 1;
	//构造矩阵U和Y
	Mat mat_u(size, x_num, CV_64F);
	Mat mat_y(size, 1, CV_64F);

	for (int i = 0; i < mat_u.rows; ++i)
		for (int j = 0; j < mat_u.cols; ++j)
		{
			mat_u.at<double>(i, j) = pow(in_point[i].x, j);
		}

	for (int i = 0; i < mat_y.rows; ++i)
	{
		mat_y.at<double>(i, 0) = in_point[i].y;
	}

	//矩阵运算，获得系数矩阵K
	Mat mat_k(x_num, 1, CV_64F);
	mat_k = (mat_u.t() * mat_u).inv() * mat_u.t() * mat_y;
	return mat_k;
}

void speed(int, void*)
{
}
void th1(int, void*)
{
}
void th2(int, void*)
{
}


//(x,y)
using Point2D = std::pair<double, double>;
using namespace std;

struct spline_curve
{
    double a0;
    double a1;
    double a2;
    double a3;
    double a4;
    double a5;
    double b0;
    double b1;
    double b2;
    double b3;
    double b4;
    double b5;
    double x_start;
    double y_start;
    double x_end;
    double y_end;
};

bool readCSVInput(int N, vector<double>& input_points_x, vector<double>& input_points_y, vector<double>& anchor_points_x, vector<double>& anchor_points_y)
{
    ifstream readCSV("../input_data.csv", ios::in);
    int lineNum = 0;
    string lineString;
    vector<vector<string>> stringArray;
    while (getline(readCSV, lineString))
    {
        std::cout << lineString << endl;
        if (lineNum > 0)
        {
            stringstream ss(lineString);
            string s;
            vector<string> oneLineStrings;
            while (getline(ss, s, ','))
                oneLineStrings.push_back(s);
            if (oneLineStrings.size() != 5)
            {
                std::cout << "ERROR:oneLineStrings.size() != 5" << endl;
                return false;
            }
            else
            {
                input_points_x.push_back(std::stod(oneLineStrings[1]));
                input_points_y.push_back(std::stod(oneLineStrings[2]));
                anchor_points_x.push_back(std::stod(oneLineStrings[3]));
                anchor_points_y.push_back(std::stod(oneLineStrings[4]));
            }
        }
        lineNum++;
    }
    if (N == input_points_x.size())
    {
        return true;
    }
    else
    {
        input_points_x.clear();
        input_points_y.clear();
        anchor_points_x.clear();
        anchor_points_y.clear();
        std::cout << "ERROR:N == input_points_x.size()" << endl;
        return false;
    }
}

bool writeCSVOutput(Eigen::VectorXd const& QPSolution)
{
    ofstream writeCSV;
    writeCSV.open("../src/QPSolution.csv", ios::out);
    writeCSV << "QPSolution" << endl;
    for (int i = 1; i < QPSolution.size(); i++)
    {
        writeCSV << QPSolution[i - 1] << endl;
    }
    writeCSV << QPSolution[QPSolution.size() - 1];
    writeCSV.close();
    cout << "INFO: csv output." << endl;
    return true;
}
cv::Vec3b color[6] = {
    {0,0,255},
    {0,255,0},
    {255,0,0},
    {0,255,255},
    {255,255,0},
    {255,0,255},
};
OsqpEigen::Solver solver;
int OSQPSolverTest(vector<Point>path, vector<Point>& outPath)
{
    Mat tmp = Mat::zeros(1000, 1000, CV_8UC3);
    int N = path.size(); //input point number
    int sampling_num = N;
    Point2D start_point(0, 0);
    vector<double> anchor_points_x;
    vector<double> anchor_points_y;
    for (int i = 0; i < path.size(); i++) {
        anchor_points_x.push_back(path[i].x - path[0].x);
        anchor_points_y.push_back(path[i].y - path[0].y);
    }
    vector<Point2D> anchor_points;
    double ref_tan_start = (thresholds1 - 50) / 10 + 0.00001;// 0 * c[0] + 1 * c[1] + 2 * c[2] * start_point.second + 3 * c[3] * start_point.second * start_point.second;
    double ref_tan_end = (thresholds2 - 50) / 10 + 0.00001;// 0 * c[0] + 1 * c[1] + 2 * c[2] * (start_point.second + (N - 1) * interval) + 3 * c[3] * pow(start_point.second + (N - 1) * interval, 2);

    double bounding_box_constraint = 1e-7;
    Point2D temp_point;
    for (int i = 0; i < sampling_num; i++)
    {
        temp_point.first = anchor_points_x[i];
        temp_point.second = anchor_points_y[i];
        anchor_points.push_back(temp_point);
        circle(tmp, Point(temp_point.first + path[0].x, temp_point.second + path[0].y), 2, Scalar(0, 255, 255));
    }
    //至此得到输入离散点列与位置约束的原始线上参考点(anchor points)

    Eigen::SparseMatrix<double> hessian(6 * N, 6 * N);
    for (int i = 0; i < hessian.rows() / 6; i++)
    {
        hessian.insert(6 * i + 3, 6 * i + 3) = 36;
        hessian.insert(6 * i + 3, 6 * i + 4) = 72;
        hessian.insert(6 * i + 3, 6 * i + 5) = 120;
        hessian.insert(6 * i + 4, 6 * i + 3) = 72;
        hessian.insert(6 * i + 4, 6 * i + 4) = 192;
        hessian.insert(6 * i + 4, 6 * i + 5) = 360;
        hessian.insert(6 * i + 5, 6 * i + 3) = 120;
        hessian.insert(6 * i + 5, 6 * i + 4) = 360;
        hessian.insert(6 * i + 5, 6 * i + 5) = 720;
    }
    Eigen::SparseMatrix<double> R(6 * N, 6 * N);
    for (int i = 0; i < R.rows(); i++)
    {
        R.insert(i, i) = 1.0e-5;
    }
    Eigen::SparseMatrix<double> H(6 * N, 6 * N);
    H = hessian + R;
    int smooth_constraint_num = 4 * (N - 1);
    int position_constraint_num = N;
    int start_end_constraint_num = 2;
    int total_constraint_num = smooth_constraint_num + position_constraint_num + start_end_constraint_num;
    Eigen::VectorXd lowerBound(total_constraint_num);
    Eigen::VectorXd upperBound(total_constraint_num);
    Eigen::SparseMatrix<double> A(total_constraint_num, 6 * N); //所有约束的矩阵
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(6 * N);
    //光滑性约束共4*(N-1)*2个,由等式化为2个不等式约束
    Eigen::VectorXd T0(12);    //0阶导数向量
    Eigen::VectorXd T0_05(12); //0阶导数向量参数0.5时
    Eigen::VectorXd T1(12);    //1阶导数向量
    Eigen::VectorXd T2(12);    //2阶导数向量
    Eigen::VectorXd T3(12);    //3阶导数向量
    T0 << 1, 1, 1, 1, 1, 1, -1, 0, 0, 0, 0, 0;
    T1 << 0, 1, 2, 3, 4, 5, 0, -1, 0, 0, 0, 0;
    T2 << 0, 0, 2, 6, 12, 20, 0, 0, -2, 0, 0, 0;
    T3 << 0, 0, 0, 6, 24, 60, 0, 0, 0, -6, 0, 0;
    T0_05 << 1, 0.5, pow(0.5, 2), pow(0.5, 3), pow(0.5, 4), pow(0.5, 5), 0, 0, 0, 0, 0, 0;
    int constraintIndex = 0;
    double dx1[50][6];
    for (int i = 0; i < N - 1; i++) {
        dx1[i][1] = path[i + 1].x - path[i].x;
        for (int j = 2; j <= 5; j++) {
            dx1[i][j] = dx1[i][1]* dx1[i][j-1];
        }
    }
    //T0光滑约束-等式约束
    for (int i = 0; i < N - 1; ++i)
    {
        //向量ai
        A.insert(constraintIndex, 6 * i + 0) = T0[0];
        A.insert(constraintIndex, 6 * i + 1) = T0[1] * dx1[i][1];
        A.insert(constraintIndex, 6 * i + 2) = T0[2] * dx1[i][2];
        A.insert(constraintIndex, 6 * i + 3) = T0[3] * dx1[i][3];
        A.insert(constraintIndex, 6 * i + 4) = T0[4] * dx1[i][4];
        A.insert(constraintIndex, 6 * i + 5) = T0[5] * dx1[i][5];
        A.insert(constraintIndex, 6 * i + 6) = T0[6];
        lowerBound(constraintIndex) = 0;
        upperBound(constraintIndex) = 0;
        constraintIndex++;
    }

    //T1光滑约束-等式约束
    for (int i = 0; i < N - 1; ++i)
    {
        //向量ai
        A.insert(constraintIndex, 6 * i + 0) = T1[0];
        A.insert(constraintIndex, 6 * i + 1) = T1[1];
        A.insert(constraintIndex, 6 * i + 2) = T1[2] * dx1[i][1];
        A.insert(constraintIndex, 6 * i + 3) = T1[3] * dx1[i][2];
        A.insert(constraintIndex, 6 * i + 4) = T1[4] * dx1[i][3];
        A.insert(constraintIndex, 6 * i + 5) = T1[5] * dx1[i][4];
        A.insert(constraintIndex, 6 * i + 7) = T1[7];
        lowerBound(constraintIndex) = 0;
        upperBound(constraintIndex) = 0;
        constraintIndex++;
    }

    //T2光滑约束-等式约束
    for (int i = 0; i < N - 1; ++i)
    {
        //向量ai
        A.insert(constraintIndex, 6 * i + 0) = T2[0];
        A.insert(constraintIndex, 6 * i + 1) = T2[1];
        A.insert(constraintIndex, 6 * i + 2) = T2[2];
        A.insert(constraintIndex, 6 * i + 3) = T2[3] * dx1[i][1];
        A.insert(constraintIndex, 6 * i + 4) = T2[4] * dx1[i][2];
        A.insert(constraintIndex, 6 * i + 5) = T2[5] * dx1[i][3];
        A.insert(constraintIndex, 6 * i + 8) = T2[8];
     
        lowerBound(constraintIndex) = 0;
        upperBound(constraintIndex) = 0;
        constraintIndex++;
    }
    //T3光滑约束-等式约束
    for (int i = 0; i < N - 1; ++i)
    {
        //向量ai
        A.insert(constraintIndex, 6 * i + 0) = T3[0];
        A.insert(constraintIndex, 6 * i + 1) = T3[1];
        A.insert(constraintIndex, 6 * i + 2) = T3[2];
        A.insert(constraintIndex, 6 * i + 3) = T3[3];
        A.insert(constraintIndex, 6 * i + 4) = T3[4] * dx1[i][1];
        A.insert(constraintIndex, 6 * i + 5) = T3[5] * dx1[i][2];
        A.insert(constraintIndex, 6 * i + 9) = T3[9];
 
        lowerBound(constraintIndex) = 0;
        upperBound(constraintIndex) = 0;
        constraintIndex++;
    }

    //起止点约束-等式约束
    A.insert(constraintIndex, 1) = 1;
    lowerBound(constraintIndex) = ref_tan_start;
    upperBound(constraintIndex) = ref_tan_start;
    constraintIndex++;

    A.insert(constraintIndex, 6 * (N - 1) + 1) = 1;
    lowerBound(constraintIndex) = ref_tan_end;
    upperBound(constraintIndex) = ref_tan_end;
    constraintIndex++;

    //位置bounding box约束-不等式约束
    for (int i = 0; i < sampling_num; i++)
    {
        // x坐标约束
        A.insert(constraintIndex, 6 * i + 0) = T0[0];
        lowerBound(constraintIndex) = anchor_points[i].second - bounding_box_constraint;
        upperBound(constraintIndex) = anchor_points[i].second + bounding_box_constraint;
        constraintIndex++;
    }

    solver.clearSolver();
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    solver.data()->setNumberOfVariables(6 * N);
    solver.data()->setNumberOfConstraints(total_constraint_num);
    solver.data()->clearHessianMatrix();
    solver.data()->clearLinearConstraintsMatrix();
    if (!solver.data()->setHessianMatrix(H))
        return 1;
    if (!solver.data()->setGradient(gradient))
        return 1;
    if (!solver.data()->setLinearConstraintsMatrix(A))
        return 1;
    if (!solver.data()->setLowerBound(lowerBound))
        return 1;
    if (!solver.data()->setUpperBound(upperBound))
        return 1;
    if (!solver.initSolver())
        return 1;

    Eigen::VectorXd QPSolution;
    solver.solveProblem();
    QPSolution = solver.getSolution();
    spline_curve temp_curve;
    vector<spline_curve> curve_seg;
    Eigen::VectorXd px;
    Eigen::VectorXd py;
    for (int i = 0; i < QPSolution.rows() / 6 - 1; i++)
    {
        temp_curve.a0 = QPSolution(6 * i + 0);
        temp_curve.a1 = QPSolution(6 * i + 1);
        temp_curve.a2 = QPSolution(6 * i + 2);
        temp_curve.a3 = QPSolution(6 * i + 3);
        temp_curve.a4 = QPSolution(6 * i + 4);
        temp_curve.a5 = QPSolution(6 * i + 5);

        temp_curve.x_start = temp_curve.a0;
        temp_curve.x_end = temp_curve.a0 + temp_curve.a1 + temp_curve.a2 + temp_curve.a3 + temp_curve.a4 + temp_curve.a5;
        float len = path[i+1].x - path[i].x;
        float f = fabs(1.0 / len);
        for (float t = 0; t <= len; t += f) {
            int y = temp_curve.a0 + temp_curve.a1 * t + temp_curve.a2 * t * t + temp_curve.a3 * t * t * t + temp_curve.a4 * t * t * t * t + temp_curve.a5 * t * t * t * t * t;
            int x = path[i].x + t;
            y += path[0].y;
            if (x < 0 || y < 0 || x >= 1000 || y >= 1000)continue;
            tmp.at<Vec3b>(y, x) = color[i % 6];
            outPath.push_back(Point(x, y));
        }
    }
    imshow("tmp1", tmp);
    return 0;
}
int OSQPSolver(vector<Point>path, vector<Point>&outPath)
{
    Mat tmp = Mat::zeros(1000, 1000, CV_8UC3);
    Eigen::VectorXd c(4);
    c << (thresholds1-50)/10, (thresholds2 - 50) / 10, (thresholds1 - 50) / 10, (thresholds2 - 50) / 10;
    int N = path.size(); //input point number
    int sampling_num = N;
    double interval = 5;
    Point2D start_point(0, 0);
    vector<double> input_points_x;
    vector<double> input_points_y;
    vector<double> anchor_points_x;
    vector<double> anchor_points_y;
    for (int i = 0; i < path.size(); i++) {
         anchor_points_x.push_back(path[i].x - path[0].x);
         anchor_points_y.push_back(path[i].y -  path[0].y);
    }
    vector<Point2D> input_points;
    vector<Point2D> anchor_points;
    double x_ref = 0.0;
    double y_ref = 0.0;
    double ref_tan_start = (thresholds1-50)/10;// 0 * c[0] + 1 * c[1] + 2 * c[2] * start_point.second + 3 * c[3] * start_point.second * start_point.second;
    double ref_tan_end = (thresholds2-50)/10;// 0 * c[0] + 1 * c[1] + 2 * c[2] * (start_point.second + (N - 1) * interval) + 3 * c[3] * pow(start_point.second + (N - 1) * interval, 2);

    double bounding_box_constraint = 1e-7;
    Point2D temp_point;
    for (int i = 0; i < path.size(); i++)
    {
        circle(tmp, path[i], 2, Scalar(255, 255, 255));
    }
    for (int i = 0; i < sampling_num; i++)
    {
        temp_point.first = anchor_points_x[i];
        temp_point.second = anchor_points_y[i];
        anchor_points.push_back(temp_point);
        circle(tmp, Point(temp_point.first + path[0].x, temp_point.second + path[0].y), 2, Scalar(0, 255, 255));
    }
    //至此得到输入离散点列与位置约束的原始线上参考点(anchor points)

    Eigen::SparseMatrix<double> hessian(12 * N, 12 * N);
    for (int i = 0; i < hessian.rows() / 6; i++)
    {
        hessian.insert(6 * i + 3, 6 * i + 3) = 36;
        hessian.insert(6 * i + 3, 6 * i + 4) = 72;
        hessian.insert(6 * i + 3, 6 * i + 5) = 120;
        hessian.insert(6 * i + 4, 6 * i + 3) = 72;
        hessian.insert(6 * i + 4, 6 * i + 4) = 192;
        hessian.insert(6 * i + 4, 6 * i + 5) = 360;
        hessian.insert(6 * i + 5, 6 * i + 3) = 120;
        hessian.insert(6 * i + 5, 6 * i + 4) = 360;
        hessian.insert(6 * i + 5, 6 * i + 5) = 720;
    }
    Eigen::SparseMatrix<double> R(12 * N, 12 * N);
    for (int i = 0; i < R.rows(); i++)
    {
        R.insert(i, i) = 1.0e-5;
    }
    Eigen::SparseMatrix<double> H(12 * N, 12 * N);
    H = hessian + R;
    int smooth_constraint_num = 4 * (N - 1) * 2;
    int position_constraint_num = N * 2;
    int start_end_constraint_num = 2;
    int total_constraint_num = smooth_constraint_num + position_constraint_num + start_end_constraint_num;
    Eigen::VectorXd lowerBound(total_constraint_num);
    Eigen::VectorXd upperBound(total_constraint_num);
    Eigen::SparseMatrix<double> A(total_constraint_num, 12 * N); //所有约束的矩阵
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(2 * 6 * N);
    //光滑性约束共4*(N-1)*2个,由等式化为2个不等式约束
    Eigen::VectorXd T0(12);    //0阶导数向量
    Eigen::VectorXd T0_05(12); //0阶导数向量参数0.5时
    Eigen::VectorXd T1(12);    //1阶导数向量
    Eigen::VectorXd T2(12);    //2阶导数向量
    Eigen::VectorXd T3(12);    //3阶导数向量
    T0 << 1, 1, 1, 1, 1, 1, -1, 0, 0, 0, 0, 0;
    T1 << 0, 1, 2, 3, 4, 5, 0, -1, 0, 0, 0, 0;
    T2 << 0, 0, 2, 6, 12, 20, 0, 0, -2, 0, 0, 0;
    T3 << 0, 0, 0, 6, 24, 60, 0, 0, 0, -6, 0, 0;
    T0_05 << 1, 0.5, pow(0.5, 2), pow(0.5, 3), pow(0.5, 4), pow(0.5, 5), 0, 0, 0, 0, 0, 0;
    int constraintIndex = 0;
    
    //T0光滑约束-等式约束
    for (int i = 0; i < N - 1; ++i)
    {
        //向量ai
        A.insert(constraintIndex, 6 * i + 0) = T0[0];
        A.insert(constraintIndex, 6 * i + 1) = T0[1];
        A.insert(constraintIndex, 6 * i + 2) = T0[2];
        A.insert(constraintIndex, 6 * i + 3) = T0[3];
        A.insert(constraintIndex, 6 * i + 4) = T0[4];
        A.insert(constraintIndex, 6 * i + 5) = T0[5];
        A.insert(constraintIndex, 6 * i + 6) = T0[6];
        //向量bi constraintIndex
        A.insert(constraintIndex + total_constraint_num / 2, 6 * i + 6 * N + 0) = T0[0];
        A.insert(constraintIndex + total_constraint_num / 2, 6 * i + 6 * N + 1) = T0[1];
        A.insert(constraintIndex + total_constraint_num / 2, 6 * i + 6 * N + 2) = T0[2];
        A.insert(constraintIndex + total_constraint_num / 2, 6 * i + 6 * N + 3) = T0[3];
        A.insert(constraintIndex + total_constraint_num / 2, 6 * i + 6 * N + 4) = T0[4];
        A.insert(constraintIndex + total_constraint_num / 2, 6 * i + 6 * N + 5) = T0[5];
        A.insert(constraintIndex + total_constraint_num / 2, 6 * i + 6 * N + 6) = T0[6];
        lowerBound(constraintIndex) = 0;
        upperBound(constraintIndex) = 0;
        lowerBound(constraintIndex + total_constraint_num / 2) = 0;
        upperBound(constraintIndex + total_constraint_num / 2) = 0;
        constraintIndex++;
    }

    //T1光滑约束-等式约束
    for (int i = 0; i < N - 1; ++i)
    {
        //向量ai
        A.insert(constraintIndex, 6 * i + 0) = T1[0];
        A.insert(constraintIndex, 6 * i + 1) = T1[1];
        A.insert(constraintIndex, 6 * i + 2) = T1[2];
        A.insert(constraintIndex, 6 * i + 3) = T1[3];
        A.insert(constraintIndex, 6 * i + 4) = T1[4];
        A.insert(constraintIndex, 6 * i + 5) = T1[5];
        A.insert(constraintIndex, 6 * i + 7) = T1[7];
        //向量bi
        A.insert(constraintIndex + total_constraint_num / 2, 6 * i + 6 * N + 0) = T1[0];
        A.insert(constraintIndex + total_constraint_num / 2, 6 * i + 6 * N + 1) = T1[1];
        A.insert(constraintIndex + total_constraint_num / 2, 6 * i + 6 * N + 2) = T1[2];
        A.insert(constraintIndex + total_constraint_num / 2, 6 * i + 6 * N + 3) = T1[3];
        A.insert(constraintIndex + total_constraint_num / 2, 6 * i + 6 * N + 4) = T1[4];
        A.insert(constraintIndex + total_constraint_num / 2, 6 * i + 6 * N + 5) = T1[5];
        A.insert(constraintIndex + total_constraint_num / 2, 6 * i + 6 * N + 7) = T1[7];
        lowerBound(constraintIndex) = 0;
        upperBound(constraintIndex) = 0;
        lowerBound(constraintIndex + total_constraint_num / 2 ) = 0;
        upperBound(constraintIndex + total_constraint_num / 2) = 0;
        constraintIndex++;
    }

    //T2光滑约束-等式约束
    for (int i = 0; i < N - 1; ++i)
    {
        //向量ai
        A.insert(constraintIndex, 6 * i + 0) = T2[0];
        A.insert(constraintIndex, 6 * i + 1) = T2[1];
        A.insert(constraintIndex, 6 * i + 2) = T2[2];
        A.insert(constraintIndex, 6 * i + 3) = T2[3];
        A.insert(constraintIndex, 6 * i + 4) = T2[4];
        A.insert(constraintIndex, 6 * i + 5) = T2[5];
        A.insert(constraintIndex, 6 * i + 8) = T2[8];
        //向量bi
        A.insert(constraintIndex + total_constraint_num / 2 , 6 * i + 6 * N + 0) = T2[0];
        A.insert(constraintIndex + total_constraint_num / 2 , 6 * i + 6 * N + 1) = T2[1];
        A.insert(constraintIndex + total_constraint_num / 2 , 6 * i + 6 * N + 2) = T2[2];
        A.insert(constraintIndex + total_constraint_num / 2 , 6 * i + 6 * N + 3) = T2[3];
        A.insert(constraintIndex + total_constraint_num / 2 , 6 * i + 6 * N + 4) = T2[4];
        A.insert(constraintIndex + total_constraint_num / 2 , 6 * i + 6 * N + 5) = T2[5];
        A.insert(constraintIndex + total_constraint_num / 2 , 6 * i + 6 * N + 8) = T2[8];

        lowerBound(constraintIndex) = 0;
        upperBound(constraintIndex) = 0;
        lowerBound(constraintIndex + total_constraint_num / 2) = 0;
        upperBound(constraintIndex + total_constraint_num / 2) = 0;
        constraintIndex++;
    }
    //T3光滑约束-等式约束
    for (int i = 0; i < N - 1; ++i)
    {
        //向量ai
        A.insert(constraintIndex, 6 * i + 0) = T3[0];
        A.insert(constraintIndex, 6 * i + 1) = T3[1];
        A.insert(constraintIndex, 6 * i + 2) = T3[2];
        A.insert(constraintIndex, 6 * i + 3) = T3[3];
        A.insert(constraintIndex, 6 * i + 4) = T3[4];
        A.insert(constraintIndex, 6 * i + 5) = T3[5];
        A.insert(constraintIndex, 6 * i + 9) = T3[9];
        //向量bi
        A.insert(constraintIndex + total_constraint_num / 2 , 6 * i + 6 * N + 0) = T3[0];
        A.insert(constraintIndex + total_constraint_num / 2 , 6 * i + 6 * N + 1) = T3[1];
        A.insert(constraintIndex + total_constraint_num / 2 , 6 * i + 6 * N + 2) = T3[2];
        A.insert(constraintIndex + total_constraint_num / 2 , 6 * i + 6 * N + 3) = T3[3];
        A.insert(constraintIndex + total_constraint_num / 2 , 6 * i + 6 * N + 4) = T3[4];
        A.insert(constraintIndex + total_constraint_num / 2 , 6 * i + 6 * N + 5) = T3[5];
        A.insert(constraintIndex + total_constraint_num / 2 , 6 * i + 6 * N + 9) = T3[9];

        lowerBound(constraintIndex) = 0;
        upperBound(constraintIndex) = 0;
        lowerBound(constraintIndex + total_constraint_num / 2) = 0;
        upperBound(constraintIndex + total_constraint_num / 2) = 0;
        constraintIndex++;
    }

    //起止点约束-等式约束
    A.insert(constraintIndex, 1) = -ref_tan_start;
    A.insert(constraintIndex, 6 * N + 1) = 1;

    A.insert(constraintIndex + total_constraint_num / 2, 6 * (N - 1) + 1) = -ref_tan_end * 1;
    A.insert(constraintIndex + total_constraint_num / 2, 6 * (2 * N - 1) + 1) =  1;
    lowerBound(constraintIndex) = 0;
    upperBound(constraintIndex) = 0;
    lowerBound(constraintIndex + total_constraint_num / 2) = 0;
    upperBound(constraintIndex + total_constraint_num / 2) = 0;
    constraintIndex++;

    //位置bounding box约束-不等式约束
    for (int i = 0; i < sampling_num; i++)
    {
        // x坐标约束
        A.insert(constraintIndex, 6 * i + 0) = T0[0];
        //A.insert(smooth_constraint_num / 2 + i, 6 * (i - 1) + 1) = T0[1];
        //A.insert(smooth_constraint_num / 2 + i, 6 * (i - 1) + 2) = T0[2];
        //A.insert(smooth_constraint_num / 2 + i, 6 * (i - 1) + 3) = T0[3];
        //A.insert(smooth_constraint_num / 2 + i, 6 * (i - 1) + 4) = T0[4];
        //A.insert(smooth_constraint_num / 2 + i, 6 * (i - 1) + 5) = T0[5];
        // y坐标约束
        A.insert(constraintIndex + total_constraint_num / 2, 6 * i + 0 + 6 * N) = T0[0];
  /*      A.insert((smooth_constraint_num + total_constraint_num) / 2 + i, 6 * (i - 1) + 1 + 6 * N) = T0[1];
        A.insert((smooth_constraint_num + total_constraint_num) / 2 + i, 6 * (i - 1) + 2 + 6 * N) = T0[2];
        A.insert((smooth_constraint_num + total_constraint_num) / 2 + i, 6 * (i - 1) + 3 + 6 * N) = T0[3];
        A.insert((smooth_constraint_num + total_constraint_num) / 2 + i, 6 * (i - 1) + 4 + 6 * N) = T0[4];
        A.insert((smooth_constraint_num + total_constraint_num) / 2 + i, 6 * (i - 1) + 5 + 6 * N) = T0[5];*/

        lowerBound(constraintIndex) = anchor_points[i].first - bounding_box_constraint;
        upperBound(constraintIndex) = anchor_points[i].first + bounding_box_constraint;
        lowerBound(constraintIndex + total_constraint_num / 2) = anchor_points[i].second - bounding_box_constraint;
        upperBound(constraintIndex + total_constraint_num / 2) = anchor_points[i].second + bounding_box_constraint;
        constraintIndex++;
     }

    OsqpEigen::Solver solver;

    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    solver.data()->setNumberOfVariables(2 * 6 * N);
    solver.data()->setNumberOfConstraints(total_constraint_num);

    if (!solver.data()->setHessianMatrix(H))
        return 1;
    if (!solver.data()->setGradient(gradient))
        return 1;
    if (!solver.data()->setLinearConstraintsMatrix(A))
        return 1;
    if (!solver.data()->setLowerBound(lowerBound))
        return 1;
    if (!solver.data()->setUpperBound(upperBound))
        return 1;
    if (!solver.initSolver())
        return 1;

    Eigen::VectorXd QPSolution;
    solver.solveProblem();
    QPSolution = solver.getSolution();
    spline_curve temp_curve;
    vector<spline_curve> curve_seg;
    Eigen::VectorXd px;
    Eigen::VectorXd py;
    for (int i = 0; i < QPSolution.rows() / 12-1; i++)
    {
        temp_curve.a0 = QPSolution(6 * i + 0);
        temp_curve.a1 = QPSolution(6 * i + 1);
        temp_curve.a2 = QPSolution(6 * i + 2);
        temp_curve.a3 = QPSolution(6 * i + 3);
        temp_curve.a4 = QPSolution(6 * i + 4);
        temp_curve.a5 = QPSolution(6 * i + 5);

        temp_curve.b0 = QPSolution(6 * (i + N) + 0);
        temp_curve.b1 = QPSolution(6 * (i + N) + 1);
        temp_curve.b2 = QPSolution(6 * (i + N) + 2);
        temp_curve.b3 = QPSolution(6 * (i + N) + 3);
        temp_curve.b4 = QPSolution(6 * (i + N) + 4);
        temp_curve.b5 = QPSolution(6 * (i + N) + 5);

        temp_curve.x_start = temp_curve.a0;
        temp_curve.y_start = temp_curve.b0;
        temp_curve.x_end = temp_curve.a0 + temp_curve.a1 + temp_curve.a2 + temp_curve.a3 + temp_curve.a4 + temp_curve.a5;
        temp_curve.y_end = temp_curve.b0 + temp_curve.b1 + temp_curve.b2 + temp_curve.b3 + temp_curve.b4 + temp_curve.b5;
        for (float t = 0; t <= 1; t += 0.1) {
            int x = temp_curve.a0 + temp_curve.a1 * t + temp_curve.a2 * t * t + temp_curve.a3* t*t*t + temp_curve.a4 * t * t * t * t + temp_curve.a5 * t * t * t * t * t;
            int y = temp_curve.b0 + temp_curve.b1 * t + temp_curve.b2 * t * t + temp_curve.b3 * t * t * t + temp_curve.b4 * t * t * t * t + temp_curve.b5 * t * t * t * t * t;
            x += path[0].x;
            y += path[0].y;
            if (x < 0 || y < 0 || x >= 1000 || y >= 1000)continue;
            tmp.at<Vec3b>(y, x) = color[i%6];
            outPath.push_back(Point(x, y));
        }
        curve_seg.push_back(temp_curve);
    }
    imshow("tmp", tmp);
    return 0;
}
int main() {
	show = Mat::zeros(1000, 1000, CV_8UC3);
	imshow("show", show);
	cv::setMouseCallback("show", onMouse);
	cv::createTrackbar("speed：", "show", &thresholds, 100, speed);
    cv::createTrackbar("t1：", "show", &thresholds1, 100, th1);
    cv::createTrackbar("t2：", "show", &thresholds2, 100, th2);
	bool calcFlag = false;
	while (1) {
		show = Mat::zeros(1000, 1000, CV_8UC3);
		for (int i = 0; i < path.size(); i++) {
			circle(show, path[i], 2, Scalar(255, 255, 255));
		}
		int key = waitKey(10);
		if (key == ' ') {
			show = Mat::zeros(1000, 1000, CV_8UC3);
			path.clear();
			calcFlag = false;
		}
		else if (key == 's'|| calcFlag) {
			calcFlag = true;
			vector<Point>bezierPoint;
			double t1 = getTickCount();
			DealWithPathBySquareBezier(path, bezierPoint);
			double t2 = getTickCount();
			printf("SquareBezier %lf\n", (t2 - t1) / getTickFrequency());
			for (int i = 0; i < bezierPoint.size(); i++) {
				show.at<Vec3b>(bezierPoint[i].y, bezierPoint[i].x)[0] = 255;
			}
			vector<Point>threePoint;
			t1 = getTickCount();
			DealWithPathByThreeBezier(path, threePoint);
			t2 = getTickCount();
			printf("ThreeBezier %lf\n", (t2 - t1) / getTickFrequency());
			for (int i = 0; i < threePoint.size(); i++) {
				show.at<Vec3b>(threePoint[i].y, threePoint[i].x)[1] = 255;
			}
			t1 = getTickCount();
			int n = 5;
			vector<Point> in_point(begin(path), end(path));
			Mat mat_k = polyfit(in_point, n);
			for (int i = in_point[0].x; i < in_point[size(in_point) - 1].x; ++i)
			{
				Point2d ipt;
				ipt.x = i;
				ipt.y = 0;
				for (int j = 0; j < n + 1; ++j)
				{
					ipt.y += mat_k.at<double>(j, 0) * pow(i, j);
				}
                if (ipt.x < 0 || ipt.y < 0 || ipt.x>=1000 || ipt.y >= 1000)continue;
				show.at<Vec3b>(ipt.y, ipt.x)[2] = 255;
			}
			t2 = getTickCount();
			printf("polyfit %lf\n", (t2 - t1) / getTickFrequency());


            vector<Point>QPPoint;
            t1 = getTickCount();
            OSQPSolverTest(path, QPPoint);
            t2 = getTickCount();
            printf("QP %lf\n", (t2 - t1) / getTickFrequency());
            for (int i = 0; i < QPPoint.size(); i++) {
                show.at<Vec3b>(QPPoint[i].y, QPPoint[i].x)[1] = 255;
                show.at<Vec3b>(QPPoint[i].y, QPPoint[i].x)[2] = 255;
            }

          /*  QPPoint.clear();
            t1 = getTickCount();
            OSQPSolver(path, QPPoint);
            t2 = getTickCount();
            printf("QP %lf\n", (t2 - t1) / getTickFrequency());
            for (int i = 0; i < QPPoint.size(); i++) {
                show.at<Vec3b>(QPPoint[i].y, QPPoint[i].x)[0] = 255;
                show.at<Vec3b>(QPPoint[i].y, QPPoint[i].x)[2] = 255;
            }*/
		}
		imshow("show", show);
	}
	return 0;
}
