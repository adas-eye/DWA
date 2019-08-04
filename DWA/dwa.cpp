#include <iostream>
#include <math.h>
#include "dwa.h"
#include <complex>
//时间
double dt = 0.1;

int main() {
	//机器人的初期状态[x(m), y(m), yaw(Rad), v(m / s), w(rad / s)]
	double** x = doubleArray(5, 1);
	for (int i = 0; i < 5; i++) {
		x[i][0] = 0;
	} 
	//目标点位置
	double goal[2] = {10, 10}; 
	//障碍物位置列表
	double obstacle[18][2] = {
		{0, 2}, {2, 2}, {3, 4}, {5, 4}, {5, 5}, {5, 6}, {5, 9}, {8, 8}, {8, 9}, {7, 9}, {6, 5}, {6, 3}, {6, 8}, {6, 7},
		{7, 4}, {9, 8}, {9, 11}, {9, 6}
	}; 
	//冲突判定用的障碍物半径
	double obstacleR = 0.5; 
	//机器人运动学模型参数
	double Kinematic[6] = {1.0, toRadian(20.0), 0.2, toRadian(50.0), 0.01, toRadian(1)}; 
	//评价函数参数
	double evalParam[4] = {0.05, 0.2, 0.1, 3.0}; 
	//模拟区域范围
	double area[4][1] = {{-1}, {11}, {-1}, {11}};
	for (int i = 0; i < 5000; i++) {
		double** u = DynamicWindowApproach(x, Kinematic, goal, evalParam, obstacle, obstacleR);
		f(x, u);
		std::cout << "=================" << i << "=============" << std::endl;
		print(x, 5, 1);
	}
}

double toRadian(double degree) {
	return degree / 180 * PI;
}

double toDegree(double radian) {
	return radian / PI * 180;
}

double max(double a, double b) {
	return a > b ? a : b;
}

double min(double a, double b) {
	return a < b ? a : b;
}

double sum(double a[], int n) {
	double sum = 0;
	for (int i = 0; i < n; i++) {
		sum += a[i];
	}
	return sum;
}

void print(double *a, int m) {
	for (int i = 0; i < m; i++) {
		std::cout << a[i] << " ";
	}
	std::cout << std::endl;
}
void print(double **a, int m, int n) {
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			std::cout << a[i][j] << " ";
		}
		std::cout << std::endl;
	}
}

//为二维数组分配内存
double** doubleArray(int m, int n) {
	double** res = (double**)malloc(sizeof(double*) * m);
	for (int i = 0; i < m; i++) {
		double* tmp = (double*)malloc(sizeof(double) * n);
		res[i] = tmp;
	}
	return res;
} 

//根据当前状态推算下一个控制周期（dt）的状态
void f(double** x, double** u) {
	double F[5][5] = {{1, 0, 0, 0, 0}, {0, 1, 0, 0, 0}, {0, 0, 1, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}};
	double B[5][2] = {{cos(x[2][0]) * dt, 0}, {dt * sin(x[2][0]), 0}, {0, dt}, {1, 0}, {0, 1}};
	double F_mul_x[5][1], B_mul_u[5][1];
	for (int i = 0; i < 5; i++) {
		double tmp = 0;
		for (int j = 0; j < 5; j++) {
			tmp += F[i][j] * x[j][0];
		}
		F_mul_x[i][0] = tmp;
	}
	for (int i = 0; i < 5; i++) {
		double tmp = 0;
		for (int j = 0; j < 2; j++) {
			tmp += B[i][j] * u[j][0];
		}
		B_mul_u[i][0] = tmp;
	}
	for (int i = 0; i < 5; i++) {
		x[i][0] = F_mul_x[i][0] + B_mul_u[i][0];
	}
} 

//DWA算法实现
double** DynamicWindowApproach(double** x, double model[6], double goal[2], double evalParam[4], double ob[18][2],
								double R) {
	double** u = doubleArray(2, 1);
	double* Vr = CalcDynamicWindow(x, model);
	std::vector<double*> evalDB = Evaluation(x, Vr, goal, ob, R, model, evalParam);
	int eval_len = evalDB.size();
	if (eval_len == 0) {
		u[0][0] = 0;
		u[1][0] = 0;
		return u;
	}
	NormalizeEval(evalDB);
	int max_index;
	double max = 0;
	for (int i = 0; i < eval_len; i++) {
		double tmp = 0;
		for (int j = 0; j < 3; j++) {
			tmp += evalParam[j] * evalDB[i][j + 2];
		}
		if (max < tmp) {
			max = tmp;
			max_index = i;
		}
	}
	u[0][0] = evalDB[max_index][0];
	u[1][0] = evalDB[max_index][1];
	return u;
} 

//计算动态窗口
double* CalcDynamicWindow(double** x, double model[6]) {
	double Vs[4] = {0, model[0], -model[1], model[1]};
	double Vd[4] = {x[3][0] - model[2] * dt, x[3][0] + model[2] * dt, x[4][0] - model[3] * dt, x[4][0] + model[3] * dt};
	double* Vr = (double*)malloc(sizeof(double) * 4);
	Vr[0] = max(Vs[0], Vd[0]);
	Vr[1] = min(Vs[1], Vd[1]);
	Vr[2] = max(Vs[2], Vd[2]);
	Vr[3] = min(Vs[3], Vd[3]);
	return Vr;
} 

//评价函数
std::vector<double*> Evaluation(double** x, double* Vr, double goal[2], double ob[18][2], double R, double model[6],
								double evalParam[4]) {
	std::vector<double*> evalDB;
	double** xt = doubleArray(5, 1);
	for (double vt = Vr[0]; vt < Vr[1]; vt += model[4]) {
		for (double ot = Vr[2]; ot < Vr[3]; ot += model[5]) {
			for (int i = 0; i < 5; i++) {
				xt[i][0] = x[i][0];
			}
			double** traj = GenerateTrajectory(xt, vt, ot, evalParam[3], model);
			double heading = CalcHeadingEval(xt, goal);
			double dist = CalcDistEval(xt, ob, R);
			double vel = abs(vt);
			double stopDist = CalcBreakingDist(vel, model);
			if (dist > stopDist) {
				double* tmp = (double*)malloc(sizeof(double) * 5);
				tmp[0] = vt;
				tmp[1] = ot;
				tmp[2] = heading;
				tmp[3] = dist;
				tmp[4] = vel;
				evalDB.push_back(tmp);
			}
		}
	}
	return evalDB;
} 

//单条轨迹生成、轨迹推演函数
double** GenerateTrajectory(double** x, double vt, double ot, double evaldt, double model[6]) {
	int index = 0;
	double** u = doubleArray(2, 1);
	u[0][0] = vt;
	u[1][0] = ot;
	int n = int(evaldt / dt) + 1;
	double** traj = doubleArray(5, n);
	for (int i = 0; i < 5; i++) {
		traj[i][index] = x[i][0];
	}
	index++;
	while (index < n) {
		f(x, u);
		for (int i = 0; i < 5; i++) {
			traj[i][index] = x[i][0];
		}
		index++;
	}
	return traj;
} 

//heading的评价函数计算
double CalcHeadingEval(double** x, double goal[2]) {
	double theta = toDegree(x[2][0]);
	double goalTheta = toDegree(atan2(goal[1] - x[1][0], goal[0] - x[0][0]));
	double targetTheta;
	if (goalTheta > theta) {
		targetTheta = goalTheta - theta;
	} else {
		targetTheta = theta - goalTheta;
	}
	return 180 - targetTheta;
} 

//障碍物距离评价函数
double CalcDistEval(double** x, double ob[18][2], double R) {
	double dist = 100;
	for (int i = 0; i < 18; i++) {
		std::complex<double> mycomplex(ob[i][0] - x[0][0], ob[i][1] - x[1][0]);
		double disttmp = std::norm(mycomplex);
		if (dist > disttmp) {
			dist = disttmp;
		}
	}
	if (dist > 2 * R) {
		dist = 2 * R;
	}
	return dist;
} 

//计算制动距离 
double CalcBreakingDist(double vel, double model[6]) {
	double stopDist = 0;
	while (vel > 0) {
		stopDist += vel * dt;
		vel -= model[2] * dt;
	}
	return stopDist;
} 

//归一化处理
void NormalizeEval(std::vector<double*> evalDB) {
	int n = evalDB.size();
	double* eval = (double*)malloc(sizeof(double) * n);
	for (int k = 2; k < 5; k++) {
		for (int i = 0; i < n; i++) {
			eval[i] = evalDB[i][k];
		}
		double tmp = sum(eval, n);
		if (tmp != 0) {
			for (int i = 0; i < n; i++) {
				evalDB[i][k] = eval[i] / tmp;
			}
		}
	}
}
