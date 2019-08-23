#pragma once
#include <vector>
#define  PI 3.1415926
double toRadian(double degree);
double toDegree(double radian);
double max(double a, double b);
double min(double a, double b);
double sum(double a[], int n);
double pointdist(double a_x, double a_y, double b_x, double b_y);
double **doubleArray(int m, int n);
void freeArray(double **p, int m);

double **DynamicWindowApproach(double **x, double model[6], double goal[2], double evalParam[4], double ob[][2], double R);
double *CalcDynamicWindow(double **x, double model[6]);
std::vector<double*> Evaluation(double **x, double *Vr, double goal[2], double ob[][2], double R, double model[6], double evalParam[4]);
void GenerateTrajectory(double **x, double vt, double ot, double evaldt, double model[6]);
void f(double **x, double **u);
double CalcHeadingEval(double **x, double goal[2]);
double CalcDistEval(double **x, double ob[][2], double R);
double CalcBreakingDist(double vel, double model[6]);
void NormalizeEval(std::vector<double*> evalDB);
void print(double *a, int m);
void print(double **a, int m, int n);

