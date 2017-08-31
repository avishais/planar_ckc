#pragma once
/* This code defines the kinematics (IK and FK) of two ABB IRB120 robotic arms */

#include <iostream>
#include <vector>
#include <fstream>
#include <ctime>
#include <math.h>

#define PI 3.1416

using namespace std;

typedef vector<double> State;
typedef vector< double > State;
typedef vector< int > VectorInt;
typedef vector<vector< double >> Matrix;

class ckc
{
private:
	double L; // Links lengths
	double bx, by, b; // base coordinates
	double qminmax; // Joint limits


	int n; // Number of joints
	//int m; // Number of passive chains

	State q_IK; // Result of the IK solution
	State p_FK_l; // result of the FK_left solution
	State p_FK_r; // result of the FK_right solution

public:
	// Constructor
	ckc(int, double);

	// Forward kinematics
	void FK_left(State, int);
	void FK_left_half(State, int); // Takes half of the last link
	void FK_right(State, int);
	State get_FK_sol_left();
	State get_FK_sol_right();

	// Inverse kinematics
	bool IKp(State, int, double);
	State get_IK_sol_q();

	// Project
	bool project(State &q, int nc, int IK_sol);

	// Misc
	void printMatrix(Matrix M);
	void printVector(State p);
	void log_q(State q);
	State constraint(State q);

	double get_bx() {
		return bx;
	}
	double get_by() {
		return by;
	}
	double get_b() {
		return b;
	}
	double getL() {
		return L;
	}
	double get_qminmax() {
		return qminmax;
	}
	State get_q_IK() {
		return q_IK;
	}

	// Generate random number
	double fRand(double fMin, double fMax)
	{
		double f = (double)rand() / RAND_MAX;
		return fMin + f * (fMax - fMin);
	}

	State LL;

	// Performance parameters
	int IK_counter;
	double IK_time;
	int get_IK_counter() {
		return IK_counter;
	}
	double get_IK_time() {
		return IK_time;
	}
};
