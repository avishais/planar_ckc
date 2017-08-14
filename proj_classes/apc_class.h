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
typedef vector< double > Vector;
typedef vector<vector< double >> Matrix;

class ckc
{
private:
	double L; // Links lengths
	double bx, by; // base length
	double qminmax; // Joint limits


	int n; // Number of joints
	//int m; // Number of passive chains

	Vector q_IK; // Result of the IK solution
	Vector p_FK_l; // result of the FK_left solution
	Vector p_FK_r; // result of the FK_right solution

public:
	// Constructor
	ckc(int, double);

	// Forward kinematics
	void FK_left(Vector, int);
	void FK_left_half(Vector, int); // Takes half of the last link
	void FK_right(Vector, int);
	Vector get_FK_sol_left();
	Vector get_FK_sol_right();

	// Inverse kinematics
	bool IKp(Vector, int);
	bool IKp(Vector, int, Vector); // For robots_class5.cpp
	Vector get_IK_sol_q();

	// Misc
	void printMatrix(Matrix M);
	void printVector(Vector p);

	double get_bx() {
		return bx;
	}
	double get_by() {
		return by;
	}
	double getL() {
		return L;
	}
	double get_qminmax() {
		return qminmax;
	}

	Vector LL;

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
