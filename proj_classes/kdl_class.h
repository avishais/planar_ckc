/*
 * kdl_class.h
 *
 *  Created on: Feb 27, 2017
 *      Author: avishai
 */

#ifndef KDL_CLASS_H_
#define KDL_CLASS_H_

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
//#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

#define PI_ 3.1416

using namespace std;
using namespace KDL;

typedef vector<double> State;
typedef vector<vector< double >> Matrix;

class kdl
{
private:
	State L;
	double bx, by; // base length
	double qminmax; // Joint limits

	int n; // Number of joints

	// IK parameters
	State q_solution; // IK solution

	// Temp variable for time efficiency
	Matrix T_pose;

public:
	// Constructor
	kdl(int, double);

	/** KDL declarations */
	KDL::Chain chain;
	//ChainFkSolverPos_recursive fksolver;
	KDL::JntArray jointpositions;
	KDL::Frame cartposFK; // Create the frame that will contain the FK results
	KDL::Frame cartposIK; // Create the frame that will contain the FK results

	/** Newton-Raphson projection onto the constraint surface */
	bool GD(State q_init);
	State get_GD_result();

	/** Check the angles limits of the ABB - IK based on a constant trans. matrix */
	bool check_angle_limits(State);

	/**  Forward kinematics of the arm - only for validation */
	void FK(State q);
	Matrix get_FK_solution();
	Matrix T_fk, T_fk_temp;
	State constraint(State q);

	/** Misc */
	void initVector(State &, int);
	void initMatrix(Matrix &, int, int);
	double deg2rad(double);
	void printMatrix(Matrix);
	void printVector(State);
	void clearMatrix(Matrix &);
	State rand_q();

	// Generate random number
	double fRand(double fMin, double fMax)
	{
		double f = (double)rand() / RAND_MAX;
		return fMin + f * (fMax - fMin);
	}

	double get_bx() {
		return bx;
	}
	double get_by() {
		return by;
	}
	State getL() {
		return L;
	}
	double get_qminmax() {
		return qminmax;
	}
	/** Log conf. to path.txt file */
	void log_q(State, bool = true);

	/** Performance parameters */
	int IK_counter;
	double IK_time;
	int get_IK_counter() {
		return IK_counter;
	}
	double get_IK_time() {
		return IK_time;
	}

};



#endif /* KDL_CLASS_H_ */
