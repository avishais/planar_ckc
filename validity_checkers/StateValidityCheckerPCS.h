/*
 * Checker.h
 *
 *  Created on: Oct 28, 2016
 *      Author: avishai
 */

#ifndef CHECKER_H_
#define CHECKER_H_

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include "ompl/base/MotionValidator.h"
#include "ompl/base/State.h"
#include <ompl/config.h>
#include <ompl/util/RandomNumbers.h>

#include "apc_class.h"

#include <iostream>

namespace ob = ompl::base;
using namespace std;

class StateValidityChecker : public ckc
{
public:
	/** Constructors */
	StateValidityChecker(const ob::SpaceInformationPtr &si, int joints_num, int passive_chains_num, double custom_num) : mysi_(si.get()), ckc(joints_num, custom_num) { //Constructor
		n = joints_num;
		m = passive_chains_num;
		q_temp.resize(6);
		L = getL();
	};
	StateValidityChecker(int joints_num, double custom_num=-1) : ckc(joints_num, custom_num) { //Constructor
		n = joints_num;
		m = n;
		q_temp.resize(6);
		L = getL();
	};

	/** Project a configuration in the ambient space to the constraint surface */
	bool IKproject(ob::State *, int, int, bool = true);
	bool IKproject(Vector &, int, int);

	/** Identify the IK solutions of a configuration using all passive chains defined */
	Vector identify_state_ik(const ob::State *);
	Vector identify_state_ik(Vector);

	/** Validity check using standard OMPL */
	bool isValid(const ob::State *, int, int, bool);

	/** Validity check for a vector<double> type  */
	bool isValidRBS(Vector&, int, int);

	/** Serial local connection check  */
	bool checkMotion(const ob::State *, const ob::State *, int, int);

	/** Recursive Bi-Section local connection check  */
	bool checkMotionRBS(const ob::State *, const ob::State *, int, int);
	bool checkMotionRBS(Vector, Vector, int active_chain, int ik_sol, int recusion_depth, int);

	/** Reconstruct a local connection using RBS for post-processing  */
	bool reconstructRBS(const ob::State *, const ob::State *, Matrix &, int, int);
	bool reconstructRBS(Vector, Vector, int, int, Matrix &, int, int, int, int);

	Vector midpoint(Vector q1, Vector q2);
	Vector angle_distance(Vector q1, Vector q2);

	/** Retrieve state from ob::State to vector<double> */
	void retrieveStateVector(const ob::State *state, Vector &q);

	/** Update state to ob::State from vector<double> */
	void updateStateVector(const ob::State *state, Vector q);

	/** Print ob::State to console */
	void printStateVector(const ob::State *state);

	/** Set default OMPL setting */
	void defaultSettings();

	/** Calculate norm distance between two vectors */
	double normDistance(Vector, Vector);
	double normVector(Vector q);
	
	/** Sample a feasible configuration */
	Vector sample_q();

	int get_valid_solution_index() {
		return valid_solution_index;
	}

	double get_RBS_tol() {
		return RBS_tol;
	}

	// Performance parameters
	int isValid_counter;
	int get_isValid_counter() {
		return isValid_counter;
	}

	double iden = 0;
	double get_iden() {
		return iden;
	}
	
	// Include constraints?
	const bool include_constraints = true; // Enable/Disable constraints
	bool check_angles(Vector, double = 1);
	bool self_collision(Vector, double = 1);
	bool obstacle_collision(Vector, double = 0.3);
	bool LinesIntersect(Vector, Vector, Vector, Vector, double);

	//Matrix obs = {{-4, 4, 1},{3.4, 7.5, 1}, {4.2, -2.9, 1.5}, {8, 2, 0.8}};
	Matrix obs = {{-3.2, 4, 1},{2.6, 6.8, 1}, {4.2, -2.9, 1.5}, {8, 2, 0.8}};

	double t3, t4, t5, t6, t7, t8;

private:
	ob::StateSpace *stateSpace_;
	ob::SpaceInformation *mysi_;
	Vector q_temp;
	int valid_solution_index;
	int n;
	int m;

	double L;
	const double dq = 0.1;
	double RBS_tol = 0.05; // RBS local connection resolution
	int RBS_max_depth = 100; // Maximum RBS recursion depth
	
	ompl::RNG rng_;
};





#endif /* CHECKER_H_ */
