/*
 * plan_C_space.h
 *
 *  Created on: Nov 10, 2016
 *      Author: avishai
 */

#ifndef PLAN_C_SPACE_H_
#define PLAN_C_SPACE_H_

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

// Modified and custom planners
#include "../planners/CBiRRT_pcs.h"

//#include "../validity_checkers/StateValidityCheckerPCS.h"
#include "../validity_checkers/verification_class.h"

// Standard libraries
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std;

bool isStateValid(const ob::State *state);

// Prototypes
class plan_C //: public StateValidityChecker
{
public:

	void plan(Vector c_start, Vector c_goal, int n, int m, double runtime, double range = 0.5);

	bool solved_bool;
	double total_runtime;

};

#endif /* PLAN_C_SPACE_H_ */
