/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Avishai Sintov, Ioan Sucan */

#include "plan_GD.h"

bool isStateValid(const ob::State *state)
{
	return true;
}

void plan_C::plan(State c_start, State c_goal, double runtime, double custom) {

	int n = c_start.size();

	// construct the state space we are planning inz
	ob::StateSpacePtr Q(new ob::RealVectorStateSpace(n)); // Angles of CKC - R^n

	// set the bounds for the Q=R^n part of 'Cspace'
	ob::RealVectorBounds Qbounds(n);
	for (int i = 0; i < n-1; i++) {
		Qbounds.setLow(i, -PI);
		Qbounds.setHigh(i, PI);
	}
	Qbounds.setLow(n-1, 0);
	Qbounds.setHigh(n-1, 2*PI);

	// set the bound for the compound space
	Q->as<ob::RealVectorStateSpace>()->setBounds(Qbounds);

	// construct a compound state space using the overloaded operator+
	ob::StateSpacePtr Cspace(Q);

	// construct an instance of  space information from this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(Cspace));

	// set state validity checking for this space
	//si->setStateValidityChecker(ob::StateValidityCheckerPtr(new myStateValidityCheckerClass(si)));
	si->setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));
	si->setStateValidityCheckingResolution(0.02); // 3% ???

	// create start state
	ob::ScopedState<ob::RealVectorStateSpace> start(Cspace);
	for (int i = 0; i < n; i++) {
		start->as<ob::RealVectorStateSpace::StateType>()->values[i] = c_start[i]; // Access the first component of the start a-state
	}

	// create goal state
	ob::ScopedState<ob::RealVectorStateSpace> goal(Cspace);
	for (int i = 0; i < n; i++) {
		goal->as<ob::RealVectorStateSpace::StateType>()->values[i] = c_goal[i]; // Access the first component of the goal a-state
	}

	// create a problem instance
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

	// set the start and goal states
	pdef->setStartAndGoalStates(start, goal);
	pdef->print();

	// create a planner for the defined space
	// To add a planner, the #include library must be added above
	ob::PlannerPtr planner(new og::RRTConnect(si, n, custom));
	//ob::PlannerPtr planner(new og::RRT(si));

	// set the problem we are trying to solve for the planner
	planner->setProblemDefinition(pdef);

	// perform setup steps for the planner
	planner->setup();

	//planner->printSettings(std::cout); // Prints some parameters such as range
	//planner->printProperties(std::cout); // Prints some decisions such as multithreading, display approx solutions, and optimize?

	// print the settings for this space
	//si->printSettings(std::cout); // Prints state space settings such as check resolution, segmant count factor and bounds
	//si->printProperties(std::cout); // Prints state space properties, average length, dimension ...

	// print the problem settings
	//pdef->print(std::cout); // Prints problem definition such as start and goal states and optimization objective

	// attempt to solve the problem within one second of planning time
	clock_t begin = clock();
	ob::PlannerStatus solved = planner->solve(runtime);
	clock_t end = clock();
	cout << "Runtime: " << double(end - begin) / CLOCKS_PER_SEC << endl;

	if (solved) {
		// get the goal representation from the problem definition (not the same as the goal state)
		// and inquire about the found path
		//ob::PathPtr path = pdef->getSolutionPath();
		std::cout << "Found solution:" << std::endl;

		// print the path to screen
		//path->print(std::cout);  // Print as vectors

		// Save path to file
		//std::ofstream myfile;
		//myfile.open("pathGD.txt");
		//og::PathGeometric& pog = static_cast<og::PathGeometric&>(*path); // Transform into geometric path class
		//pog.printAsMatrix(myfile); // Print as matrix to file
		//myfile.close();
		solved_bool = true;
	}
	else {
		std::cout << "No solutions found" << std::endl;
		solved_bool = false;
	}
}

int main(int argn, char ** args) {
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	double runtime;

	if (argn == 1)
		runtime = 1; // sec
	else {
		runtime = atof(args[1]);
	}

	plan_C Plan;

	int mode = 4;
	switch (mode) {
	case 3: { // Obstacle experiment
		State c_start = {1.6581, 0.17453, 0.17453, 0.17453, -0.034907, -0.17453, -0.17453, -0.5236, -0.69813, -0.5236, -0.87266, -0.17453, 0.087266, 0.34907, 0.17453, 0.17453, 0.17453, 0.18147, -0.80904, 2.4791};
		State c_goal = {-2.1293, 0.34907, 0.5236, 0.5236, 0.69813, 0.61087, 0.61087, -0.17453, -0.7854, -0.5236, -0.34907, 0.5236, 0.7854, 0.7854, 0.2618, 0.43633, -0.17453, -1.2474, 1.2172, 5.0836}; // 4 obs

		Plan.plan(c_start, c_goal, runtime);

		verification_class vfc(c_start.size());
		vfc.verify_path();
		break;
	}
	case 4: {// Benchmark the same scenario
		int N = 1000; // Number of points to take for each k<=m
		string line;

		State c_start = {1.6581, 0.17453, 0.17453, 0.17453, -0.034907, -0.17453, -0.17453, -0.5236, -0.69813, -0.5236, -0.87266, -0.17453, 0.087266, 0.34907, 0.17453, 0.17453, 0.17453, 0.18147, -0.80904, 2.4791};
		State c_goal = {-2.1293, 0.34907, 0.5236, 0.5236, 0.69813, 0.61087, 0.61087, -0.17453, -0.7854, -0.5236, -0.34907, 0.5236, 0.7854, 0.7854, 0.2618, 0.43633, -0.17453, -1.2474, 1.2172, 5.0836}; // 4 obs

		int n = c_start.size();
		verification_class vfc(c_start.size());

		std::ofstream mf;
		std::ifstream pf;
		mf.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc2d/matlab/benchmark_GD_obs_range2.txt", ios::app);

		for (int i = 0; i < N; i++) { // N points for this number of passive chains

			Plan.plan(c_start, c_goal, runtime);

			bool verf = vfc.verify_path();
			if (!verf) {
				cout << "Verification error. press to continue...\n";
				cin.ignore();
			}

			mf << verf << " ";

			pf.open("./paths/perf_log.txt");
			getline(pf, line);
			mf << line << endl;
			pf.close();
		}
		mf.close();
		break;
	}
	case 5: { // Dimensionality analysis
		int N = 500; // Number of points to take for each d
		string line;

		std::ofstream mf;
		std::ifstream pf;
		mf.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc2d/matlab/JulyAnalysis/benchmarkGD_dimensionality_" + std::to_string((int)runtime) + ".txt", ios::app);

		State nn = {5  ,   9  ,  13  ,  17  ,  21  ,  25  ,  29  ,  33  ,  37  ,  41  ,  45  ,  49  ,  52}; // Dimensionality of CKC
		//State nn = {25};//, 20, 25, 27}; // Dimensionality of CKC
		for (int j = 11; j < nn.size(); j++)
		{
			int n = nn[j];
			StateValidityChecker svc(n); // The checker class
			State c_start(n), c_goal(n);

			for (int i = 0; i < N; i++) { // N points for this number of passive chains

				do {
					c_start = svc.sample_q();
				} while (c_start[0] < -900);
				do {
					c_goal = svc.sample_q();
				} while (c_goal[0] < -900);

				Plan.plan(c_start, c_goal, runtime);

				mf << n << " ";
				pf.open("perf_log_GD.txt");
				getline(pf, line);
				mf << line << endl;
				pf.close();
			}
			mf << endl;
		}

		mf.close();

		break;
	}
	case 6 : { // Links/base ratio analysis
		string line;

		std::ofstream mf;
		std::ifstream pf;
		mf.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc2d/matlab/JulyAnalysis/benchmarkGD_baseRatio_" + std::to_string((int)runtime) + "_V2.txt", ios::app);

		int n = 9;

		int N = 1500; // Number of trials

		//for (double r = 0.05; r < 1; r=+0.05) {
		for (int ir = 18; ir < 19; ir++) {
			double r = (ir + 1) * 0.05;

			StateValidityChecker svc(n, r); // The checker class
			State c_start(n), c_goal(n);

			for (int i = 0; i < N; i++) { // N points for this number of passive chains

				do {
					c_start = svc.sample_q();
				} while (c_start[0] < -900);
				do {
					c_goal = svc.sample_q();
				} while (c_goal[0] < -900);

				Plan.plan(c_start, c_goal, runtime, r);

				mf << r << " ";
				pf.open("perf_log_GD.txt");
				getline(pf, line);
				mf << line << endl;
				pf.close();
			}
			mf << endl;
		}

		mf.close();
		break;
	}
	case 7 : { // Annulus analysis
		string line;

		std::ofstream mf;
		std::ifstream pf;
		mf.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc2d/matlab/JulyAnalysis/benchmarkGD_annulus_5D_" + std::to_string((int)runtime) + ".txt", ios::app);

		int n = 5;

		int N = 500; // Number of trials

		//for (double r = 0.05; r < 1; r=+0.05) {
		for (int ir = 0; ir < 8; ir++) {
			double r = ir * 0.15 + 1.25;

			StateValidityChecker svc(n, r); // The checker class
			State c_start(n), c_goal(n);

			for (int i = 0; i < N; i++) { // N points for this number of passive chains

				do {
					c_start = svc.sample_q();
				} while (c_start[0] < -900);
				do {
					c_goal = svc.sample_q();
				} while (c_goal[0] < -900);

				int m = n-2;
				Plan.plan(c_start, c_goal, runtime, r);

				mf << r << " ";
				pf.open("perf_log_GD.txt");
				getline(pf, line);
				mf << line << endl;
				pf.close();
			}
			mf << endl;
		}

		mf.close();
		break;
	}
	}

	std::cout << std::endl << std::endl;

	return 0;
}

