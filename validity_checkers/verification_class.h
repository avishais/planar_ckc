/*
 * veification_class.cpp
 *
 *  Created on: Aug 1, 2017
 *      Author: avishai
 */

#ifdef PCS
#include "StateValidityCheckerPCS.h"
#endif
#ifdef PGD
#include "StateValidityCheckerGD.h"
#endif

//#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

typedef vector<double> State;
typedef vector<vector< double >> Matrix;

class verification_class :
	#ifdef PCS
		public StateValidityCheckerPCS
	#else
		public StateValidityCheckerGD
	#endif
{
public:
	verification_class(int joints_num, double custom_num = 0.3) :
	#ifdef PCS
		StateValidityCheckerPCS(joints_num, custom_num)
	#else
	StateValidityCheckerGD(joints_num, custom_num)
	#endif
	{
		cout << "Initiated verification module." << endl;
		continuity_tol = get_RBS_tol() * 1.1;

		n = joints_num;
	}

	bool verify_path();
	bool verify_path(Matrix M);
	void log_path_file(Matrix M);
	bool test_constraint(State q);

private:
	double L;

	int n;

	double continuity_tol;
	
	string path_file = "./paths/path.txt";

	bool cAngles = true; // Check angle limits?
	bool cObs = true; // Check collisions with obstacles?
	bool cColl = true; // Check self collisions?
};


