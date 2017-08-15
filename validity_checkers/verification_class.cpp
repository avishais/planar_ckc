/*
 * veification_class.cpp
 *
 *  Created on: Aug 1, 2017
 *      Author: avishai
 */

#include "verification_class.h"

bool verification_class::verify_path() {

	cout << "Verifying path in file: " << path_file << "..." << endl;

	State q(12);
	Matrix M;
	int tmp;

	ifstream fq;
	fq.open(path_file);

	fq >> tmp;

	int i = 0;
	while(!fq.eof()) {
		for (int j=0; j < 12; j++) {
			fq >> q[j];
		}
		M.push_back(q);
		i++;
	}
	fq.close();

	bool vrf = verify_path(M);

	if (vrf) {
		cout << "***************************************" << endl;
		cout << "************ PATH FEASIBLE ************" << endl;
		cout << "***************************************" << endl;
	}
	else {
		cout << "***************************************" << endl;
		cout << "*********** ! PATH FAILED ! ***********" << endl;
		cout << "***************************************" << endl;
	}

	return vrf;
}

bool verification_class::verify_path(Matrix M) {

	int m = M.size();

	// Validate joint limits
	for (int i = 0; i < m; i++) {
		if (!check_angle_limits(M[i])) {
			cout << "***************************************" << endl;
			cout << "@ Joint limits failure!" << endl;
			cout << "@ Node " << i << endl;
			return false; // Report joint limit breach
		}
	}

	// Validate continuity
	for (int i = 1; i < m; i++) {
		//cout << normDistance(M[i],M[i-1]) << endl;
		if (normDistance(M[i],M[i-1]) > continuity_tol) {
			cout << "***************************************" << endl;
			cout << "@ Discontinuity failure!" << endl;
			cout << "@ Node " << i << endl;
			return false; // Report discontinuity
		}
	}

	// Validate closure constraint
	for (int i = 0; i < m; i++) {
		if (!test_constraint(M[i])) {
			cout << "***************************************" << endl;
			cout << "@ Closure constraint failure!" << endl;
			cout << "@ Node " << i << endl;
			return false;
		}
	}

	// Validate collisions
	State q1(M[0].size()/2), q2(M[0].size()/2);
	for (int i = 0; i < m; i++) {
		seperate_Vector(M[i], q1, q2);
		if (collision_state(getPMatrix(), q1, q2)) {
			cout << "***************************************" << endl;
			cout << "@ Collision failure!" << endl;
			cout << "@ Node " << i << endl;
			return false;
		}
	}

	return true; // No errors
}

void verification_class::log_path_file(Matrix M) {

	std::ofstream mf;

	mf.open(path_file);

	mf << M.size() << endl;

	for (int i = 0; i < M.size(); i++) {
		for (int j = 0; j < M[0].size(); j++)
			mf << M[i][j] << " ";
		mf << endl;
	}

	mf.close();
}

bool verification_class::test_constraint(State q) {
	Matrix Tsb = Tsb_matrix(q);

	bool valid = true;
	for (int j = 0; j < 3 && valid; j++)
		for (int k = 0; k < 4; k++) {
			if ((k < 3 && fabs(Tsb[j][k]-Tsd[j][k]) > 0.1) || (k==3 && fabs(Tsb[j][k]-Tsd[j][k]) > 0.5)) {
				printVector(q);
				printMatrix(Tsb);
				printMatrix(Tsd);
				valid = false;
				break;
			}

		}
	return valid;

}




