#include "kdl_class.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

#define ROBOTS_DISTANCE 900
#define ROD_LENGTH 300

double fRand(double fMin, double fMax)
{
	double f = (double)rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}

double dist(State p1, State p2) {
	double sum = 0;
	for (int i = 0; i < p1.size(); i++)
		sum += (p1[i]-p2[i])*(p1[i]-p2[i]);

	return sqrt(sum);
}

int main() {

	int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	int n = 20;

	// KDL
	kdl K(20, 0.3);

	std::ofstream f;
	f.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc2d/tests/results/kdl_verification.txt", ios::app);

	int N = 0.5e6;
	State q(n), qp_kdl(n);
	double kdl_time = 0, gd_time = 0;

	for (int i = 0; i < N; i++) {

		for (int j = 0; j < n-1; j++)
			q[j] = fRand(-PI, PI);
		q[n-1] = fRand(0, 2*PI);

		// KDL
		clock_t begin = clock();
		bool Ksuc = K.GD(q);
		qp_kdl = K.get_GD_result();
		kdl_time = double(clock() - begin) / CLOCKS_PER_SEC;

		State C = K.constraint(qp_kdl);
		bool verify = true;
		for (int j = 0; j < 3 && verify; j++)
			if (fabs(C[j]) > 0.05)
				verify = false;

		f << Ksuc << " " << verify << " " << kdl_time << endl;


	}

	f.close();
}

