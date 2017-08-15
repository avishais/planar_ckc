#include "apc_class.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

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

	// APC
	ckc A(n, 0.3);

	std::ofstream f;
	f.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc2d/tests/results/apc_verification.txt", ios::app);

	int N = 1;//0.5e6;
	State q(n), qp_apc(n);
	double aph_time = 0;

	int i = 0;
	while (i < N) {

		for (int j = 0; j < n-1; j++)
			q[j] = fRand(-PI, PI);
		q[n-1] = fRand(0, 2*PI);

		State q_rand = q;

		// APC
		int tries;
		bool Asuc;
		for (tries = 0; tries < 20; tries++) {
			int active_chain = 0;//rand()/RAND_MAX * (n-2);
			int ik_sol = 0;//rand()/RAND_MAX * 2;
			clock_t begin = clock();
			Asuc = A.project(q, active_chain, ik_sol); // q is updated
			aph_time = double(clock() - begin) / CLOCKS_PER_SEC;
			if (Asuc)
				break;
		}
		if (tries==20)
			continue;

		A.printVector(q);
		A.printVector(A.constraint(q));

		f << Asuc << " " << aph_time << endl;

		A.log_q(q);

		i++;
	}

	f.close();


}

