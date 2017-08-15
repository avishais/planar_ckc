#include "../validity_checkers/StateValidityCheckerPCS.h"
//#include "../validity_checkers/verification_class.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

int main() {

	int Seed = time(NULL);//1501629079;//
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	// KDL
	StateValidityChecker svc(5, 0.3);

	//Verify
	//verification_class vfc;

	int n = 20;
	State q1(n), q2(n);

	std::ofstream f;
	f.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc2d/tests/results/pcs_rbs_verification.txt", ios::app);

	int N = 1e5, i = 0;
	while (i < N) {

		q1 = svc.sample_q();

		double s = svc.fRand(0.01, 1);
		for (int j = 0; j < n-1; j++)
			q2[j] = q1[j] + s * (svc.fRand(-PI, PI)-q1[j]);
		q2[n-1] = q1[n-1] + s * (svc.fRand(0, 2*PI)-q1[j]);

		int active_chain = rand() / RAND_MAX * n;
		int ik_sol = rand() / RAND_MAX * 2;

		if (!svc.IKproject(q2, active_chain, ik_sol))
			continue;
		State ik2 = svc.identify_state_ik(q2);

		State ik1 = svc.identify_state_ik(q1);

		bool vsuc;
		double rbs_time;
		for (int j = 0; j < n; j++) {
			if (ik1[j]==ik2[j]) {
				clock_t begin = clock();
				vsuc = svc.checkMotionRBS(q1a, q1b, q2a, q2b, 0, ik1[0], 0, 0);
				rbs_time = double(clock() - begin) / CLOCKS_PER_SEC;
			}
		}


		if (ik1[0]==ik2[0]) {
			clock_t begin = clock();
			vsuc = svc.checkMotionRBS(q1a, q1b, q2a, q2b, 0, ik1[0], 0, 0);
			rbs_time = double(clock() - begin) / CLOCKS_PER_SEC;
			active_chain = 0;
		}
		if (!vsuc && ik1[1]==ik2[1]) {
			clock_t begin = clock();
			vsuc = svc.checkMotionRBS(q1a, q1b, q2a, q2b, 1, ik1[1], 0, 0);
			rbs_time = double(clock() - begin) / CLOCKS_PER_SEC;
			active_chain = 1;
		}

		if (vsuc) {
			//cout << "Found LC." << endl;
			Matrix path;
			path.push_back(svc.join_Vectors(q1a, q1b));
			path.push_back(svc.join_Vectors(q2a, q2b));
			svc.reconstructRBS(q1a, q1b, q2a, q2b, active_chain, ik1[active_chain], path, 0, 1, 1);

			bool path_valid = vfc.verify_path(path);

			if (!path_valid) {
				vfc.log_path_file(path);
				cout << "Verified to be: " << vfc.verify_path(path) << endl;
				cout << "Press...\n";
				cin.ignore();
			}

			f << vsuc << " " << path_valid << " " << svc.normDistance(q1, q2) << " " << rbs_time << endl;
			i++;
		}
		else
			f << 0 << " " << 0 << " " << svc.normDistance(q1, q2) << " " << rbs_time << endl;
	}

	f.close();

	return 0;
}

