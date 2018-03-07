#include "../validity_checkers/StateValidityCheckerPCS.h"
//#include "../validity_checkers/verification_class.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

using namespace pcs;

int main() {

	int Seed = 1502899207;//time(NULL);//1501629079;//
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	int n = 20;

	// KDL
	StateValidityChecker svc(n, 0.3);

	//Verify
	//verification_class vfc(n, 0.3);

	State q1(n), q2(n);

	std::ofstream f;
	f.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc2d/tests/results/pcs_m_analysis2.txt", ios::app);

	int N = 0.2e6, i = 0;
	while (i < N) {

		q1 = svc.sample_q();

		double s = svc.fRand(0.01, 1);
		for (int j = 0; j < n-1; j++)
			q2[j] = q1[j] + s * (svc.fRand(-PI_, PI_)-q1[j]);
		q2[n-1] = q1[n-1] + s * (svc.fRand(0, 2*PI_)-q1[n-1]);

		int active_chain = rand() / RAND_MAX * n;
		int ik_sol = rand() / RAND_MAX * 2;

		if (!svc.IKproject(q2, active_chain, ik_sol))
			continue;
		State ik2 = svc.identify_state_ik(q2);
		State ik1 = svc.identify_state_ik(q1);

		State m_suc(n, 0);

		bool vsuc;
		for (int m = 0; m < n; m++) {
			if (ik1[m] == ik2[m] && ik1[m] != -1) {
				vsuc = svc.checkMotionRBS(q1, q2, m, ik1[m], 0, 0);
				active_chain = m;

				if (vsuc) {
					for (int j = m; j < n; j++)
						m_suc[j] = 1;

					break;
				}
			}
		}

		if (vsuc) {
			//cout << "Found LC." << endl;
			Matrix path;
			path.push_back(q1);
			path.push_back(q2);
			svc.reconstructRBS(q1, q2, active_chain, ik1[active_chain], path, 0, 1, 1, 0);

			bool path_valid = true;//vfc.verify_path(path);

			//vfc.log_path_file(path);

			/*if (!path_valid) {
				vfc.log_path_file(path);
				cout << "Verified to be: " << vfc.verify_path(path) << endl;
				cout << "Press...\n";
				cin.ignore();
			}*/

			f << vsuc << " " << path_valid << " " << svc.normDistance(q1, q2) << " ";
			i++;
		}
		else
			f << vsuc << " " << 0 << " " << svc.normDistance(q1, q2) << " ";

		// Log passive chains results
		for (int j = 0; j < n; j++)
			f << m_suc[j] << " ";
		f << endl;
	}

	f.close();

	return 0;
}

