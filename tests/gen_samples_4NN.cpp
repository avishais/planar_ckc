#include "kdl_class.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

bool withObs = false;

int main() {

	int Seed = time(NULL);
	srand(Seed);
	cout << "Seed in testing: " << Seed << endl;

	std::ofstream File;
	File.open("ckc2d_20_samples_noJL.txt", ios::app);

	// KDL
	int n = 20;
	kdl pj(n, -1);

	int N = 2e6-1;
	State qb4(n), qaf(n);
	for (int i = 0; i < N; i++) {
		
		while (1) {
			qb4 = pj.rand_q();

			if (!pj.GD(qb4))
				continue;

			qaf = pj.get_GD_result();

			break;
		}

		cout << "Completed " << double(i+1)/N*100 << "%%." << endl;

		pj.log_q(qaf);

		for (int j = 0; j < qb4.size(); j++)
			File << qb4[j] << " ";
		for (int j = 0; j < qaf.size(); j++)
			File << qaf[j] << " ";
		File << endl;
	}

	File.close();

	return 0;
}
