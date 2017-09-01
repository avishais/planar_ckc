#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"

#include <iostream>
#include <fstream>

#include "../validity_checkers/StateValidityCheckerPCS.h"
#include "../validity_checkers/StateValidityCheckerGD.h"

class testI : public pcs::StateValidityChecker, public gd::StateValidityChecker
{
public:
	testI(int n) : pcs::StateValidityChecker::StateValidityChecker(n), gd::StateValidityChecker::StateValidityChecker(n) {
		cout << "In constructor.\n";
	}
};


int main() {
	testI t(20);
	
	return 0;
}