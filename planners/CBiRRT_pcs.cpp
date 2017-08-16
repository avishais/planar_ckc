/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

//#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

#include "myRRTConnect.h" // Avishai

// Debugging tool
template <class T>
void o(T a) {
	cout << a << endl;
}

ompl::geometric::RRTConnect::RRTConnect(const base::SpaceInformationPtr &si, int joints_num, int passive_chains_num, double custom_num) : base::Planner(si, "RRTConnect"), StateValidityChecker(si, joints_num, passive_chains_num, custom_num)
{
	specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
	specs_.directed = true;

	maxDistance_ = 0.0;

	Planner::declareParam<double>("range", this, &RRTConnect::setRange, &RRTConnect::getRange, "0.:1.:10000.");
	connectionPoint_ = std::make_pair<base::State*, base::State*>(nullptr, nullptr);
	defaultSettings(); // Avishai

	n = joints_num;
	m = passive_chains_num;
	Range = 2;

}

ompl::geometric::RRTConnect::~RRTConnect()
{
	freeMemory();
}

void ompl::geometric::RRTConnect::setup()
{
	Planner::setup();
	tools::SelfConfig sc(si_, getName());
	sc.configurePlannerRange(maxDistance_);

	if (!tStart_)
		tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
	if (!tGoal_)
		tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
	tStart_->setDistanceFunction(std::bind(&RRTConnect::distanceFunction, this, std::placeholders::_1, std::placeholders::_2));
	tGoal_->setDistanceFunction(std::bind(&RRTConnect::distanceFunction, this, std::placeholders::_1, std::placeholders::_2));
}

void ompl::geometric::RRTConnect::freeMemory()
{
	std::vector<Motion*> motions;

	if (tStart_)
	{
		tStart_->list(motions);
		for (unsigned int i = 0 ; i < motions.size() ; ++i)
		{
			if (motions[i]->state)
				si_->freeState(motions[i]->state);
			delete motions[i];
		}
	}

	if (tGoal_)
	{
		tGoal_->list(motions);
		for (unsigned int i = 0 ; i < motions.size() ; ++i)
		{
			if (motions[i]->state)
				si_->freeState(motions[i]->state);
			delete motions[i];
		}
	}
}

void ompl::geometric::RRTConnect::clear()
{
	Planner::clear();
	sampler_.reset();
	freeMemory();
	if (tStart_)
		tStart_->clear();
	if (tGoal_)
		tGoal_->clear();
	connectionPoint_ = std::make_pair<base::State*, base::State*>(nullptr, nullptr);
}

double ompl::geometric::RRTConnect::activeDistance(const Motion *a, const Motion *b) {

	Vector qa(n);
	Vector qb(n);

	retrieveStateVector(a->state, qa);
	retrieveStateVector(b->state, qb);

	double sum = 0;
	for (int i=0; i < qa.size(); i++) {
		if (i < active_chain || i > active_chain+2) 
			sum += pow(qa[i]-qb[i], 2);
	}
	return sqrt(sum);
}

double ompl::geometric::RRTConnect::distanceBetweenTrees(TreeData &tree1, TreeData &tree2) {

	std::vector<Motion*> motions;
	tree1->list(motions);

	Motion *nmotion;
	double minD = 1e10, curD;
	for (unsigned int i = 0 ; i < motions.size() ; ++i)
	{
		nmotion = tree2->nearest(motions[i]);
		curD = distanceFunction(nmotion, motions[i]);
		if (curD < minD) {
			minD = curD;
		}
	}
	return minD;
}


ompl::geometric::RRTConnect::Motion* ompl::geometric::RRTConnect::growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *nmotion, Motion *tmotion, int mode, int count_iterations)
// tmotion - target
// nmotion - nearest
// mode = 1 -> extend, mode = 2 -> connect.
{
	Vector q(n), ik(m);

	// Choose active chain
	active_chain = rand() % m; 

	bool reach = false;
	growTree_reached = false;

	//cout << "nmotion: "; printStateVector(nmotion->state);
	
	//int count_iterations = 500;
	while (count_iterations) {
		count_iterations--;

		// find state to add
		base::State *dstate = tmotion->state;
		double d = activeDistance(nmotion, tmotion);

		if (d > maxDistance_)
		{
			si_->getStateSpace()->interpolate(nmotion->state, tmotion->state, maxDistance_ / d, tgi.xstate);
			dstate = tgi.xstate;
			reach = false;
		}
		else
			reach = true;

		if (mode==1 || !reach) { // equivalent to (!(mode==2 && reach))
			if (!IKproject(dstate, active_chain, nmotion->ik_vect[active_chain])) {
				project_fail++;
				return nmotion;
			}
			else
				project_success++;

			retrieveStateVector(dstate, q);
			updateStateVector(tgi.xstate, q);
			dstate = tgi.xstate;

			ik = identify_state_ik(dstate);
		}
		else { // Added but not tested
			retrieveStateVector(dstate, q, ik);
			active_chain = -1;
			for (int i = 0; i < m; i++) {
				if (nmotion->ik_vect[i] == ik[i])
					active_chain = i;
			}
			if (active_chain == -1) {
				zero_rejection_count++;
				return nmotion;
			}
		}

		// Check motion
		//bool validMotion = checkMotion(nmotion->state, dstate, active_chain, nmotion->ik_vect[active_chain]);
		//bool validMotion = checkMotionRBS(nmotion->state, dstate, active_chain, nmotion->ik_vect[active_chain]);

		bool validMotion = false;
		for (int i = 0; i < ik.size(); i++) {
			if (nmotion->ik_vect[i] == ik[i])
				validMotion = checkMotionRBS(nmotion->state, dstate, i, nmotion->ik_vect[i]);
			if (validMotion)
				break;
		}

		if (validMotion)
		{
			RBS_success++;
			// Update advanced motion
			Motion *motion = new Motion(si_);
			motion->ik_vect.resize(m);
			motion->ik_vect = ik;
			updateStateVectorIK(dstate, ik);
			si_->copyState(motion->state, dstate);
			motion->parent = nmotion;
			motion->root = nmotion->root;
			motion->a_chain = active_chain;
			tgi.xmotion = motion;
			tree->add(motion);

			nmotion = motion;

			if (reach) {
				growTree_reached = true;
				return nmotion;
			}
		}
		else {
			RBS_fail++;
			return nmotion;
		}
	}
	return nmotion;
}


ompl::base::PlannerStatus ompl::geometric::RRTConnect::solve(const base::PlannerTerminationCondition &ptc)
{
	initiate_log_parameters();
	setRange(Range);
	base::State *start_node = si_->allocState();

	Vector q(n), ik(m);

	checkValidity();
	startTime = clock();
	base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

	if (!goal)
	{
		OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
		return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
	}

	while (const base::State *st = pis_.nextStart())
	{
		if (0) {//!close_chain(st, 1)) {
			OMPL_ERROR("%s: Start state not feasible!", getName().c_str());
			return base::PlannerStatus::INVALID_START;
		}
		ik = identify_state_ik(st);
		updateStateVectorIK(st, ik);
		retrieveStateVector(st, q, ik);
		cout << "Start: "; printStateVector(st);

		Motion *motion = new Motion(si_);
		si_->copyState(motion->state, st);
		motion->root = motion->state;
		motion->ik_vect.resize(m);
		motion->ik_vect = ik;
		motion->a_chain = 0;
		tStart_->add(motion);

		si_->copyState(start_node,st);
	}

	// ------

	/*Vector qq = sample_q();
	updateStateVector(start_node, qq);
	printStateVector(start_node);
	ik = identify_state_ik(start_node);
	printVector(ik);


	return base::PlannerStatus::INVALID_START;*/
	// ------

	if (tStart_->size() == 0)
	{
		OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
		return base::PlannerStatus::INVALID_START;
	}

	if (!goal->couldSample())
	{
		OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
		return base::PlannerStatus::INVALID_GOAL;
	}

	if (!sampler_)
		sampler_ = si_->allocStateSampler();

	OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(), (int)(tStart_->size() + tGoal_->size()));

	TreeGrowingInfo tgi;
	tgi.xstate = si_->allocState();

	Motion   *rmotion   = new Motion(si_);
	base::State *rstate = rmotion->state;
	bool startTree      = true;
	bool solved         = false;

	while (ptc == false)
	{
		TreeData &tree      = startTree ? tStart_ : tGoal_;
		tgi.start = startTree;
		startTree = !startTree;
		TreeData &otherTree = startTree ? tStart_ : tGoal_;

		if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
		{
			const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
			if (st)
			{
				if (0){//!close_chain(st, 0)) {
					OMPL_ERROR("%s: Goal state not feasible!", getName().c_str());
					printStateVector(st);
					return base::PlannerStatus::INVALID_GOAL;
				}
				ik = identify_state_ik(st);
				updateStateVectorIK(st, ik);
				retrieveStateVector(st, q, ik);
				cout << "Goal: "; printStateVector(st);

				Motion *motion = new Motion(si_);
				si_->copyState(motion->state, st);
				motion->root = motion->state;
				motion->ik_vect.resize(m);
				motion->ik_vect = ik;
				motion->a_chain = 0;
				tGoal_->add(motion);
				PlanDistance = si_->distance(start_node, st);
			}

			if (tGoal_->size() == 0)
			{
				OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
				break;
			}
		}

		//cout << "Trees size: " << tStart_->size() << ", " << tGoal_->size() << endl;
		//cout << "Current trees distance: " << distanceBetweenTrees(tree, otherTree) << endl << endl;

		//===================== New ==========================
		Motion* reached_motion;
		// sample random state
		sampler_->sampleUniform(rstate);

		// Grow tree
		Motion *nmotion = tree->nearest(rmotion); // NN over the active distance
		reached_motion = growTree(tree, tgi, nmotion, rmotion, 1, 100);

		// remember which motion was just added
		Motion *addedMotion = reached_motion;

		// Grow otherTree toward reached_motion in tree <-connect
		tgi.xmotion = nullptr;

		nmotion = otherTree->nearest(reached_motion); // NN over the active distance
		reached_motion = growTree(otherTree, tgi, nmotion, reached_motion, 2, 500);

		Motion *startMotion = startTree ? tgi.xmotion : addedMotion;
		Motion *goalMotion  = startTree ? addedMotion : tgi.xmotion;

		// if we connected the trees in a valid way (start and goal pair is valid)
		if (growTree_reached) {
			cout << "Connection point active chain is " << ((!a_chain_connection) ? "q1." : "q2.") << endl;

			// Report computation time
			endTime = clock();
			total_runtime = double(endTime - startTime) / CLOCKS_PER_SEC;
			cout << "Solved in " << total_runtime << "s." << endl;

			// it must be the case that either the start tree or the goal tree has made some progress
			// so one of the parents is not nullptr. We go one step 'back' to avoid having a duplicate state
			// on the solution path

			cout << addedMotion << " " << tgi.xmotion << endl;

			if (startMotion->parent)
				startMotion = startMotion->parent;
			else
				goalMotion = goalMotion->parent;

			connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

			// construct the solution path
			Motion *solution = startMotion;
			std::vector<Motion*> mpath1;
			while (solution != nullptr)
			{
				mpath1.push_back(solution);
				solution = solution->parent;
			}

			solution = goalMotion;
			std::vector<Motion*> mpath2;
			while (solution != nullptr)
			{
				mpath2.push_back(solution);
				solution = solution->parent;
			}

			cout << "Path from tree 1 size: " << mpath1.size() << ", path from tree 2 size: " << mpath2.size() << endl;
			nodes_in_path = mpath1.size() + mpath2.size();
			nodes_in_trees = tStart_->size() + tGoal_->size();

			PathGeometric *path = new PathGeometric(si_);
			path->getStates().reserve(mpath1.size() + mpath2.size());
			for (int i = mpath1.size() - 1 ; i >= 0 ; --i)
				path->append(mpath1[i]->state);
			for (unsigned int i = 0 ; i < mpath2.size() ; ++i)
				path->append(mpath2[i]->state);
			save2file(mpath1, mpath2);
			pdef_->addSolutionPath(base::PathPtr(path), false, 0.0, getName());
			solved = true;
			break;
		}

		//====================================================
	}
	if (!solved)
	{
		// Report computation time
		endTime = clock();
		total_runtime = double(endTime - startTime) / CLOCKS_PER_SEC;

		nodes_in_trees = tStart_->size() + tGoal_->size();
	}

	si_->freeState(tgi.xstate);
	si_->freeState(rstate);
	delete rmotion;

	OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(), tStart_->size(), tGoal_->size());

	final_solved = solved;
	LogPerf2file();

	return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::RRTConnect::getPlannerData(base::PlannerData &data) const
{
	Planner::getPlannerData(data);

	std::vector<Motion*> motions;
	if (tStart_)
		tStart_->list(motions);

	for (unsigned int i = 0 ; i < motions.size() ; ++i)
	{
		if (motions[i]->parent == nullptr)
			data.addStartVertex(base::PlannerDataVertex(motions[i]->state, 1));
		else
		{
			data.addEdge(base::PlannerDataVertex(motions[i]->parent->state, 1),
					base::PlannerDataVertex(motions[i]->state, 1));
		}
	}

	motions.clear();
	if (tGoal_)
		tGoal_->list(motions);

	for (unsigned int i = 0 ; i < motions.size() ; ++i)
	{
		if (motions[i]->parent == nullptr)
			data.addGoalVertex(base::PlannerDataVertex(motions[i]->state, 2));
		else
		{
			// The edges in the goal tree are reversed to be consistent with start tree
			data.addEdge(base::PlannerDataVertex(motions[i]->state, 2),
					base::PlannerDataVertex(motions[i]->parent->state, 2));
		}
	}

	// Add the edge connecting the two trees
	data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));
}

void ompl::geometric::RRTConnect::log_q(Vector q) {
	// Open a_path file
	std::ofstream myfile;
	myfile.open("path.txt");

	for (int j = 0; j < q.size(); j++) 
		myfile << q[j] << " ";
	myfile << endl;

	myfile.close();
}

void ompl::geometric::RRTConnect::LogPerf2file() {

	std::ofstream myfile;
	myfile.open("perf_log.txt");

	myfile << final_solved << " ";
	myfile << PlanDistance << " "; // Distance between nodes 1
	myfile << total_runtime << " "; // Overall planning runtime 2
	myfile << get_IK_counter() << " "; // How many IK checks? 5
	myfile << get_IK_time() << " "; // IK computation time 6
	//myfile << get_collisionCheck_counter() << endl; // How many collision checks? 7
	//myfile << get_collisionCheck_time() << endl; // Collision check computation time 8
	myfile << get_isValid_counter() << " "; // How many nodes checked 9
	myfile << nodes_in_path << " "; // Nodes in path 10
	myfile << nodes_in_trees << " "; // 11
	myfile << RBS_success << " " << RBS_fail << " ";
	myfile << project_success << " " << project_fail;

	myfile.close();
}

void ompl::geometric::RRTConnect::save2file(vector<Motion*> mpath1, vector<Motion*> mpath2) {

	cout << "Logging path to files..." << endl;

	Vector q(n);
	Matrix path;
	int active_chain;


	// Log env. info
	std::ofstream mf;
	mf.open("path_info.txt");
	mf << n << endl << 0 << endl << getL() << endl << get_bx() << endl << get_by() << endl << get_qminmax() << endl;
	if (include_constraints)
	for (int i = 0; i < obs.size(); i++)
		for (int j = 0; j < 3; j++)
			mf << obs[i][j] << endl;
	mf.close();

	// Only milestones
	{
		// Open a_path file
		std::ofstream myfile, ikfile;
		myfile.open("path_ms.txt");

		Vector temp;
		for (int i = mpath1.size() - 1 ; i >= 0 ; --i) {
			retrieveStateVector(mpath1[i]->state, q);
			for (int j = 0; j<n; j++) {
				myfile << q[j] << " ";
			}
			myfile << endl;

			path.push_back(q);
		}
		for (unsigned int i = 0 ; i < mpath2.size() ; ++i) {
			retrieveStateVector(mpath2[i]->state, q);
			for (int j = 0; j<n; j++) {
				myfile << q[j] << " ";
			}
			myfile << endl;

			path.push_back(q);
		}
		myfile.close();
	}
	
	//printMatrix(path);

	// Milestones with interpolation
	{
		ob::State *s1 = si_->allocState();
		ob::State *s2 = si_->allocState();
		Matrix interPath;
		Vector q, ik1(m), ik2(m);

		// Open a_path file
		std::ofstream qfile;
		qfile.open("path.txt");

		// First point
		for (int j = 0; j < n; j++) {
			qfile << path[0][j] << " ";
		}
		qfile << endl;

		// Rest of points
		for (int i = 1; i < path.size() ; i++) {
			updateStateVector(s1, path[i-1]);
			ik1 = identify_state_ik(s1);
			updateStateVector(s2, path[i]);
			ik2 = identify_state_ik(s2);
			//printVector(ik1);
			//printVector(ik2);
			bool flag = false;
			int j = 0;
			while (j < m) {
				if (ik1[j]!=ik2[j]) {
					j++;
					continue;
				}						
				interPath.clear();

				if (InterMotion(s1, s2, interPath, j, ik1[j])) {
					//cout << "Found connection: " << i << " and " << i-1 << endl;
					flag  = true;
					break;
				}
				else {
					j++;
					continue;
				}	
			}

			if (!flag)
				cout << "Error in path interpolation between points " << i << " and " << i-1 << "." << endl;
			else {
				for (j = 1; j < interPath.size(); j++) {
					for (int k = 0; k < n; k++) {
						qfile << interPath[j][k] << " ";
					}
					qfile << endl;
				}
			}
		}
		qfile.close();
	}
}