#include "robots_class.h"

// Constructor for the robots
ckc::ckc(int joints_num, double custom_num) {
	bx = 7; // Base for scenarion with obs (n = 20)
	by = 4;
	L = 1;
/*
	// Automate dimensionality experiment
	if (joints_num==30)
		bx = 8; // Base for random exp. (n=30)
	if (joints_num==25 || joints_num==27)
		bx = 7; // Base for random exp. (n=25,27)
	if (joints_num==20 || joints_num==22)
		bx = 6; // Base for random exp. (n=20,22)
	if (joints_num==14 || joints_num==15 || joints_num==17)
		bx = 4; // Base for random exp. (n=14,15,17)
	if (joints_num==10 || joints_num==12)
		bx = 2.5; // For testing with n=10,12
	if (joints_num==5 || joints_num==6 || joints_num==7)
		bx = 1.5; // For testing with n=5,6,7

	//bx = 0.7; // For testing with n=4

	by = 0;*/

/*	if (custom_num==-1)
		custom_num = 0.3;

	double total_length = 6.5;
	double base_links_ratio = custom_num; // 0.3

	// For general dimensionality analysis
	//L = total_length/(joints_num*(1+base_links_ratio)); // Link length.
	//bx = base_links_ratio*(L*joints_num);

	L = total_length/((joints_num-1)*(1+base_links_ratio)); // Link length.
	bx = base_links_ratio*(L*(joints_num-1));
	by = 0;*/

	n = joints_num;
	//m = n-2;

	cout << "Base-links ratio: " << custom_num << ", L: " << L << ", b: " << bx << endl;

	// Joint limits
	qminmax = 179.9/180*PI;

	q_IK.resize(n);
	p_FK_l.resize(3);
	p_FK_r.resize(3);

	cout << "CKC initialized." << endl;
}

// -----FK-------

void ckc::FK_left(Vector q, int nd) {
	// IK solution for the arm based at the left
	// takes nd joints

	double x, y, theta, l;
	x = y = theta = 0;

	double cum_q = 0;
	for (int i = 0; i < nd; i++) {
		cum_q += q[i];

		x = x + L*cos(cum_q);
		y = y + L*sin(cum_q);
		theta = theta + q[i];
	}

	theta = fmod (theta,  2*PI);
	if (theta > PI)
		theta -= 2*PI;
	if (theta < -PI)
		theta += 2*PI;

	p_FK_l = {x, y, theta};
}

void ckc::FK_left_half(Vector q, int nd) {
	// IK solution for the arm based at the left
	// takes nd joints and half of the last link

	double x, y, theta, l;
	x = y = theta = 0;

	double cum_q = 0;
	for (int i = 0; i < nd; i++) {
		l = i==nd-1 ? L/2 : L;

		cum_q += q[i];

		x = x + l*cos(cum_q);
		y = y + l*sin(cum_q);
		theta = theta + q[i];
	}

	theta = fmod (theta,  2*PI);
	if (theta > PI)
		theta -= 2*PI;
	if (theta < -PI)
		theta += 2*PI;

	p_FK_l = {x, y, theta};
}

void ckc::FK_right(Vector q, int nd) {
	// IK solution for the arm based at the right
	// takes nd joints

	double x, y, theta, l, ang;
	theta = 0;
	x = bx;
	y = by;

	double cum_q = 0;
	for (int i = n-1; i >= n-nd; i--) {
		l = i==n-nd ? L/2 : L;
		ang = i==n-1 ? q[i] : -q[i];

		cum_q += ang;

		x = x + l*cos(cum_q);
		y = y + l*sin(cum_q);
		theta = theta + ang;
	}

	theta = fmod (theta,  2*PI);
	if (theta > PI)
		theta -= 2*PI;
	if (theta < -PI)
		theta += 2*PI;

	p_FK_r = {x, y, theta};
}

Vector ckc::get_FK_sol_left() {
	return p_FK_l;
}

Vector ckc::get_FK_sol_right() {
	return p_FK_r;
}


// -------IK----------

bool ckc::IKp(Vector pose, int ik_sol) {
	IK_counter++;
	clock_t begin = clock();

	// Pose = {x3 ; y3; theta}
	Vector q(3);
	int sign;

	double theta = pose[2];
	double x2 = pose[0] - L/2 * cos(theta);
	double y2 = pose[1] - L/2 * sin(theta);

	double r = sqrt(x2*x2+y2*y2);
	double S = (r*r-L*L-L*L)/(-2*L*L);
	double Ss = (1-S*S);

	if (ik_sol == 1)
	    sign = 1;
	else
	    sign = -1;

	if (Ss<=1e-6) {
		clock_t end = clock();
		IK_time += double(end - begin) / CLOCKS_PER_SEC;
	    return false;
	}

	double Phi = atan2(sign*sqrt(Ss), S);
	q[1] = PI+Phi;
	if (q[1] < -PI)
		q[1] += 2*PI;
	if (q[1] > PI)
		q[1] -= 2*PI;

	q[0] = atan2(L*sin(Phi), L-L*cos(Phi)) + atan2(y2, x2);
	if (q[0] < -PI)
		q[0] += 2*PI;
	if (q[0] > PI)
		q[0] -= 2*PI;

	q[2] = theta - (q[0]+q[1]);
	if (q[2] < -PI)
		q[2] += 2*PI;
	if (q[2] > PI)
		q[2] -= 2*PI;

	q_IK = q;

	clock_t end = clock();
	IK_time += double(end - begin) / CLOCKS_PER_SEC;

	return true;
}

Vector ckc::get_IK_sol_q() {
	return q_IK;
}


// ------- MISC ---------

void ckc::printMatrix(Matrix M) {
	for (unsigned i = 0; i < M.size(); i++) {
		for (unsigned j = 0; j < M[i].size(); j++)
			cout << M[i][j] << " ";
		cout << endl;
	}
}

void ckc::printVector(Vector p) {
	cout << "[";
	for (unsigned i = 0; i < p.size(); i++) {
		if (fabs(p[i])<1e-5)
			cout << 0 << " ";
		else
			cout << p[i] << " ";
	}
	cout << "]" << endl;
}

