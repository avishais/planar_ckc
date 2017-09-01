#include "apc_class.h"

// Constructor for the robots
ckc::ckc(int joints_num, double custom_num) {

	if (joints_num == 20) {
		bx = 7; // Base for scenarion with obs (n = 20)
		by = 4;
		L = 1;
	}
	else {
		bx = 1.5;
		by = 0;
		L = 1;
	}

	/*if (custom_num==-1)
		custom_num = 0.3;

	double total_length = 6.5;
	double base_links_ratio = 0.3;

	L = total_length/((joints_num-1)*(1+base_links_ratio)); // Link length.
	bx = base_links_ratio*(L*(joints_num-1));
	by = 0;*/

	b = sqrt(bx*bx + by*by);

	n = joints_num;
	//m = n-2;

	cout << "Base-links ratio: " << custom_num << ", L: " << L << ", b: " << bx << endl;

	// Joint limits
	qminmax = 179.9/180*PI_;

	q_IK.resize(n);
	p_FK_l.resize(3);
	p_FK_r.resize(3);

	cout << "CKC initialized." << endl;
}

// ----- project -----

bool ckc::project(State &q, int nc, int IK_sol) {
	// nc - passive chain number

	State q_IK(3);
	State p_left(3), p_right(3), pose(3);

	if (nc < n-3) { // All passive chains except the last one
		if (nc==0)
			p_left = {0,0,0};
		else {
			FK_left(q, nc);
			p_left = get_FK_sol_left();
		}

		FK_right(q, n-3-nc);
		p_right = get_FK_sol_right();
		p_right[2] -= PI_;
		p_right[2] = fmod (p_right[2],  2*PI_);
		if (p_right[2] > PI_)
			p_right[2] -= 2*PI_;
		if (p_right[2] < -PI_)
			p_right[2] += 2*PI_;

		pose = {p_right[0]*cos(p_left[2]) - p_left[0]*cos(p_left[2]) - p_left[1]*sin(p_left[2]) + p_right[1]*sin(p_left[2]), p_right[1]*cos(p_left[2]) - p_left[1]*cos(p_left[2]) + p_left[0]*sin(p_left[2]) - p_right[0]*sin(p_left[2]), (p_right[2]-p_left[2])};
		if (pose[2] > PI_)
			pose[2] -= 2*PI_;
		if (pose[2] < -PI_)
			pose[2] += 2*PI_;

		if (IKp(pose, IK_sol, L))
			q_IK = get_IK_sol_q();
		else
			return false;

		q[nc] = q_IK[0];// + p_left[2];
		q[nc+1] = q_IK[1];
		q[nc+2] = q_IK[2];

	}
	if (nc == n-3)  { // The last passive chain (not including base) - special treatment

		FK_left_half(q, n-3);
		p_left = get_FK_sol_left();
		p_left[2] += PI_;
		p_left[2] = fmod (p_left[2],  2*PI_);
		if (p_left[2] > PI_)
			p_left[2] -= 2*PI_;
		if (p_left[2] < -PI_)
			p_left[2] += 2*PI_;

		pose = {p_left[0] - get_bx(), p_left[1] - get_by(), p_left[2]};
		if (pose[2] > PI_)
			pose[2] -= 2*PI_;
		if (pose[2] < -PI_)
			pose[2] += 2*PI_;

		if (IKp(pose, IK_sol, L))
			q_IK = get_IK_sol_q();
		else
			return false;

		/*if (q_IK[0] < 0)
			q_IK[0] += 2*PI_;*/

		q[n-1] = q_IK[0];
		q[n-2] = -q_IK[1];
		q[n-3] = -q_IK[2];

		if (q[n-1] < 0)	q[n-1] += 2*PI_;
		if (q[n-1] > 2*PI_) q[n-1] -= 2*PI_;
	}
	if (nc == n-2) { // Passive chain including the base and the right base joint

		p_left = {0,0,0};

		State qr(n-3);
		for (int i = n-2, j = 0; i > 1; i--, j++)
			qr[j] = -q[i];

		double x = L, y = 0, theta = 0, Lp;
		for (int i = 0; i < qr.size(); i++) {
			Lp = L;
			if (i==qr.size()-1)
				Lp = L/2;

			theta += qr[i];
			x += Lp*cos(theta);
			y += Lp*sin(theta);
		}

		p_right = {x, y, theta - PI_};
		p_right[2] = fmod (p_right[2],  2*PI_);
		if (p_right[2] > PI_)
			p_right[2] -= 2*PI_;
		if (p_right[2] < -PI_)
			p_right[2] += 2*PI_;

		pose = {p_right[0]*cos(p_left[2]) - p_left[0]*cos(p_left[2]) - p_left[1]*sin(p_left[2]) + p_right[1]*sin(p_left[2]), p_right[1]*cos(p_left[2]) - p_left[1]*cos(p_left[2]) + p_left[0]*sin(p_left[2]) - p_right[0]*sin(p_left[2]), (p_right[2]-p_left[2])};
		if (pose[2] > PI_)
			pose[2] -= 2*PI_;
		if (pose[2] < -PI_)
			pose[2] += 2*PI_;

		if (IKp(pose, IK_sol, b))
			q_IK = get_IK_sol_q();
		else
			return false;

		double al = atan(by/bx);

		q[n-1] = PI_ - q_IK[0] + al;
		q[0] = PI_ + q_IK[1] + al;
		q[1] = q_IK[2];

		if (q[0] < -PI_)	q[0] += 2*PI_;
		if (q[0] > PI_) q[0] -= 2*PI_;
		if (q[1] < -PI_)	q[1] += 2*PI_;
		if (q[1] > PI_) q[1] -= 2*PI_;
		if (q[n-1] < 0)	q[n-1] += 2*PI_;
		if (q[n-1] > 2*PI_) q[n-1] -= 2*PI_;
	}
	if (nc == n-1) { // Passive chain including the base and the left base joint

		State ql(n-3);
		for (int i = 1; i < n-2; i++)
			ql[i-1] = q[i];
		ql[0] = PI_ + ql[0];

		double x = 0, y = 0, theta = 0, Lp;
		for (int i = 0; i < ql.size(); i++) {
			Lp = L;
			if (i==ql.size()-1)
				Lp = L/2;

			theta += ql[i];
			x += Lp*cos(theta);
			y += Lp*sin(theta);
		}

		p_left = {x, y, theta + PI_};
		p_left[2] = fmod (p_left[2],  2*PI_);
		if (p_left[2] > PI_)
			p_left[2] -= 2*PI_;
		if (p_left[2] < -PI_)
			p_left[2] += 2*PI_;

		pose = {p_left[0] - L, p_left[1], p_left[2]};
		if (pose[2] > PI_)
			pose[2] -= 2*PI_;
		if (pose[2] < -PI_)
			pose[2] += 2*PI_;

		if (IKp(pose, IK_sol, b))
			q_IK = get_IK_sol_q();
		else
			return false;

		double al = atan(by/bx);

		q[0] = PI_ - q_IK[0] + al;
		q[n-1] = q_IK[1] + al;
		q[n-2] = -q_IK[2];

		if (q[0] < -PI_)	q[0] += 2*PI_;
		if (q[0] > PI_) q[0] -= 2*PI_;
		if (q[n-2] < -PI_)	q[n-2] += 2*PI_;
		if (q[n-2] > PI_) q[n-2] -= 2*PI_;
		if (q[n-1] < 0)	q[n-1] += 2*PI_;
		if (q[n-1] > 2*PI_) q[n-1] -= 2*PI_;
	}

	return true;
}

// -----FK-------

void ckc::FK_left(State q, int nd) {
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

	theta = fmod (theta,  2*PI_);
	if (theta > PI_)
		theta -= 2*PI_;
	if (theta < -PI_)
		theta += 2*PI_;

	p_FK_l = {x, y, theta};
}

void ckc::FK_left_half(State q, int nd) {
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

	theta = fmod (theta,  2*PI_);
	if (theta > PI_)
		theta -= 2*PI_;
	if (theta < -PI_)
		theta += 2*PI_;

	p_FK_l = {x, y, theta};
}

void ckc::FK_right(State q, int nd) {
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

	theta = fmod (theta,  2*PI_);
	if (theta > PI_)
		theta -= 2*PI_;
	if (theta < -PI_)
		theta += 2*PI_;

	p_FK_r = {x, y, theta};
}

State ckc::get_FK_sol_left() {
	return p_FK_l;
}

State ckc::get_FK_sol_right() {
	return p_FK_r;
}


// -------IK----------

bool ckc::IKp(State pose, int ik_sol, double L1) {
	IK_counter++;
	clock_t begin = clock();

	// Pose = {x3 ; y3; theta}
	State q(3);
	int sign;

	double theta = pose[2];
	double x2 = pose[0] - L/2 * cos(theta);
	double y2 = pose[1] - L/2 * sin(theta);

	double r = sqrt(x2*x2+y2*y2);
	double S = (r*r-L1*L1-L*L)/(-2*L1*L);
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
	q[1] = PI_+Phi;
	if (q[1] < -PI_)
		q[1] += 2*PI_;
	if (q[1] > PI_)
		q[1] -= 2*PI_;

	q[0] = atan2(L*sin(Phi), L1-L*cos(Phi)) + atan2(y2, x2);
	if (q[0] < -PI_)
		q[0] += 2*PI_;
	if (q[0] > PI_)
		q[0] -= 2*PI_;

	q[2] = theta - (q[0]+q[1]);
	if (q[2] < -PI_)
		q[2] += 2*PI_;
	if (q[2] > PI_)
		q[2] -= 2*PI_;

	q_IK = q;

	clock_t end = clock();
	IK_time += double(end - begin) / CLOCKS_PER_SEC;

	return true;
}

State ckc::get_IK_sol_q() {
	return q_IK;
}

State ckc::constraint(State q) {

	FK_left(q, q.size()-1);

	State C = get_FK_sol_left();

	C[0] -= bx;
	C[1] -= by;
	C[2] = C[2] - q[q.size()-1] + PI_;

	return C;
}


// ------- MISC ---------

void ckc::printMatrix(Matrix M) {
	for (unsigned i = 0; i < M.size(); i++) {
		for (unsigned j = 0; j < M[i].size(); j++)
			cout << M[i][j] << " ";
		cout << endl;
	}
}

void ckc::printVector(State p) {
	cout << "[";
	for (unsigned i = 0; i < p.size(); i++) {
		if (fabs(p[i])<1e-5)
			cout << 0 << " ";
		else
			cout << p[i] << " ";
	}
	cout << "]" << endl;
}

void ckc::log_q(State q) {
	std::ofstream myfile;
	myfile.open("../paths/path.txt");

	myfile << 1 << endl;

	for (int i = 0; i < q.size(); i++)
		myfile << q[i] << " ";
	myfile << endl;

	myfile.close();

	myfile.open("../paths/path_info.txt");
	myfile << n << endl << 0 << endl << getL() << endl << get_bx() << endl << get_by() << endl << get_qminmax() << endl;
	myfile.close();
}
