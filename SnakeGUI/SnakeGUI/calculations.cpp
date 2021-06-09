#include "SnakeRobot.h"
# define M_PI           3.14159265358979323846
using namespace Eigen;

	
	void SnakeRobot::Render()
	{
		numXyzPoints = currentXYZCoords.size();
		glDisable(GL_COLOR_MATERIAL);
		glShadeModel(GL_SMOOTH);
		glRotatef(90, 0.0f, 1.0f, 0.0f);
		glScaled(1 / length(), 1 / length(), 1 / length());
		//    glScaled( 1 / 200., 1 / 200., 1 / 200.);
		glEnable(GL_NORMALIZE);

		GLfloat no_mat[] = { 0.0, 0.0, 0.0, 1.0 };
		GLfloat mat_ambient[] = { 0.7, 0.7, 0.2, 1.0 };
		GLfloat mat_ambient_color[] = { 0.3, 0.3, 0.0, 1.0 };
		GLfloat mat_diffuse[] = { 0.8, 0.8, 0.2, 1.0 };
		GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
		GLfloat no_shininess[] = { 0.0 };
		GLfloat low_shininess[] = { 5.0 };
		GLfloat high_shininess[] = { 100.0 };
		GLfloat mat_emission[] = { 0.3, 0.2, 0.2, 0.0 };

									  //xyz will be different whenever the render function is called, depending on whether inv kinematics or forward kinematics has been applied

		Tube *tempTube;

		std::vector<Matrix3d> vn(numXyzPoints); // create an array of vectors

												// here, we will build a set of orthonormal vectors for every point in the generated xyz points
												// these vectors will be used as a basis for the display of the snake
		for (int i = 0; i < numXyzPoints; i++)
		{
			vn[i] = Matrix3d::Zero();

			if (i == 0)
				vn[i].col(0) = Eigen::Vector3d(0, 0, 1);    // we will always use the first direction vector as z 
			else                                            // we will take the next direction vectors as the difference between the points, thus
															// the vector 0 will always be tangent to the snake's curvature
				vn[i].col(0) = Eigen::Vector3d(currentXYZCoords[i](0), currentXYZCoords[i](1), currentXYZCoords[i](2)) - Eigen::Vector3d(currentXYZCoords[i - 1](0), currentXYZCoords[i - 1](0), currentXYZCoords[i - 1](0));
				vn[i].col(0).normalize();                       // normalize vector 0 (direction vector) 

				vn[i].col(1) = Eigen::Vector3d(vn[i].col(0)[1] + vn[i].col(0)[2], vn[i].col(0)[0], vn[i].col(0)[0]);    // obtain a vector perpendicular to the 0 vector
				vn[i].col(1).normalize();                       // normalize it

				vn[i].col(2) = vn[i].col(0).cross(vn[i].col(1));    // obtain a vector perpendicular to both vector0 and vector1,  
				vn[i].col(2).normalize();                           // normalize the vector2, with this we obtain a set of three normalized, orthogonal vectors
		}

		int segmentsPerTube = numXyzPoints / tubes.size();

		//	recursing all the tubes
		for (int i = 0; i < tubes.size(); i++)
		{
			tempTube = tubes[i];

			GLfloat mat_diffuse[] = { tempTube->color[0], tempTube->color[1], tempTube->color[2], 1.0f };    // set material properties for displaying
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_diffuse);
			glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
			glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
			glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, high_shininess);
			glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, no_mat);

			double thetaInc = 2.0 * M_PI / detail;      // we will go around a complete circunference using a step of theteinc

			double r = tempTube->dO / 2.0;      // we use the dO data as the exterior diameter of the tube
			for (int j = 0; j < segmentsPerTube; j++)
			{
				int l = i*segmentsPerTube + j;
				if (l >= numXyzPoints - 1)
					break;
				double theta0 = 0.0, theta1;
				for (int k = 0; k < detail; k++)
				{
					theta1 = theta0 + thetaInc;

					Eigen::Vector3d v0 = Eigen::Vector3d(currentXYZCoords[l](0), currentXYZCoords[l](1), currentXYZCoords[l](2));      // we use the current point and the next point to build a quad of a cylinder 
					Eigen::Vector3d v1 = Eigen::Vector3d(currentXYZCoords[l + 1](0), currentXYZCoords[l + 1](1), currentXYZCoords[l + 1](2));
					Eigen::Vector3d p1 = (r*vn[l].col(1) * cos(theta0) + r*vn[l].col(2) * sin(theta0)) + v0;    // we use the precalculated orthogonal vectors to build a segment of the circunference
					Eigen::Vector3d p2 = (r*vn[l].col(1) * cos(theta1) + r*vn[l].col(2) * sin(theta1)) + v0;
					Eigen::Vector3d p3 = (r*vn[l + 1].col(1) * cos(theta1) + r*vn[l + 1].col(2) * sin(theta1)) + v1;
					Eigen::Vector3d p4 = (r*vn[l + 1].col(1) * cos(theta0) + r*vn[l + 1].col(2) * sin(theta0)) + v1;
					glBegin(GL_QUADS);
					Eigen::Vector3d n = (p2 - p1).cross(p4 - p1);   // we calculate the normal to the quad using three points
					glNormal3f(n[0], n[1], n[2]);
					glVertex3f(p1[0], p1[1], p1[2]);
					glVertex3f(p2[0], p2[1], p2[2]);
					glVertex3f(p3[0], p3[1], p3[2]);
					glVertex3f(p4[0], p4[1], p4[2]);
					theta0 = theta1;
					glEnd();
				}

			}
		}
	}

	void SnakeRobot::updateTubeSegments() {
		
		for (int i = 0; i < tubes.size(); i++) {
			tubes[i]->O = q(((i + 1) * 2) - 2);
			tubes[i]->si = q(((i + 1) * 2) - 1);
		}
	}

	void SnakeRobot::updateqSegments() {
		q.resize(tubes.size()*2);
		for (int i = 0; i < tubes.size(); i++) {
			q(((i + 1) * 2) - 2) = tubes[i]->O;
			q(((i + 1) * 2) - 1) = tubes[i]->si;
		}
	}

	void SnakeRobot::InvKinUpdateXYZ() {
		updateqSegments();

		q = position_inverseKinematics(P, q, K);
		currentXYZCoords = DrawTubes();
		updateTubeSegments();

		Render();
	}

	void SnakeRobot::MoveToXYZ() {
		updateqSegments();

		q = quick_inverseKinematics(P, q);
		currentXYZCoords = DrawTubes();
		updateTubeSegments();
		P = currentXYZCoords[currentXYZCoords.size()-1];
		Render();
	}

	void SnakeRobot::ForwardKinUpdateXYZ() {
		initialize();   // initialize all the values needed for generating the xyz points 
		currentXYZCoords = obtainxyz();
		//update P with new tip point
		P = currentXYZCoords[currentXYZCoords.size() - 1];
		//update q after
		updateqSegments();
		Render();
	}

	MatrixXd SnakeRobot::tubeStiffness(double E, double GJ, double d0, double di) { //K matrix -- forumla 1
		double I = (M_PI / 64.0)*((d0 * d0 * d0 * d0) - (di *di *di *di));
		double EI = E*I;
		MatrixXd K(3, 3);
		K << EI, 0, 0,
			0, EI, 0,
			0, 0, GJ;

		return K;
	}

	double SnakeRobot::areaMomentInertia(double d0, double di) {
		double I = (M_PI / 64.0f)*(d0 * d0 * d0 * d0 - di *di *di *di);
		return I;
	}

	VectorXd SnakeRobot::natSpatialCurvature(double uijx, double uijy, double uijz) {
		VectorXd uij(3);			//Natural spatial curvature
		uij << uijx,
			uijy,
			uijz;
		return uij;
	}
	VectorXd SnakeRobot::resultSpatialCurvature(double uix, double uiy, double uiz) {
		VectorXd ui(3);	//Reultant spatial curvature
		ui << uix, uiy, uiz;
		return ui;
	}

	VectorXd SnakeRobot::resultantCurvature(int i) { //outputs ui, resultant curvature vector -- 3
		VectorXd ui(3);
		MatrixXd leftsum(3, 3), product(3, 1);
		leftsum << 0, 0, 0,
			0, 0, 0,
			0, 0, 0;
		product << 0,
			0,
			0;
		for (int j = i; j < tubes.size(); j++) {
			leftsum += (K[j]);
			product += ((Rz[j] * K[j])*uibar[i]);
		}
		ui = leftsum.inverse() * product; //Resultant curvature vector, ui.
		return ui;
	}

	MatrixXd SnakeRobot::shapeRender(double uz, double uy, double ux) { //creates uhat matrix. 
		MatrixXd uhat(3, 3);
		uhat << 0, -uz, uy,
			uz, 0, -ux,
			-uy, ux, 0;
		return uhat;
	}

	MatrixXd SnakeRobot::createRotationMatrix(double O) { // Rz 0j - forumla 4. Inputs theta, outputs Rz matrix.
		MatrixXd R1(3, 3);
		R1(0, 0) = cos(O);
		R1(0, 1) = -sin(O);
		R1(0, 2) = 0;
		R1(1, 0) = sin(O);
		R1(1, 1) = cos(O);
		R1(1, 2) = 0;
		R1(2, 0) = 0;
		R1(2, 1) = 0;
		R1(2, 2) = 1;
		return R1;
	}

	MatrixXd SnakeRobot::distalFrame(VectorXd ui, double si, MatrixXd uhat) { //inputs ui vector, outputs gi matrix.

		MatrixXd I(3, 3); //Identity matrix
		I << 1, 0, 0,
			0, 1, 0,
			0, 0, 1;

		double ui1 = sqrt((ui(0)*ui(0)) + (ui(1)*ui(1)) + (ui(2)*ui(2))); //absolute value from ui vector
																		  //double si = 2.0; //length 

		VectorXd vi(3);
		vi << 0, 0, 1;
		MatrixXd R(3, 3);
		R << 1, 0, 0,
			0, 1, 0,
			0, 0, 1;
		MatrixXd b(3, 1);


		MatrixXd m(3, 3);
		m << 1, 0, 0,
			0, 1, 0,
			0, 0, 1;
		if (ui1 == 0) {
			R = I;
			b = si*vi;
		}
		else {
			R = m + (sin(ui1*si) / ui1)* uhat + (((1.00000000f - cos(ui1*si)) / (ui1 *ui1)) * (uhat * uhat));
			b = (si*I + (((1.0000000f - cos(ui1*si)) / (ui1*ui1)))*uhat + ((((ui1*si) - sin(ui1*si)) / (ui1*ui1*ui1)))*(uhat*uhat))*vi;
		}
		MatrixXd g(4, 4);
		g << R(0, 0), R(0, 1), R(0, 2), b(0),
			R(1, 0), R(1, 1), R(1, 2), b(1),
			R(2, 0), R(2, 1), R(2, 2), b(2),
			0, 0, 0, 1;

		return g;

	}

	std::vector<VectorXd> SnakeRobot::obtainxyz() { //si = length, changeInDistance = along each tube, the distance between each xyz vector
		VectorXd ui1(3);
		MatrixXd endPosition;
		MatrixXd b(3, 1);
		MatrixXd R(3, 3);
		MatrixXd result(4, 4);
		MatrixXd g1(4, 4);
		std::vector<VectorXd> xyzPoints(numXyzPoints);
		VectorXd v(4);
		VectorXd P(3);
		VectorXd xyz(4);

		VectorXd rcUi(3);
		v << 0,
			0,
			0,
			1;
		MatrixXd P1(3, 1);
		P1 << 0,
			0,
			0;
		int count = 0;
		double distance = 0;
		double changeInDistance = length() / numXyzPoints;
		R = gi[0].block(0, 0, 3, 3);
		while (distance < tubes[0]->si && count < numXyzPoints) { //Creates xyz vectors for first section
			g1 = distalFrame(ui[0], distance, uhat[0]);
			b = g1.block(0, 3, 3, 1);
			P = R*P1 + b;
			xyzPoints[count] = P;
			distance += changeInDistance;
			count++;
		}

		distance = 0;
		MatrixXd newresult(4, 4);
		if (tubes.size() > 1) {

			for (int d = 1; d < tubes.size(); d++) { //For each tube
				result = gi[0]; //Initializes for product

				for (int k = 1; k < d; k++) { //Gets the product of all gi matrixes up to the specific tube number
					result = result * gi[k];
				}

				while (count < numXyzPoints && distance < tubes[d]->si) { //For each xyz 
					ui1 = resultantCurvature(d); //could make newresult = result * gi[d] and the result should be the same?
					newresult = result * distalFrame(ui1, distance, uhat[d]);	//Multiplies previous gi matrices with current tube gi matrix
					xyz = newresult*v; //Multiplies all the products of distal frames of the tubes
					xyzPoints[count] = xyz.head(3);
					distance += changeInDistance;
					count++;
				}
				distance = 0;
			}
		}
		else {
			//Do nothing, because tube 1 already calculated
		}
		return xyzPoints;

	}


	void SnakeRobot::initialize() {
		int numTubes = tubes.size();
		Rz.resize(numTubes);
		K.resize(numTubes);
		ui.resize(numTubes);
		distalFrames.resize(numTubes);
		uhat.resize(numTubes);
		uibar.resize(numTubes);
		gi.resize(numTubes);

		int i, j;
		for (i = 0; i < numTubes; i++) {
			K[i] = tubeStiffness(tubes[i]->E, tubes[i]->GJ, tubes[i]->dO, tubes[i]->di); //Inputs E, GJ, d0,di, outputs K matrix for each section and stores in array.
		}
		for (j = 0; j < numTubes; j++) { //Creates array of Rz and Kj matrices. 
			Rz[j] = createRotationMatrix(tubes[j]->O); //Input theta, output Rz matrix for each section and stores in array
			uibar[j] = natSpatialCurvature(tubes[j]->uij(0), tubes[j]->uij(1), tubes[j]->uij(2)); //Inputs uijx, uijy, uij, outputs uij vector for each section and stores in array
		
		}

		for (int currentSection = 0; currentSection < numTubes; currentSection++) {
			ui[currentSection] = resultantCurvature(currentSection);  //This will output the resultant curvature at current section, ui, into an array
		}

		for (j = 0; j < numTubes; j++) { //Creates array of Rz and Kj matrices. 
			uhat[j] = shapeRender(ui[j](2), ui[j](1), ui[j](0));	//obtains uhat matrix by using ux, uy, and uz
		}
		for (j = 0; j < numTubes; j++) {
			gi[j] = distalFrame(ui[j], tubes[j]->si, uhat[j]); //aka gi
		}

	}

	/**
	Inverse kinematics Below
	**/

	VectorXd SnakeRobot::position_inverseKinematics(VectorXd P, VectorXd q0, std::vector<MatrixXd> K) { //Depedencies: ForwardKinematics, JacobianMatrix(), 
																										/*
																										% calculate the inverse kinematics given the current joint variables and the target position
																										% Input:
																										%	P : target position, 3 by 1 vector
																										%	q0 : current joint variables, 2n by 1 vector q0 = [theta_1, s_1, theta_2, s_2, theta_3, s_3, …theta_n, s_n]
																										% Output :
																										%	q : desired joint variables, 2n by 1 vector
																										%	convergence : a boolean indicating whether the algorithm converges. 0 means not converging; 1 means converging.
																										*/
		int n = tubes.size();//number of sections

		VectorXd e_Z(3);
		e_Z << 1000000, 1000000, 1000000; //initialize with a very large number 1000000
		VectorXd q = q0;
		MatrixXd g;
		VectorXd Pe;
		MatrixXd J_Z;
		MatrixXd result = MatrixXd::Zero(3, 6);
		MatrixXd H;
		MatrixXd x;
		MatrixXd J0;
		MatrixXd f;
		int k = 0;
		int convergence;
		while (e_Z.norm() > 1e-6) {

			g = ForwardKinematics(q, K); //g is a 4 by 4 matrix, Forward Kinematics, multiply all the g_i together to get the matrix

			Pe = g.block(0, 3, 3, 1);  //g(1:3, 4) means the submatrix of g with rows 1:3 and columns 4 : 4 (having only one column, it is a vector actually)
			e_Z = P - Pe;

			J0 = JacobianMatrix(q, K); // J0 is a 6 by 2n matrix, Jacobian Matrix, will be defined below

			result.block(0, 0, 3, 3) = -skew(Pe);
			result.block(0, 3, 3, 3) = MatrixXd::Identity(3, 3);

			J_Z = result * J0;

			f = J_Z.transpose() * e_Z; //'*e_Z; %J_Z' is the transpose of J_Z

			H = J_Z.transpose()*J_Z + 0.5*e_Z.transpose()*e_Z * MatrixXd::Identity(2 * n, 2 * n);
			x = pseudoInverse(H)*f; //inv means the inverse of a matrix

			q = q + x;
			k = k + 1;

			if (k > 1000) {
				convergence = 0; //not converging
				return q;
			}
		}
		convergence = 1; //converging	
		return q;
	}

	VectorXd SnakeRobot::quick_inverseKinematics(VectorXd P, VectorXd q0) { //Depedencies: ForwardKinematics, JacobianMatrix(), 
																										/*
																										% calculate the inverse kinematics given the current joint variables and the target position
																										% Input:
																										%	P : target position, 3 by 1 vector
																										%	q0 : current joint variables, 2n by 1 vector q0 = [theta_1, s_1, theta_2, s_2, theta_3, s_3, …theta_n, s_n]
																										% Output :
																										%	q : desired joint variables, 2n by 1 vector
																										%	convergence : a boolean indicating whether the algorithm converges. 0 means not converging; 1 means converging.
																										*/
		
		int n = tubes.size();//number of sections
	    VectorXd e_Z(3);
		e_Z << 1000000, 1000000, 1000000; //initialize with a very large number 1000000
		VectorXd q = q0;
		MatrixXd g;
		VectorXd Pe;
		MatrixXd J_Z;
		MatrixXd result = MatrixXd::Zero(3, 6);
		MatrixXd H;
		MatrixXd x;
		MatrixXd J0;
		MatrixXd f;

		g = ForwardKinematics(q, K); //g is a 4 by 4 matrix, Forward Kinematics, multiply all the g_i together to get the matrix

		Pe = g.block(0, 3, 3, 1);  //g(1:3, 4) means the submatrix of g with rows 1:3 and columns 4 : 4 (having only one column, it is a vector actually)
		e_Z = P - Pe;

		J0 = JacobianMatrix(q, K); // J0 is a 6 by 2n matrix, Jacobian Matrix, will be defined below

		result.block(0, 0, 3, 3) = -skew(Pe);
		result.block(0, 3, 3, 3) = MatrixXd::Identity(3, 3);

		J_Z = result * J0;

		f = J_Z.transpose() * e_Z; //J_Z' is the transpose of J_Z

		H = J_Z.transpose()*J_Z + 0.5*e_Z.transpose()*e_Z * MatrixXd::Identity(2 * n, 2 * n);
		x = pseudoInverse(H)*f; 

		q = q + x;
		return q;
	}


	MatrixXd SnakeRobot::se3Exp(MatrixXd kesi) { //Returns 4x4
		double n1 = kesi.block(0, 0, 3, 1).norm();
		double n2 = kesi.block(3, 0, 3, 1).norm();
		double DELTA = 10 ^ (-12);
		MatrixXd T;
		if (abs(n1) < DELTA) {
			if (abs(n2)<DELTA) {
				T = MatrixXd::Identity(4, 4);
			}
			else {
				T = se3Translation(kesi.block(3, 0, 3, 1) / n2, n2);
			}
		}
		else {
			T = se3Rotation(kesi.block(0, 0, 3, 1) / n1, kesi.block(3, 0, 3, 1) / n1, n1);
		}
		return T;
	}

	MatrixXd SnakeRobot::se3Translation(MatrixXd v, double theta) {
		MatrixXd T(4, 4);
		T.block(0, 0, 3, 3) = MatrixXd::Identity(3, 3);
		T.block(0, 3, 3, 1) = v*theta;
		T.block(3, 0, 1, 4) << 0, 0, 0, 1;
		return T;
	}

	MatrixXd SnakeRobot::se3Rotation(MatrixXd w, MatrixXd v, double theta) {
		MatrixXd R, p;
		R = rotationMatrix(w, theta);
		p = (theta*MatrixXd::Identity(3, 3) + (1 - cos(theta))*skew(w) + (theta - sin(theta))*skew(w)*skew(w))*v;

		MatrixXd T(4, 4);
		T.block(0, 0, 3, 3) = R;
		T.block(0, 3, 3, 1) = p;
		T.block(3, 0, 1, 4) << 0, 0, 0, 1;

		return T;
	}

	MatrixXd SnakeRobot::rotationMatrix(VectorXd w, double theta) {
		MatrixXd R;
		R = MatrixXd::Identity(3, 3) + skew(w)*sin(theta) + skew(w)*skew(w)*(1 - cos(theta));
		return R;
	}

	MatrixXd SnakeRobot::sumOfK(int n) {
		MatrixXd Ksum = MatrixXd::Zero(3, 3);
		for (int i = n; i <tubes.size(); i++) {
			Ksum = Ksum + K[i];
		}
		return Ksum;
	}

	MatrixXd SnakeRobot::JacobianMatrix(VectorXd q, std::vector<MatrixXd> K) { //outputs J
																			   /*
																			   % Jacobian matrix
																			   %Input:
																			   % 	q = [theta_1, s_1, theta_2, s_2, theta_3, s_3, …theta_n, s_n]
																			   % Output :
																			   %	J is a 6 by 2n matrix
																			   */

		int n = tubes.size();
		double * s;
		double * theta;
		s = new double[n];
		theta = new double[n];
		for (int i = 0; i <n; i++) {
			theta[i] = q((2 * (i + 1)) - 2); //theta used to be 0
			s[i] = q((2 * (i + 1)) - 1);
		}

		VectorXd * u1;
		u1 = new VectorXd[n];

		MatrixXd * A;
		A = new MatrixXd[n];

		MatrixXd * gi;
		gi = new MatrixXd[n];

		MatrixXd xi = GenerateTwist(q, K);
		for (int i = 0; i < n; i++) {

		}

		MatrixXd ** b; // is 3d matrix
		b = new MatrixXd*[n];
		for (int i = 0; i < n; i++) {
			b[i] = new MatrixXd[n];
			for (int j = 0; j < n; j++) {
				b[i][j] = MatrixXd::Zero(1, 6);
			}
		}

		MatrixXd * Adg;
		Adg = new MatrixXd[n];
		MatrixXd delta(n, n);

		std::vector<MatrixXd> gs(n);

		MatrixXd ksum(3, 3);
		MatrixXd temp;

		for (int i = 0; i<n; i++) {

			gs[i] = se3Exp(xi.block(0, i, 6, 1)*s[i]);
			A[i] = aMatrix(xi.block(0, i, 6, 1), s[i]);

			for (int j = 0; j<n; j++) {

				if (i>j) {
					b[i][j] = MatrixXd::Zero(6, 1); // %b is a n by n by 6 array
				}
				else {
					ksum = Sum(K, i, n);
					b[i][j] = MatrixXd::Zero(6, 1);
					temp = ksum.inverse()*K[j] * dRz(theta[j])*tubes[j]->uij;
					b[i][j] << temp(0), temp(1), temp(2), 0, 0, 0;
				}
			}

			if (i == 0) {
				Adg[i] = MatrixXd::Identity(6, 6);
			}
			else {
				Adg[i] = adM(Product1(gs, 0, i - 1)); // %adM() and Product() will be defined later
			}
		}
		MatrixXd J = MatrixXd::Zero(6, 2 * n);

		for (int j = 0; j<n; j++) {
			J.block(0, (2 * (j + 1)) - 2, 6, 1) << 0, 0, 0, 0, 0, 0;

			for (int i = 0; i<n; i++) {
				J.block(0, (2 * (j + 1)) - 2, 6, 1) = J.block(0, (2 * (j + 1)) - 2, 6, 1) + Adg[i] * A[i] * b[i][j];
			}

			J.block(0, (2 * (j + 1)) - 1, 6, 1) = J.block(0, (2 * (j + 1)) - 1, 6, 1) + Adg[j] * xi.block(0, j, 6, 1);

		}
		return J;
	}

	/*
	MatrixXd SnakeRobot::DexteritySphere(VectorXd q0, std::vector<VectorXd> uF, std::vector<MatrixXd> K, int Ntheta, int Nphi) {
		double dtheta = 2 * M_PI / Ntheta;

		double dphi = M_PI / Nphi;

		MatrixXd FeasibleOrientation = MatrixXd::Zero(Ntheta, Nphi);

		MatrixXd g0 = ForwardKinematics(q0, K);

		MatrixXd R0 = g0.block(0, 0, 3, 3); // (1:3,1:3) initial rotation matrix

		MatrixXd P0 = g0.block(0, 3, 3, 1); // (1:3, 4); initial tip position

		VectorXd qc;

		int convergence;

		for (int i = 1; i < Ntheta; i++) {
			qc = q0;

			for (int j = 1; i < Nphi; i++) {
				
				MatrixXd Rt = R0*rotz((i - 0.5)*dtheta)*rotz((j - 0.5)*dphi); //target rotation 

				MatrixXd gt(4 , 4);
				gt.block(0, 0, 3, 3) = Rt;
				gt.block(0, 2, 3, 1) = P0;
				gt.block(3,0,1,4) << 0, 0, 0, 1;   // target transformation
				MatrixXd qt;
				MatrixXd * result;
				result = InverseKinematicsRemoveOmegaZ(gt, qc, uF, K); //find a feasible solution
				qt = result[0];
				covergence = result[1](0,0);
				if (covergence != 0) {  //% found a solution
					FeasibleOrientation(i, j) = 1;
					qc = qt; //update current configuration
				}
				else {

					FeasibleOrientation(i, j) = 0;



					break; // break for j
				}

			}

		}
	}
	*/

	/*
	MatrixXd *  SnakeRobot::InverseKinematicsRemoveOmegaZ(VectorXd gt, VectorXd q0, std::vector<MatrixXd> uF, std::vector<MatrixXd> K) {	
		int n = tubes.size() ; 

		double e_removeZ = INFINITE;

		VectorXd q = q0;

		int k = 0;

		VectorXd e_Z(3);
		e_Z << 1000000, 1000000, 1000000; //initialize with a very large number 1000000
		VectorXd q = q0;
		MatrixXd g;
		VectorXd Pe;
		MatrixXd J_Z;
		MatrixXd result = MatrixXd::Zero(3, 6);
		MatrixXd H;
		MatrixXd x;
		MatrixXd J0;
		MatrixXd f;
		int k = 0;
		int convergence;
		while (e_Z.norm() > 1e-6) {
			//e_removeZ.norm();
			g = ForwardKinematics(q, K); //g is a 4 by 4 matrix, Forward Kinematics, multiply all the g_i together to get the matrix

			Pe = g.block(0, 3, 3, 1);  //g(1:3, 4) means the submatrix of g with rows 1:3 and columns 4 : 4 (having only one column, it is a vector actually)
			e_Z = P - Pe;

			J0 = JacobianMatrix(q, K); // J0 is a 6 by 2n matrix, Jacobian Matrix, will be defined below
			MatrixXd J1 = adM(g.inverse())*J0; // 6 by 2n matrix

			result.block(0, 0, 3, 3) = -skew(Pe);
			result.block(0, 3, 3, 3) = MatrixXd::Identity(3, 3);

			J_Z = result * J0;

			f = J_Z.transpose() * e_Z; //'*e_Z; %J_Z' is the transpose of J_Z

			H = J_Z.transpose()*J_Z + 0.5*e_Z.transpose()*e_Z * MatrixXd::Identity(2 * n, 2 * n);
			x = pseudoInverse(H)*f; //inv means the inverse of a matrix

			q = q + x;
			k = k + 1;

			if (k > 1000) {
				convergence = 0; //not converging
				return q;
			}
		}
		convergence = 1; //converging	
		return q;


	

}

	double rotationTheta(MatrixXd g) {
		double tr = (g.block(0, 0,3,3).diagonal().sum() - 1) / 2;
		if (tr > 1) {
			tr = 1;
		}
		else if (tr < -1) {
			tr = -1;
		}
		return acos(tr);
	}
	MatrixXd vlogR(MatrixXd R) {
		// w is the rotation axis of a rotation matrix R
		double DELTA = 10 ^ (-12);
		double tr = (R.diagonal().sum() - 1) / 2;
		MatrixXd w;
		if (tr > 1) {
			tr = 1;
		}
		else if (tr < -1) {
			tr = -1;

		}
		double fai = acos(tr);

		if (abs(fai) < DELTA) {
			//MatrixXd w(3, 1);
			w << 0, 0, 0;
		}
	
		else if ( abs(fai - M_PI) < DELTA){
			//warning('Logarithm of rotation matrix with angle PI.');
		}
		MatrixXd eigvals = R.eigenvalues();
		//[V, D] 
		MatrixXd V = eigvals.col(0).real();
		MatrixXd D = eigvals.col(1).real();
		
		if (D.maxCoeff() == D(1));
			w = V(:, 1);
		else if max(D) == D(2);
			w = V(:, 2);
		else
			w = V(:, 3);
		end
			if max(max(cos(fai)*eye(3) + (1 - cos(fai))*(w*w')+sin(fai)*skew(w)-R))>DELTA
				w = -w;
		end
			w = w*fai;
			else
				w = fai / (2 * sin(fai))*[R(3, 2) - R(2, 3); R(1, 3) - R(3, 1); R(2, 1) - R(1, 2)];
		end

	}
	MatrixXd vlog(MatrixXd g) {
		MatrixXd p;
		double DELTA = pow(10, -12);
		double fai = rotationTheta(g);
		MatrixXd w = vlogR(g.block(0,0,3,3));
		
		if (abs(fai) < DELTA) {
			p = g.block(0,3,3,1);
		}
		else {
			p = (eye(3) - 0.5*skew(w) + (2 * sin(fai) - fai*(1 + cos(fai))) / (2 * fai*fai*sin(fai))*skew(w)*skew(w))*g(1:3, 4);
		}
			xi = [w; p];
	}
	*/
	MatrixXd SnakeRobot::skew(VectorXd v) {
		//% cross matrix of a vector
		MatrixXd S(3, 3);
		S << 0, -v(2), v(1),
			v(2), 0, -v(0),
			-v(1), v(0), 0;
		return S;
	}



	MatrixXd SnakeRobot::aMatrix(VectorXd xi, double s) { //tested
														  //% Input:
														  //%	xi is a 6 by 1 vector, s is a scaler
														  //%Output :
														  //%	aM is a 6 by 6 matrix
		MatrixXd aM(6, 6);
		VectorXd w(3);
		w = xi.head(3); //(1:3);

		MatrixXd v = xi.tail(3); //(4:6)

		MatrixXd bW = MatrixXd::Zero(6, 6);

		bW.block(0, 0, 3, 3) = skew(w);

		bW.block(3, 0, 3, 3) = skew(v);

		bW.block(3, 3, 3, 3) = skew(w);
		double n;
		n = w.norm();

		double t = n*s;
		if (n == 0) {

			aM = s*MatrixXd::Identity(6, 6);

		}
		else {
			MatrixXd ident = MatrixXd::Identity(6, 6);

			aM = s* ident + (((4 - t*sin(t)) - (4 * cos(t))) / 2 / (n*n))*bW + ((4 * t - 5 * sin(t) + t*cos(t)) / 2 / n*n*n)*(bW*bW) + ((2 - t*sin(t) - 2 * cos(t)) / 2 / n*n*n)*(bW*bW*bW) + ((2 * t - 3 * sin(t) + t*cos(t)) / 2 / n*n*n*n*n)*(bW*bW*bW*bW);

		}
		return aM;
	}

	MatrixXd SnakeRobot::GenerateTwist(VectorXd q, std::vector<MatrixXd> K) {
		int n = tubes.size();
		MatrixXd left = MatrixXd::Zero(3, 3);
		MatrixXd right = MatrixXd::Zero(3, 1);
		MatrixXd * ufW = new MatrixXd[n];
		MatrixXd xi = MatrixXd::Zero(6, n);

		for (int i = 0; i < n; i++) {
			left = left + K[i];
			right = right + K[i] * rotz(q((2 * (i + 1)) - 2)) * tubes[i]->uij;
		}
		for (int i = 0; i < n; i++) {
			ufW[i] = left.inverse()*right;
			left = left - K[i];
			right = right - K[i] * rotz(q((2 * (i + 1)) - 2)) * tubes[i]->uij;
			xi.block(0, i, 6, 1) << ufW[i](0), ufW[i](1), ufW[i](2), 0, 0, 1;

		}
		return xi;
	}

	MatrixXd SnakeRobot::ForwardKinematics(VectorXd q, std::vector<MatrixXd> K) {
		int n = tubes.size();
		MatrixXd xi = GenerateTwist(q, K);
		MatrixXd g = MatrixXd::Identity(4, 4);
		for (int i = 0; i < n; i++) {
			g = g * se3Exp(xi.block(0, i, 6, 1)*q(((i + 1) * 2) - 1));
		}
		return g;
	}

	MatrixXd SnakeRobot::rotz(double t) {
		double ct = cos(t);
		double st = sin(t);
		MatrixXd R(3, 3);
		R << ct, -st, 0,
			st, ct, 0,
			0, 0, 1;
		return R;
	}

	MatrixXd SnakeRobot::adM(MatrixXd m) {
		//adjoint matrix
		//Input:
		//	m is a 4 by 4 matrix
		//Output :
		//   ad is a 6 by 6 matrix
		MatrixXd r(3, 3);
		r = m.block(0, 0, 3, 3);
		MatrixXd p = m.block(0, 3, 3, 1);
		MatrixXd ad(6, 6);
		ad = MatrixXd::Zero(6, 6);
		ad.block(0, 0, 3, 3) = r;
		ad.block(3, 0, 3, 3) = skew(p)*r;
		ad.block(3, 3, 3, 3) = r;
		return ad;
	}

	MatrixXd SnakeRobot::dRz(double theta) {
		//% derivative of Rz
		//%Input:
		//%	theta is a scaler
		//%Output :
		//%   	M is a 3 by 3 matrix
		MatrixXd M(3, 3);
		M << -sin(theta), -cos(theta), 0,
			cos(theta), -sin(theta), 0,
			0, 0, 0;
		return M;
	}

	MatrixXd SnakeRobot::Sum(std::vector<MatrixXd> K, int i, int n) {
		//% sum of K from i to n
		//%Input:
		//%	K is a 3 by 3 matrix, i is a scaler, n is a scaler
		//%Output :
		//S is a 3 by 3 matrix

		MatrixXd S = MatrixXd::Zero(3, 3);
		for (int j = i; j<n; j++) {
			S += K[j];
		}
		return S;
	}

	MatrixXd SnakeRobot::Product1(std::vector<MatrixXd> gi, int i, int n) {
		/*% product of gi from i to n
		%Input:
		%	gi is a 4 by 4 matrix, i is a scaler, n is a scaler
		%Output :
		%   	P is a 4 by 4 matrix
		*/
		MatrixXd P(4, 4);
		P = MatrixXd::Identity(4, 4);
		for (int j = i; j <= n; j++) {
			P *= gi[j];
		}
		return P;
	}

	std::vector<Eigen::VectorXd> SnakeRobot::DrawTubes() {
		int n = tubes.size();

		MatrixXd xi = GenerateTwist(q, K);
		MatrixXd gi = MatrixXd::Identity(4, 4);
		MatrixXd ni = MatrixXd::Zero(n, 1);

		double sum_ni = 1;

		for (int i = 0; i < n; i++) {
			ni(i) = floor(q(2 * (i + 1) - 1) / d) + 1;
			sum_ni = sum_ni + ni(i);
		}

		int k = 1;

		//MatrixXd Point = MatrixXd::Zero(3, sum_ni);
		std::vector<VectorXd> Pt(sum_ni-1);
		MatrixXd g;
		for (int i = 0; i < n; i++) {

			for (int j = 1; j < ni(i); j++) {
				g = gi*se3Exp(xi.block(0, i, 6, 1) * j *d);
				Pt[k - 1] = g.block(0, 3, 3, 1);
				k++;
			}
			g = gi*se3Exp(xi.block(0, i, 6, 1)*q((2 * (i + 1)) - 1));
			gi = g;
			Pt[k - 1] = g.block(0, 3, 3, 1);
			k++;
		}

		return Pt;
	}

