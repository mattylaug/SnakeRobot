#include "SnakeRobot.h"
using namespace Eigen;

// Renders the snake using the properties from the tubes
//
/*
void SnakeRobot::Render()
{
	glDisable(GL_COLOR_MATERIAL);
	glShadeModel(GL_SMOOTH);
	glRotatef(90, 0.0f, 1.0f, 0.0f);
	glScaled( 1 / length(), 1 / length(), 1 / length());
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

    initialize();   // initialize all the values needed for generating the xzy points 
    //double **xyz=obtainxyz();   // generate the xyz points and save them in a new array
	
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
			vn[i].col(0) = Eigen::Vector3d(xyz[i][0], xyz[i][1], xyz[i][2]) - Eigen::Vector3d(xyz[i - 1][0], xyz[i - 1][0], xyz[i - 1][0]);
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
	
		GLfloat mat_diffuse[] = { tempTube->color[0], tempTube->color[1], tempTube->color[2], 1.0f};    // set material properties for displaying
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_diffuse);   
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
		glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, high_shininess);
		glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, no_mat);

		double thetaInc = 2.0 * M_PI / detail;      // we will go around a complete circunference using a step of theteinc

		double r = tempTube->dO / 2.0;      // we use the dO data as the exterior diameter of the tube
		for (int j = 0; j < segmentsPerTube; j++)		
		{
			int l=i*segmentsPerTube+j;
			if (l >= numXyzPoints-1)
				break;
			double theta0 = 0.0,theta1;
			for (int k = 0; k < detail; k++)
			{	
				theta1 = theta0 + thetaInc;
				
				Eigen::Vector3d v0 = Eigen::Vector3d(xyz[l][0], xyz[l][1], xyz[l][2]);      // we use the current point and the next point to build a quad of a cylinder 
				Eigen::Vector3d v1 = Eigen::Vector3d(xyz[l+1][0], xyz[l+1][1], xyz[l+1][2]);
				Eigen::Vector3d p1 = (r*vn[l].col(1) * cos(theta0) + r*vn[l].col(2) * sin(theta0)) + v0;    // we use the precalculated orthogonal vectors to build a segment of the circunference
				Eigen::Vector3d p2 = (r*vn[l].col(1) * cos(theta1) + r*vn[l].col(2) * sin(theta1)) + v0;
				Eigen::Vector3d p3 = (r*vn[l + 1].col(1) * cos(theta1) + r*vn[l + 1].col(2) * sin(theta1)) + v1;
				Eigen::Vector3d p4 = (r*vn[l + 1].col(1) * cos(theta0) + r*vn[l + 1].col(2) * sin(theta0)) + v1;
				glBegin(GL_QUADS);
					Eigen::Vector3d n = (p2 - p1).cross(p4 - p1);   // we calculate the normal to the quad using three points
					glNormal3f(n[0],n[1],n[2]);
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
		product += ((Rz[j] * K[j])*uibar[j]); /// NOTE: using uibar for uij
	}
	ui = leftsum.inverse() * product; //Resultant curvature vector, ui.
	return ui;
}

MatrixXd SnakeRobot::shapeRender(double uz, double uy, double ux) { //creates uhat matrix. What is uz,uy,ux? 
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


void SnakeRobot::initialize() {
    int numTubes=tubes.size();
	Rz.resize(numTubes);
	K.resize(numTubes);
	ui.resize(numTubes);
	distalFrames.resize(numTubes);
	uhat.resize(numTubes);
	uibar.resize(numTubes);
	gi.resize(numTubes);

	int i, j;
	for (i = 0; i < numTubes; i++) {
		K[i] = tubeStiffness(tubes[i]->E,tubes[i]->GJ,tubes[i]->dO,tubes[i]->di); //Inputs E, GJ, d0,di, outputs K matrix for each section and stores in array.
	}
	for (j = 0; j < numTubes; j++) { //Creates array of Rz and Kj matrices. 
		Rz[j] = createRotationMatrix(tubes[j]->O); //Input theta, output Rz matrix for each section and stores in array
		uibar[j] = natSpatialCurvature(tubes[j]->uij[0],tubes[j]->uij[1],tubes[j]->uij[2]); //Inputs uijx, uijy, uij, outputs uij vector for each section and stores in array
	}
	
	for (int currentSection = 0; currentSection < numTubes; currentSection++) {
		ui[currentSection] = resultantCurvature(currentSection);  //This will output the resultant curvature at current section, ui, into an array
	}

	for (j = 0; j < numTubes; j++) { //Creates array of Rz and Kj matrices. 
		uhat[j] = shapeRender(ui[j](2), ui[j](1), ui[j](0));	//obtains uhat matrix by using ux, uy, and uz
	}
	for (j = 0; j < numTubes; j++) {	//Getting issues here
		gi[j] = distalFrame(ui[j], tubes[j]->si, uhat[j]); //aka gi
	}
}



double ** SnakeRobot::obtainxyz() { //si = length, changeInDistance = along each tube, the distance between each xyz vector
VectorXd ui1(3);
MatrixXd endPosition;
MatrixXd b(3, 1);
MatrixXd R(3, 3);
MatrixXd result(4, 4);
MatrixXd g1(4, 4);
MatrixXd *xyzPoints=new MatrixXd[numXyzPoints];
VectorXd v(4);
VectorXd P(3);
VectorXd xyz(4);
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

double changeInDistance=length()/numXyzPoints;
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
ui1 = resultantCurvature(d);
newresult = result * distalFrame(ui1, distance, uhat[d]);	//Multiplies previous gi matrices with current tube gi matrix
xyz = newresult*v; //Multiplies all the products of distal frames of the tubes
xyzPoints[count] = xyz; //I should return this matrix instead of converting to a double array

distance += changeInDistance;
count++;
}
distance = 0;
}
}
else {
//Do nothing, because tube 1 already calculated
}
double ** xyzAr = 0;
xyzAr = new double *[numXyzPoints];
for (int i = 0; i < numXyzPoints; i++) {
xyzAr[i] = new double[3];
for (int j = 0; j <= 2; j++) {
xyzAr[i][j] = xyzPoints[i](j, 0);
}
}
return xyzAr;

} 

*/



