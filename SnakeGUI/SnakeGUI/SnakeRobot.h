#ifndef _SNAKEROBOT_H_
#define _SNAKEROBOT_H_
#include <iostream>
#include <math.h>
#include <GL/glut.h>
#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <vector>
#include "Tube.h"

class SnakeRobot
{
private:
	std::vector<Eigen::VectorXd> currentXYZCoords;// = SnakeRobot::obtainxyz();
	Eigen::VectorXd targetPosition;
	
	double position[3];

	double d=5;

    std::vector<Eigen::MatrixXd> Rz;
    std::vector<Eigen::MatrixXd> K;
    std::vector<Eigen::VectorXd> ui;
    std::vector<Eigen::MatrixXd> distalFrames;
    std::vector<Eigen::MatrixXd> uhat;
    std::vector<Eigen::VectorXd> uibar; 
    std::vector<Eigen::VectorXd> uij; //uibar = uF
    std::vector<Eigen::MatrixXd> gi;

	//std::vector<Eigen::VectorXd> uF;

	
	/*
	MatrixXd * Rz;
	MatrixXd * K;
	VectorXd * ui;
	MatrixXd *distalFrames;
	MatrixXd *uhat;
	VectorXd *uibar;
	VectorXd * uij;
	MatrixXd *gi;
	double * siArray;
	double * E;
	double * O;
	double * GJ;
	double * dO;
	double * di;
	double * si;
	*/

	//std::vector<Eigen::VectorXd> q; //INVKIN
	//std::vector<Eigen::VectorXd> currentXYZcoords
	void SnakeRobot::updateTubeSegments();
	void SnakeRobot::updateqSegments();



	Eigen::MatrixXd tubeStiffness(double E, double GJ, double d0, double di);
	double areaMomentInertia(double d0, double di);
	Eigen::VectorXd natSpatialCurvature(double uijx, double uijy, double uijz);
	Eigen::VectorXd resultSpatialCurvature(double uix, double uiy, double uiz);
	Eigen::VectorXd resultantCurvature(int i);
	Eigen::MatrixXd shapeRender(double uz, double uy, double ux);
	Eigen::MatrixXd createRotationMatrix(double O);
	Eigen::MatrixXd distalFrame(Eigen::VectorXd ui, double si, Eigen::MatrixXd uhat);
	//void setUpVariables();
	//double ** xyzCustomVariables(int numT, double E1[], double O1[], double GJ1[], double dO1[], double di1[], double si1[], VectorXd uij1[]);
	std::vector<Eigen::VectorXd> SnakeRobot::DrawTubes();
	Eigen::MatrixXd GenerateTwist(Eigen::VectorXd q, std::vector<Eigen::MatrixXd> K);
	Eigen::MatrixXd rotz(double t);
	Eigen::VectorXd position_inverseKinematics(Eigen::VectorXd P, Eigen::VectorXd q0, std::vector<Eigen::MatrixXd> K);
	Eigen::MatrixXd ForwardKinematics(Eigen::VectorXd q, std::vector<Eigen::MatrixXd> K);
	Eigen::MatrixXd Product1(std::vector<Eigen::MatrixXd> gi, int i, int n);
	Eigen::MatrixXd JacobianMatrix(Eigen::VectorXd q, std::vector<Eigen::MatrixXd> K);
	Eigen::MatrixXd skew(Eigen::VectorXd v);
	Eigen::MatrixXd aMatrix(Eigen::VectorXd xi, double s);
	Eigen::MatrixXd dRz(double theta);
	Eigen::MatrixXd Sum(std::vector<Eigen::MatrixXd> K, int i, int n);
	Eigen::MatrixXd adM(Eigen::MatrixXd m);
	Eigen::MatrixXd se3Exp(Eigen::MatrixXd kesi);
	Eigen::MatrixXd se3Translation(Eigen::MatrixXd v, double theta);
	Eigen::MatrixXd se3Rotation(Eigen::MatrixXd w, Eigen::MatrixXd v, double theta);
	Eigen::MatrixXd rotationMatrix(Eigen::VectorXd w, double theta);
	Eigen::MatrixXd sumOfK(int n);
	Eigen::MatrixXd * InverseKinematicsRemoveOmegaZ(Eigen::VectorXd gt, Eigen::VectorXd q0, std::vector<Eigen::MatrixXd> uF, std::vector<Eigen::MatrixXd> K);
	Eigen::MatrixXd DexteritySphere(Eigen::VectorXd q0, std::vector<Eigen::VectorXd> uF, std::vector<Eigen::MatrixXd> K, int Ntheta, int Nphi);
	Eigen::VectorXd quick_inverseKinematics(Eigen::VectorXd P, Eigen::VectorXd q0);

	std::vector<Eigen::VectorXd> obtainxyz();

	template<typename _Matrix_Type_>
	_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
	{
		Eigen::JacobiSVD< _Matrix_Type_ > svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
		double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
		return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
	}

   
public:
	std::vector<Tube*>	tubes;
	Eigen::VectorXd P;
	Eigen::VectorXd q;
    int numXyzPoints;
	int	detail;

	SnakeRobot()
	{
		setPosition(0, 0, -7);

		detail = 30; // number of cylinder segments per tube
        
        numXyzPoints=60;    // number of points to render the complete snake

		Eigen::VectorXd temp(3);
		temp << 0, 0, 0;

		P = temp;

		Eigen::VectorXd temp2(6);
		temp2 << 0, 0, 0, 0, 0, 0;
		q = temp2;
	}

	~SnakeRobot()
	{
		while (tubes.size() > 0)
		{
			Tube * tube = tubes.back();
			delete tube;

			tubes.pop_back();
		}

	}	

	int NumTubes() {
		return tubes.size();
	}

	void RemoveTube()
	{
		tubes.pop_back();
	}

	void AddTube(Tube* tube)
	{
		tubes.push_back(tube);
		
	}
    Tube* getTube(int nTube)
    {
        return tubes[nTube];
    }

    // calculates the current length of the snake by summing over the tube lengths
    double length()
    {
        double l=0.0;
        for(int i=0; i<tubes.size(); i++)
            l+=tubes[i]->si;
        return l;
    }

    void setPosition(double x,double y,double z)
    {
        position[0]=x;
        position[1]=y;
        position[2]=z;
    }

	/*
	VectorXd getTipLocation(){
			currentXYZcoords[numXyzPoints-1];
	}

	void InvKinUpdateXYZ(){
		getTipLocation();
	}

	*/
	//Eigen::VectorXd SnakeRobot::quick_inverseKinematics(Eigen::VectorXd P);
	void ForwardKinUpdateXYZ();
	void InvKinUpdateXYZ();
	void Render();
//	void updateTubeSegments();
//	void updateqSegments();
    void initialize();    
	void MoveToXYZ();
};

#endif
