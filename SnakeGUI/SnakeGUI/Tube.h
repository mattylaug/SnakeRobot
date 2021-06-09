#ifndef _TUBE_H_
#define _TUBE_H_
# define M_PI           3.14159265358979323846

class Tube
{
public:
	float color[3]; // color r,g,b

	double O;       // variables used for generating the xyz positions
	double dO;

	double E;
	double GJ;
	double di;
	double si;
	Eigen::VectorXd uij;

	Tube()
	{		
		color[0]=1; // white color by default
		color[1]=1;
		color[2]=1;

		//default values taken from setupvariables
		Eigen::VectorXd temp(3);
		temp << 0, 1.0 / 100.0, 0;
		uij = temp;
		GJ = 5;
		E = 64.0 / M_PI; // stress / strain; //Sets E for each tube.
		di = 0;
		si = 100.0;
	}

	void setColor(float r,float g,float b)
	{
	    color[0]=r;
	    color[1]=g;
	    color[2]=b;
	}
};

#endif
