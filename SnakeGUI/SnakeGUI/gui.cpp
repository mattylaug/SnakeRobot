#include <string>
#include <sstream>
#include "SnakeRobot.h"
#include <Windows.h>
#include <GL/GLU.h>
#include<GL/glui.h>
#include <iostream>

#include <vector>
#include <cassert>
#include "importing.h"

# define M_PI           3.14159265358979323846

SnakeRobot* robot;

float xy_aspect;

/** These are the live variables passed into GLUI ***/
int   main_window;
float scale = 1.0;
float view_rotate[16] = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
float obj_pos[] = { 0.0, 0.0, 0.0 };
int   curTube = 0;

float O;
float dO;
float E;
float GJ;
float di;
float si;
float uijx;
float uijy;
float uijz;
float Px, Py, Pz;
float r=0, g=0, b=0;

GLfloat clickedDepth;
int counter1;

/** Pointers to the windows and some of the controls we'll create **/
GLUI *glui, *glui2;
GLUI_Panel *tube_panel;

GLUI_Listbox *tube_list;

GLUI_Spinner *e_spinner;
GLUI_Spinner *gj_spinner;
GLUI_Spinner *o_spinner;

GLUI_Spinner *do_spinner;
GLUI_Spinner *di_spinner;
GLUI_Spinner *si_spinner;

GLUI_Spinner *ux_spinner;
GLUI_Spinner *uy_spinner;
GLUI_Spinner *uz_spinner;

GLUI_Spinner *px_spinner;
GLUI_Spinner *py_spinner;
GLUI_Spinner *pz_spinner;

GLUI_Spinner *r_spinner;
GLUI_Spinner *g_spinner;
GLUI_Spinner *b_spinner;

GLUI_Button			*action_button;
GLUI_Button	*remove_button;

/********** User IDs for callbacks ********/
#define TUBE_LIST_ID         300
#define E_SPINNER_ID         301
#define GJ_SPINNER_ID        302
#define O_SPINNER_ID         303
#define DO_SPINNER_ID        304
#define DI_SPINNER_ID        305
#define SI_SPINNER_ID        306
#define UX_SPINNER_ID        307
#define UY_SPINNER_ID        308
#define UZ_SPINNER_ID        309

#define PX_SPINNER_ID        310
#define PY_SPINNER_ID        311
#define PZ_SPINNER_ID        312

#define ADD_TUBE_ID        313
#define REMOVE_TUBE_ID	314
#define COLOUR_ID        315



/********** Miscellaneous global variables **********/

GLfloat light0_ambient[] =  {0.1f, 0.1f, 0.3f, 1.0f};
GLfloat light0_diffuse[] =  {.6f, .6f, 1.0f, 1.0f};
GLfloat light0_position[] = {.5f, .5f, 1.0f, 0.0f};
GLfloat lights_rotation[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };

bool tipIsClicked;
int clickCount;

std::vector<aiVector3D> skull;

void updatePspinners() {
	px_spinner->set_float_val(robot->P(0));
	py_spinner->set_float_val(robot->P(1));

	pz_spinner->set_float_val(robot->P(2));

	px_spinner->redraw();
	py_spinner->redraw();
	pz_spinner->redraw();
	
}

void resetTubeBox() {

	curTube = 0;
	std::stringstream ss;
	ss << "Tube " << (curTube + 1);   // change the panel title to "Tube X"
	tube_panel->set_name(ss.str().c_str());
	e_spinner->set_float_val(robot->getTube(curTube)->E); // update the field editor with the internal tube variables 
	gj_spinner->set_float_val(robot->getTube(curTube)->GJ);
	o_spinner->set_float_val(robot->getTube(curTube)->O);
	do_spinner->set_float_val(robot->getTube(curTube)->dO);
	di_spinner->set_float_val(robot->getTube(curTube)->di);
	si_spinner->set_float_val(robot->getTube(curTube)->si);
	ux_spinner->set_float_val(robot->getTube(curTube)->uij(0));
	uy_spinner->set_float_val(robot->getTube(curTube)->uij(1));
	uz_spinner->set_float_val(robot->getTube(curTube)->uij(2));

	tube_panel->redraw();
	tube_list->redraw();
}

/**************************************** control_cb() *******************/
/* GLUI control callback                                                 */

void control_cb( int control )      /* main callback function */
{
	
  if ( control == TUBE_LIST_ID )    // if the selected tube has changed
  {
      std::stringstream ss;
      ss<< "Tube "<< (curTube+1);   // change the panel title to "Tube X"
      tube_panel->set_name(ss.str().c_str());
      e_spinner->set_float_val(robot->getTube(curTube)->E); // update the field editor with the internal tube variables 
      gj_spinner->set_float_val(robot->getTube(curTube)->GJ);
      o_spinner->set_float_val(robot->getTube(curTube)->O);
      do_spinner->set_float_val(robot->getTube(curTube)->dO);
      di_spinner->set_float_val(robot->getTube(curTube)->di);
      si_spinner->set_float_val(robot->getTube(curTube)->si);
      ux_spinner->set_float_val(robot->getTube(curTube)->uij(0));
      uy_spinner->set_float_val(robot->getTube(curTube)->uij(1));
      uz_spinner->set_float_val(robot->getTube(curTube)->uij(2)); 

	  r_spinner->set_float_val(robot->getTube(curTube)->color[0]);
	  g_spinner->set_float_val(robot->getTube(curTube)->color[1]);
	  b_spinner->set_float_val(robot->getTube(curTube)->color[2]);
  }
  else if( control == E_SPINNER_ID )            // all the following ifs respond to changes on the different field editors
  {                                                     // they only update the variable in the tubes of the snake so they can be rendered afterwards
      robot->getTube(curTube)->E=e_spinner->get_float_val();     //in each if statement, the xyz variables will be recalculated
	  robot->ForwardKinUpdateXYZ();
  }
  else if( control == GJ_SPINNER_ID )
  {
      robot->getTube(curTube)->GJ=gj_spinner->get_float_val();  
	  robot->ForwardKinUpdateXYZ();
  }
  else if( control == O_SPINNER_ID )
  {
      robot->getTube(curTube)->O=o_spinner->get_float_val();  
	  robot->ForwardKinUpdateXYZ();
  }
  else if( control == DO_SPINNER_ID )
  {
      robot->getTube(curTube)->dO=do_spinner->get_float_val(); 
	  robot->ForwardKinUpdateXYZ();
  }
  else if( control == DI_SPINNER_ID )
  {
      robot->getTube(curTube)->di=di_spinner->get_float_val();    
	  robot->ForwardKinUpdateXYZ();
  }
  else if( control == SI_SPINNER_ID )
  {
      robot->getTube(curTube)->si=si_spinner->get_float_val();  
	  robot->ForwardKinUpdateXYZ();
  }
  else if( control == UX_SPINNER_ID )
  {
      robot->getTube(curTube)->uij(0)=ux_spinner->get_float_val();  
	  robot->ForwardKinUpdateXYZ();
  }
  else if( control == UY_SPINNER_ID )
  {
      robot->getTube(curTube)->uij(1)=uy_spinner->get_float_val();    
	  robot->ForwardKinUpdateXYZ();
  }
  else if( control == UZ_SPINNER_ID )
  {
      robot->getTube(curTube)->uij(2)=uz_spinner->get_float_val();  
	  robot->ForwardKinUpdateXYZ();
  }

  
  else if( control == PX_SPINNER_ID )
  {
	  robot->P(0)=px_spinner->get_float_val();
	  robot->InvKinUpdateXYZ();
	  updatePspinners();
  }

  else if( control == PY_SPINNER_ID )
  {
	robot->P(1)=py_spinner->get_float_val();
	robot->InvKinUpdateXYZ();
	updatePspinners();
  }
  else if( control == PZ_SPINNER_ID )
  {
	robot-> P(2)=pz_spinner->get_float_val();
	robot->InvKinUpdateXYZ();
	updatePspinners();
	
  }
  else if (control == ADD_TUBE_ID) {
	  //Add new tube
	  Tube *tube;                         // adds 3 tubes and sets the corresponding parameters for each one of them
	  tube = new Tube();
	  tube->dO = sqrt(sqrt(4));
	  tube->O = M_PI / 3.0;
	  robot->AddTube(tube);

	  std::stringstream ss;
	  ss << "Tube " << robot->NumTubes();
	  tube_list->add_item(robot->NumTubes()-1, ss.str().c_str());
	  tube_list->redraw();

	  robot->ForwardKinUpdateXYZ();
	  updatePspinners();
  }

  else if (control == REMOVE_TUBE_ID) {
	  //Remove last tube
	  std::stringstream ss;
	  ss << "Tube " << robot->NumTubes();
	  tube_list->delete_item(ss.str().c_str());

	  robot->RemoveTube();
	  robot->ForwardKinUpdateXYZ();
	  updatePspinners();
	  resetTubeBox();
  }

  else if (control == COLOUR_ID) {
	  robot->tubes[curTube]->setColor(r_spinner->get_float_val(), g_spinner->get_float_val(), b_spinner->get_float_val());
	  robot->Render();
  }

}
/** Mouse **/
bool isInsideCircle(double x, double y, double z) {
	double tipX = robot->P(0), tipY = robot->P(1), tipZ = robot->P(2);
	// The mouse is inside if the distance to the center of the circle
	// is less than the radius
	double result = std::sqrt((x - tipX)*(x - tipX) + (y - tipY)*(y - tipY) + (z - tipZ)*(z - tipZ));
	return result < 32;
}

int mousex, mousey;
bool mouse_clicked, zIsSet;
int mouse_state;
// handles mouse click events
// button will say which button is presed, e.g. GLUT_LEFT_BUTTON, GLUT_RIGHT_BUTTON
// state will say if the button is GLUT_UP or GLUT_DOWN
// x and y are the poitner position 
void mouse_click(int button, int state, int mx, int my)
{
	mousex = mx;
	mousey = my;
	mouse_state = state;
	if (state == GLUT_DOWN) {
		mouse_clicked = true;
	}
	else if (state == GLUT_UP) {
		mouse_clicked = false;
		tipIsClicked = false;
		zIsSet = false;
	}


}
// drag function
void drag(int x, int y)
{
	if(mouse_clicked== true){
		mousex = x;
		mousey = y;
		glutPostRedisplay();
		counter1++;

	}
	
//    mouse_clicked = true;
	//
}

/**************************************** myGlutKeyboard() **********/

void myGlutKeyboard(unsigned char Key, int x, int y)
{
  switch(Key)
  {
  case 27:      // if the user presses ESC or q, quit the application
  case 'q':
    exit(0);
    break;
  };
  
  glutPostRedisplay();
}


/***************************************** myGlutMenu() ***********/

void myGlutMenu( int value )
{
  myGlutKeyboard( value, 0, 0 );
}


/***************************************** myGlutIdle() ***********/

void myGlutIdle( void )
{
  /* According to the GLUT specification, the current window is 
     undefined during an idle callback.  So we need to explicitly change
     it if necessary */
  if ( glutGetWindow() != main_window ) 
    glutSetWindow(main_window);  

  /*  GLUI_Master.sync_live_all();  -- not needed - nothing to sync in this
                                       application  */

  glutPostRedisplay();
}


/***************************************** myGlutMotion() **********/

void myGlutMotion(int x, int y )
{
  glutPostRedisplay(); 
}

/**************************************** myGlutReshape() *************/

void myGlutReshape( int x, int y )  // reshapes the display to the change of window parameters
{
  int tx, ty, tw, th;
  GLUI_Master.get_viewport_area( &tx, &ty, &tw, &th );
  glViewport( tx, ty, tw, th );

  xy_aspect = (float)tw / (float)th;

  glutPostRedisplay();
}


/************************************************** draw_axes() **********/
/* Disables lighting, then draws RGB axes                                */

void draw_axes( float scale )  
{
  glDisable( GL_LIGHTING );

  glPushMatrix();
  glScalef( scale, scale, scale );

  glBegin( GL_LINES );
 
  glColor3f( 1.0, 0.0, 0.0 );
  glVertex3f( 0.0, 0.0, 0.0 );  glVertex3f( 1.0, 0.0, 0.0 ); /* X axis      */

  glColor3f( 0.0, 1.0, 0.0 );
  glVertex3f( 0.0, 0.0, 0.0 );  glVertex3f( 0.0, 1.0, 0.0 ); /* Y axis      */

  glColor3f( 0.0, 0.0, 1.0 );
  glVertex3f( 0.0, 0.0, 0.0 );  glVertex3f( 0.0, 0.0, 1.0 ); /* Z axis    */
  glEnd();

  glPushMatrix();                       // draw a cone at the x axis tip
  glColor3f( 1.0, 0.0, 0.0 );
  glTranslatef(1.0, 0.0f, 0.0f);
  glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
  glutWireCone(0.027, 0.09, 10, 10);
  glPopMatrix();

  glPushMatrix();                       // draw a cone at the y axis tip
  glColor3f( 0.0, 1.0, 0.0 );
  glTranslatef(0.0, 1.0f, 0.0f);
  glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
  glutWireCone(0.027, 0.09, 10, 10);
  glPopMatrix();

  glPushMatrix();                       // draw a cone at the z axis tip
  glColor3f( 0.0, 0.0, 1.0 );
  glTranslatef(0.0, 0.0f, 1.0f);
  glRotatef(90.0f, 0.0f, 0.0f, 1.0f);
  glutWireCone(0.027, 0.09, 10, 10);
  glPopMatrix();

  glPopMatrix();

  glEnable( GL_LIGHTING );
}


/***************************************** myGlutDisplay() *****************/



void myGlutDisplay( void )          // this is the function that renders the scene 
{

	
  glClearColor( .0f, .1f, .1f, .0f );                       // clears the window
 // glClearDepth(0.5);
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT ); 

  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();
  glFrustum( -xy_aspect*.04, xy_aspect*.04, -.04, .04, .1, 15.0 );  // sets the perspective parameters

  glMatrixMode( GL_MODELVIEW );

  glLoadIdentity();                                     // sets the light 
  glMultMatrixf( lights_rotation );
  glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
  
  glLoadIdentity();
  glTranslatef( 0.0, 0.0, -2.6f );                      // sets the current view based on the position, rotation and scale selected by the user
  glTranslatef( obj_pos[0], obj_pos[1], -obj_pos[2] ); 
  glMultMatrixf( view_rotate );

  glScalef( scale, scale, scale );

  glPushMatrix();
  robot->Render();          // render the snake. This function should be changed so that there is a current xyzcoords variable within the snake class, 
							//and that whenever a variable changes the xyz coords are updated, and this render function simply renders the xyz coords.

  if (mouse_clicked) {

	  int mx = mousex, my = mousey;
	  GLdouble ox, oy, oz;
	  GLdouble tipX = robot->P(0), tipY = robot->P(1), tipZ = robot->P(2);
	  GLint viewport[4];
	  GLdouble modelview[16], projection[16];
	  GLfloat winX, winY, winZ;
	  glGetIntegerv(GL_VIEWPORT, viewport);
	  glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	  glGetDoublev(GL_PROJECTION_MATRIX, projection);
	  winX = (float)(mx + viewport[0]);
	  winY = (float)(viewport[3] - my + viewport[1]);

	  if(zIsSet == false){
		 
		glReadBuffer(GL_BACK);
		glReadPixels(winX, winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &clickedDepth);
		gluUnProject(winX, winY, clickedDepth, modelview, projection, viewport, &ox, &oy, &oz);
		tipIsClicked = isInsideCircle(ox, oy, oz);
		if (tipIsClicked) { zIsSet = true; }
	  }
	  if(zIsSet && tipIsClicked) {
		  glReadBuffer(GL_BACK);
		  gluUnProject(winX, winY, clickedDepth, modelview, projection, viewport, &ox, &oy, &oz);
		  Eigen::VectorXd ptarget(3);
		  ptarget << ox, oy, oz;
		  robot->P = ptarget;
		  robot->MoveToXYZ();
	  }
  }


  GLfloat no_mat[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat mat_ambient[] = { 0.7, 0.7, 0.2, 1.0 };
  GLfloat mat_ambient_color[] = { 0.3, 0.3, 0.0, 1.0 };

  GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat no_shininess[] = { 0.0 };
  GLfloat low_shininess[] = { 5.0 };
  GLfloat high_shininess[] = { 100.0 };
  GLfloat mat_emission[] = { 0.3, 0.2, 0.2, 0.0 };

  GLfloat mat_diffuse[] = { 1.0, 1.0, 1.0, 1.0f };
  glDisable(GL_COLOR_MATERIAL);
  glShadeModel(GL_SMOOTH);
 glRotatef(-90, 1, 0, 0);
 glRotatef(180, 0, 0, 1);
 glTranslatef(0,340,0);
  glScaled( 1, 1, 1);
glEnable(GL_NORMALIZE);

  for (int i = 0; i < skull.size(); i+=3) {
	  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_diffuse);
	  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
	  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
	  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, high_shininess);
	  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, no_mat);

	  glBegin(GL_TRIANGLES);
	  for(int j = 0; j<3; j++){
		  aiVector3D temp = skull[i+j];
		  float x = temp.x * 50, y = temp.y*50, z= temp.z*50;
		  
		  glNormal3f(x, y, z);
		  glVertex3f(x, y, z);
		  
	  }
	  glEnd();
}

  glPopMatrix();
  draw_axes(.2f);           // draw the axes

  glEnable(GL_LIGHTING);

  glutSwapBuffers(); 



}


/**************************************** main() ********************/

int main(int argc, char* argv[])
{
	FreeConsole();                      // gets rid of the console opened by default in windows
    robot = new SnakeRobot();           // create a new snake robot object
	Tube *tube;                         // adds 3 tubes and sets the corresponding parameters for each one of them
	tube = new Tube();
	tube->dO = sqrt(sqrt(4));
	tube->O = M_PI/3.0;
	robot->AddTube(tube);
	tube = new Tube();
	tube->setColor(1.0f, 0.0f, 0.0f);
	tube->dO = sqrt(sqrt(3));
	tube->O = -M_PI / 2.0;
	robot->AddTube(tube);
	tube = new Tube();
	tube->setColor(0.0f, 1.0f, 0.0f);
	tube->dO = sqrt(sqrt(2));
	tube->O = M_PI;
	robot->AddTube(tube);
	
	mouse_clicked = false;
	tipIsClicked = false;
	counter1 = 0;
	clickCount = 0;
	zIsSet = false;

	skull = loadModel("skull.3ds");
	int size1 = skull.size();
  /****************************************/
  /*   Initialize GLUT and create window  */
  /****************************************/

  glutInit(&argc, argv);                                        // uses the command line to set glu options
  glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );   // sets the display properties
  glutInitWindowPosition( 50, 50 );                             // sets window position and size
  glutInitWindowSize( 800, 600 );
 
  main_window = glutCreateWindow( "Snake Robot V1.0" );         // create main window and sets the window title
  glutDisplayFunc( myGlutDisplay );                             // sets the main render function
  GLUI_Master.set_glutReshapeFunc( myGlutReshape );             // sets the function called when the window parameters are changed (size, position)
  GLUI_Master.set_glutKeyboardFunc( myGlutKeyboard );           // sets the function called when a key is pressed
  GLUI_Master.set_glutSpecialFunc( NULL );
  glutMotionFunc( myGlutMotion );

  /****************************************/
  /*       Set up OpenGL lights           */
  /****************************************/

  glEnable(GL_LIGHTING);
  glEnable( GL_NORMALIZE );

  glEnable(GL_LIGHT0);
  glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
  glLightfv(GL_LIGHT0, GL_POSITION, light0_position);

  /****************************************/
  /*          Enable z-buferring          */
  /****************************************/
// glEnable(GL_DEPTH);
 glEnable(GL_DEPTH_TEST);
 // glDepthMask(1);
 // glDepthFunc(GL_LEQUAL);

  //glDepthFunc(GL_ALWAYS); 

  /****************************************/
  /*         Here's the GLUI code         */
  /****************************************/

  //printf( "GLUI version: %3.2f\n", GLUI_Master.get_version() );

  /*** Create the side subwindow ***/
  glui = GLUI_Master.create_glui_subwindow( main_window, 
					    GLUI_SUBWINDOW_RIGHT );

  /**** Add listbox ****/
  tube_list = new GLUI_Listbox( glui, "Edit:", &curTube,TUBE_LIST_ID,control_cb );
  int i;
  for( i=1; i<=3; i++ )
  {
    std::stringstream ss;
    ss<<"Tube "<<i;
    tube_list->add_item( i-1, ss.str().c_str() );
  }
  tube_list->set_alignment( GLUI_ALIGN_CENTER );
  
  new GLUI_StaticText( glui, "" );
  
  tube_panel = glui->add_panel("Tube 1");
  // spinner for editing the current tube's E value
  E=robot->getTube(0)->E;  
  e_spinner = new GLUI_Spinner( tube_panel, "E:", &E,E_SPINNER_ID,control_cb);
  e_spinner->set_float_limits( 0, 100 );
  e_spinner->set_alignment( GLUI_ALIGN_RIGHT );

  // spinner for editing the current tube's GJ value
  GJ=robot->getTube(0)->GJ;  
  gj_spinner = new GLUI_Spinner( tube_panel, "GJ:", &GJ,GJ_SPINNER_ID,control_cb);
  gj_spinner->set_float_limits( 0, 10 );
  gj_spinner->set_alignment( GLUI_ALIGN_RIGHT );

  // spinner for editing the current tube's O value
  O=robot->getTube(0)->O;  
  o_spinner = new GLUI_Spinner( tube_panel, "O:", &O,O_SPINNER_ID,control_cb);
  o_spinner->set_float_limits( -M_PI, M_PI );
  o_spinner->set_alignment( GLUI_ALIGN_RIGHT );

  // spinner for editing the current tube's dO value
  dO=robot->getTube(0)->dO;  
  do_spinner = new GLUI_Spinner( tube_panel, "do:", &dO,DO_SPINNER_ID,control_cb);
  do_spinner->set_float_limits( 1, 100 );
  do_spinner->set_alignment( GLUI_ALIGN_RIGHT );

  // spinner for editing the current tube's di value
  di=robot->getTube(0)->di;  
  di_spinner = new GLUI_Spinner( tube_panel, "di:", &di,DI_SPINNER_ID,control_cb);
  di_spinner->set_float_limits( 0, 100 );
  di_spinner->set_alignment( GLUI_ALIGN_RIGHT );

  // spinner for editing the current tube's si value
  si=robot->getTube(0)->si;  
  si_spinner = new GLUI_Spinner( tube_panel, "si:", &si,SI_SPINNER_ID,control_cb);
  si_spinner->set_float_limits( 1, 100 );
  si_spinner->set_alignment( GLUI_ALIGN_RIGHT );
  si_spinner->disable();

  r_spinner = new GLUI_Spinner(tube_panel, "R:", &r, COLOUR_ID, control_cb);
  r_spinner->set_alignment(GLUI_ALIGN_RIGHT);
  r_spinner->set_float_limits(0, 1);

  g_spinner = new GLUI_Spinner(tube_panel, "G:", &g, COLOUR_ID, control_cb);
  g_spinner->set_alignment(GLUI_ALIGN_RIGHT);
  g_spinner->set_float_limits(0, 1);

  b_spinner = new GLUI_Spinner(tube_panel, "B:", &b, COLOUR_ID, control_cb);
  b_spinner->set_alignment(GLUI_ALIGN_RIGHT);
  b_spinner->set_float_limits(0, 1);

  // makes a clickable panel to display the 3 components of the uij vector
  GLUI_Rollout *uij_roll = new GLUI_Rollout(tube_panel, "uij", false );

  // spinner for editing the current tube's uijx value
  uijx=robot->getTube(0)->uij(0);
  ux_spinner = new GLUI_Spinner( uij_roll, "x:", &uijx,UX_SPINNER_ID,control_cb);
  ux_spinner->set_float_limits( 0, 1 );
  ux_spinner->set_alignment( GLUI_ALIGN_RIGHT );

  // spinner for editing the current tube's uijy value
  uijy=robot->getTube(0)->uij(1);
  uy_spinner = new GLUI_Spinner( uij_roll, "y:", &uijy,UY_SPINNER_ID,control_cb);
  uy_spinner->set_float_limits( 0, 1 );
  uy_spinner->set_alignment( GLUI_ALIGN_RIGHT );

  // spinner for editing the current tube's uijz value
  uijz=robot->getTube(0)->uij(2);
  uz_spinner = new GLUI_Spinner( uij_roll, "z:", &uijz,UZ_SPINNER_ID,control_cb);
  uz_spinner->set_float_limits( 0, 1 );
  uz_spinner->set_alignment( GLUI_ALIGN_RIGHT );

  //For inverse kinematics
  GLUI_Rollout *p_roll = new GLUI_Rollout(tube_panel, "P:", false);
  // spinner for editing the current tube's Px value
  Px = robot->P(0);
  px_spinner = new GLUI_Spinner(p_roll, "x:", &Px, PX_SPINNER_ID, control_cb);
  px_spinner->set_float_limits(-500, 500);
  px_spinner->set_alignment(GLUI_ALIGN_RIGHT);

  // spinner for editing the current tube's Py value
  Py = robot->P(1);
  py_spinner = new GLUI_Spinner(p_roll, "y:", &Py, PY_SPINNER_ID, control_cb);
  py_spinner->set_float_limits(-500, 500);
  py_spinner->set_alignment(GLUI_ALIGN_RIGHT);

  // spinner for editing the current tube's Pz value
  Pz = robot->P(2);
  pz_spinner = new GLUI_Spinner(p_roll, "z:", &Pz, PZ_SPINNER_ID, control_cb);
  pz_spinner->set_float_limits(-500, 500);
  pz_spinner->set_alignment(GLUI_ALIGN_RIGHT);



  action_button = new GLUI_Button(tube_panel, "Add tube", ADD_TUBE_ID, control_cb);

  remove_button = new GLUI_Button(tube_panel, "Remove tube", REMOVE_TUBE_ID, control_cb);



  /******** Add some controls for lights ********/
  new GLUI_StaticText( glui, "" );

  /****** A 'quit' button *****/
  new GLUI_Button( glui, "Quit", 0,(GLUI_Update_CB)exit );


  /**** Link windows to GLUI, and register idle callback ******/
  
  glui->set_main_gfx_window( main_window );


  /*** Create the bottom subwindow ***/
  glui2 = GLUI_Master.create_glui_subwindow( main_window, 
                                             GLUI_SUBWINDOW_BOTTOM );
  glui2->set_main_gfx_window( main_window );

  GLUI_Rotation *view_rot = new GLUI_Rotation(glui2, "Rotate", view_rotate );   // bind the rotation control to the variable view_rotate
  view_rot->set_spin( 1.0 );
  new GLUI_Column( glui2, false );

  GLUI_Translation *trans_xy = 
    new GLUI_Translation(glui2, "Pan", GLUI_TRANSLATION_XY, obj_pos );  // bind the translation control to the variable obj_pos in x and y
  trans_xy->set_speed( .005 );
  new GLUI_Column( glui2, false );

  GLUI_Translation *trans_z = 
    new GLUI_Translation( glui2, "Zoom", GLUI_TRANSLATION_Z, &obj_pos[2] ); // bind the zoom control to the variable obj_pos in z
  trans_z->set_speed( .005 );

  robot->ForwardKinUpdateXYZ();

  px_spinner->set_float_val(robot->P(0));
  py_spinner->set_float_val(robot->P(1));
  pz_spinner->set_float_val(robot->P(2));

  glutMotionFunc(drag);
  glutMouseFunc(mouse_click);
 

  /**** Regular GLUT main loop ****/
  

  glutMainLoop();

  return EXIT_SUCCESS;
}

