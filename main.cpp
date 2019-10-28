#include <iostream>
#include <string>
#include <vector>
#include <glew.h>
#include <GLFW/glfw3.h>
#include "GLUT/GL/glut.h"
#include "GLM/glm/matrix.hpp"
#include "GLM/glm/gtc/matrix_transform.hpp"
#include "cugl.h"

using namespace cugl;
using namespace std;

int windowWidth = 800;
int windowHeight = 800;

// Material data
GLfloat red[] = { 0.9, 0.3, 0.3, 1.0 };
GLfloat green[] = { 0.3, 0.9, 0.3, 1.0 };
GLfloat blue[] = { 0.3, 0.3, 0.9, 1.0 };
GLfloat white[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat shiny[] = { 50 };
GLfloat dir[] = { 0.0, 0.0, 1.0, 0.0 };

class Link
{
public:
    Link(double length, double radius, GLfloat *col) : length(length), radius(radius), col(col)
    {
        bar = gluNewQuadric();
        gluQuadricDrawStyle(bar, GLU_FILL);
        gluQuadricOrientation(bar, GLU_OUTSIDE);
        gluQuadricNormals(bar, GLU_SMOOTH);
    }

    void draw()
    {
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, col);
        ori.apply();
        gluCylinder(bar, radius, radius, length, 20, 20);
        glTranslated(0, 0, length);
        glutSolidSphere(1.5 * radius, 20, 20);
        for (vector<Link*>::const_iterator i = pLinks.begin(); i != pLinks.end(); ++i)
        {
            glPushMatrix();
            (*i)->draw();
            glPopMatrix();
        }
    }

    void addLink(Link *p)
    {
        pLinks.push_back(p);
    }

    void setRot(Quaternion newOri)
    {
        ori = newOri;
    }

    double getLength()
    {
        return length;
    }

private:
    double length;
    double radius;
    GLfloat *col;
    Quaternion ori;
    vector<Link*> pLinks;
    GLUquadricObj *bar;
};

Link *top_link;
Link *middle_link;
Link* bottom_link;

double init_x;
double init_y;
double final_x = -10; //final x position of the tip
double final_y = -15; //final y position of the tip
double curr_x; //the current position of the tip
double curr_y; //the current position of the tip

void build()
{
    top_link = new Link(10, 1.5, red);
    top_link->setRot(Quaternion(J, 0));
    middle_link = new Link(7.5, 0.75, green);
    middle_link->setRot(Quaternion(J, 0));
    top_link->addLink(middle_link);
    bottom_link = new Link(5, 0.25, blue);
    bottom_link->setRot(Quaternion(J, 0));
    middle_link->addLink(bottom_link);

    //we need to calculate the initial position of the tip (x and y coordinates) using the formulas:
    // x = Acos(alpha) + Bcos(alpha + beta) + Ccos(alpha + beta + gamma)
    // y = Asin(alpha) + Bsin(alpha + beta) + Csin(alpha + beta + gamma)

    //the initial angles are all 90
    init_x = 0;
    init_y = top_link->getLength() + middle_link->getLength() + bottom_link->getLength();
    curr_x = init_x;
    curr_y = init_y;
}

void display (void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslated(-15, -10, -100);
    glRotated(-90, 1, 0, 0);


    glutSolidSphere(3, 20, 20);
    top_link->draw();

    glutSwapBuffers();
}

Quaternion upperQuat = Quaternion(J, 0);
Quaternion middleQuat = Quaternion(J, 0);
Quaternion lowerQuat = Quaternion(J, 0);

double sp = 0.0008;

void idle()
{
    //if we are at the final position no need to move anything
    if(abs(final_x - curr_x) < 0.0001 && abs(final_y - curr_y) < 0.0001)
        return;

    //we have to recalculate the jacobian (J) everytime
    //J is a 3x2 matrix
    //we take partial derivatives of
    // x = Acos(alpha) + Bcos(alpha + beta) + Ccos(alpha + beta + gamma)
    // y = Asin(alpha) + Bsin(alpha + beta) + Csin(alpha + beta + gamma)
    // J[0][0] = -Asin(alpha) - Bsin(alpha + beta) - Csin(alpha + beta + gamma)
    // J[1][0] = -Bsin(alpha + beta) - Csin(alpha + beta + gamma)
    // J[2][0] = -Csin(alpha + beta + gamma)
    // J[0][1] = Acos(alpha) + Bcos(alpha + beta) + Ccos(alpha + beta + gamma)
    // J[1][1] = Bcos(alpha + beta) + Ccos(alpha + beta + gamma)
    // J[2][1] = Ccos(alpha + beta + gamma)

    double Jacobian[3][2];
    Jacobian[0][0] = -top_link->getLength()*sin(upperQuat.angle())
                     -middle_link->getLength()*sin(upperQuat.angle() + middleQuat.angle())
                     -bottom_link->getLength()*sin(upperQuat.angle() + middleQuat.angle() + lowerQuat.angle());
    Jacobian[1][0] = -middle_link->getLength()*sin(upperQuat.angle() + middleQuat.angle())
                     -bottom_link->getLength()*sin(upperQuat.angle() + middleQuat.angle() + lowerQuat.angle());
    Jacobian[2][0] = -bottom_link->getLength()*sin(upperQuat.angle() + middleQuat.angle() + lowerQuat.angle());
    Jacobian[0][1] = top_link->getLength()*cos(upperQuat.angle()) +
                     middle_link->getLength()*cos(upperQuat.angle() + middleQuat.angle()) +
                     bottom_link->getLength()*cos(upperQuat.angle() + middleQuat.angle() + lowerQuat.angle());
    Jacobian[1][1] = middle_link->getLength()*cos(upperQuat.angle() + middleQuat.angle()) +
                     bottom_link->getLength()*cos(upperQuat.angle() + middleQuat.angle() + lowerQuat.angle());
    Jacobian[2][1] = bottom_link->getLength()*cos(upperQuat.angle() + middleQuat.angle() + lowerQuat.angle());

    //this is the transpose of the jacobian matrix
    double JacobianTranspose[2][3];
    JacobianTranspose[0][0] = Jacobian[0][0];
    JacobianTranspose[0][1] = Jacobian[1][0];
    JacobianTranspose[0][2] = Jacobian[2][0];
    JacobianTranspose[1][0] = Jacobian[0][1];
    JacobianTranspose[1][1] = Jacobian[1][1];
    JacobianTranspose[1][2] = Jacobian[2][1];

    //this is the jacobian multiplied by its transpose to give a 2x2 matrix
    double JJT[2][2];

    JJT[0][0] = Jacobian[0][0]*JacobianTranspose[0][0] +
                Jacobian[1][0]*JacobianTranspose[0][1] +
                Jacobian[2][0]*JacobianTranspose[0][2];
    JJT[1][0] = Jacobian[0][0]*JacobianTranspose[1][0] +
                Jacobian[1][0]*JacobianTranspose[1][1] +
                Jacobian[2][0]*JacobianTranspose[1][2];
    JJT[0][1] = Jacobian[0][1]*JacobianTranspose[0][0] +
                Jacobian[1][1]*JacobianTranspose[0][1] +
                Jacobian[2][1]*JacobianTranspose[0][2];
    JJT[1][1] = Jacobian[0][1]*JacobianTranspose[1][0] +
                Jacobian[1][1]*JacobianTranspose[1][1] +
                Jacobian[2][1]*JacobianTranspose[1][2];

    //now we need the inverse of JJT
    //to do this we need to calculate the determinant
    double detJJT = JJT[0][0]*JJT[1][1] - JJT[0][1]*JJT[1][0];

    //if the determinant is essentially zero we have a singularity
    if(detJJT < 0.00002)
    {
        cout << "determinant is zero - singularity" << endl;
        lowerQuat *= Quaternion(J, 1/sp);
        bottom_link->setRot(lowerQuat);
    }

    else
    {
        //otherwise we can calculate the inverse if JJT
        double JJTinv[2][2];
        JJTinv[0][0] = JJT[0][0]/detJJT;
        JJTinv[0][1] = JJT[0][1]/detJJT;
        JJTinv[1][0] = JJT[1][0]/detJJT;
        JJTinv[1][1] = JJT[1][1]/detJJT;

        //next we need the pseudoinverse which is JacobianTranspose multiples by JJTinv
        double pseudoInv[2][3];

        pseudoInv[0][0] = JacobianTranspose[0][0]*JJTinv[0][0] + JacobianTranspose[1][0]*JJTinv[0][1];
        pseudoInv[1][0] = JacobianTranspose[0][0]*JJTinv[1][0] + JacobianTranspose[1][0]*JJTinv[1][1];
        pseudoInv[0][1] = JacobianTranspose[0][1]*JJTinv[0][0] + JacobianTranspose[1][1]*JJTinv[0][1];
        pseudoInv[1][1] = JacobianTranspose[0][1]*JJTinv[1][0] + JacobianTranspose[1][1]*JJTinv[1][1];
        pseudoInv[0][2] = JacobianTranspose[0][2]*JJTinv[0][0] + JacobianTranspose[1][2]*JJTinv[0][1];
        pseudoInv[1][2] = JacobianTranspose[0][2]*JJTinv[1][0] + JacobianTranspose[1][2]*JJTinv[1][1];

        //now we can use this to compute the change in angle based on the change in x and y which is just the speed
        //calculate the current position of the tip
        curr_x = top_link->getLength()*cos(upperQuat.angle()) + middle_link->getLength()*cos(upperQuat.angle() + middleQuat.angle()) + bottom_link->getLength()*cos(upperQuat.angle() + middleQuat.angle() + lowerQuat.angle());
        curr_y = top_link->getLength()*sin(upperQuat.angle()) + middle_link->getLength()*sin(upperQuat.angle() + middleQuat.angle()) + bottom_link->getLength()*sin(upperQuat.angle() + middleQuat.angle() + lowerQuat.angle());

        //now we take the difference between the final x and the curr x and the same for the y. if the difference is negative
        //then delta_x should be negative, otherwise it should be positive
        double delta_x = 0;
        double delta_y = 0;

        if((final_x - curr_x) < 0)
            delta_x = -sp;
        else
            delta_x = sp;

        if((final_y - curr_y) < 0)
            delta_y = -sp;
        else
            delta_y = sp;

        //now we need to multiply the vector containing delta_x and delta_y to have  the change in angle to apply to each
        double angle_changes[3];

        angle_changes[0] = pseudoInv[0][0]*delta_x + pseudoInv[1][0]*delta_y;
        angle_changes[1] = pseudoInv[0][1]*delta_x + pseudoInv[1][1]*delta_y;
        angle_changes[2] = pseudoInv[0][2]*delta_x + pseudoInv[1][2]*delta_y;

        upperQuat *= Quaternion(J, angle_changes[0]);
        top_link->setRot(upperQuat);
        middleQuat *= Quaternion(J, angle_changes[1]);
        middle_link->setRot(middleQuat);
        lowerQuat *= Quaternion(J, angle_changes[2]);
        bottom_link->setRot(lowerQuat);

        upperQuat.normalize();
        middleQuat.normalize();
        lowerQuat.normalize();

        glutPostRedisplay();
    }

}

void keyboard (unsigned char key, int x, int y)
{
    switch (key)
    {
        case 'f':
            glutFullScreen();
            break;
        case 27:
        case 'q':
            exit(0);
            break;
    }
}

void reshape (int w, int h)
{
    windowWidth = w;
    windowHeight = h;
    glViewport(0, 0, windowWidth, windowHeight);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40, double(w)/double(h), 1, 200);
    glutPostRedisplay();
}

int main(int argc, char *argv[])
{
    cout <<
         "f  full screen\n"
         "ESC  quit\n";
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(windowWidth, windowHeight);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("Forward Kinematics");
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutReshapeFunc(reshape);
    glutIdleFunc(idle);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_POSITION, dir);
    glMaterialfv(GL_FRONT, GL_SPECULAR, white);
    glMaterialfv(GL_FRONT, GL_SHININESS, shiny);
    build();
    glutMainLoop();
    return 0;
}