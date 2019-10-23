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

void build()
{
    top_link = new Link(10, 1.5, red);
    middle_link = new Link(7.5, 0.75, green);
    top_link->addLink(middle_link);
    bottom_link = new Link(5, 0.25, blue);
    middle_link->addLink(bottom_link);
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
double sp = 0.001;

void idle()
{

    upperQuat *= Quaternion(J, sp);
    top_link->setRot(upperQuat);
    middleQuat *= Quaternion(Vector(0, 0, 0), 3 * sp);
    middle_link->setRot(middleQuat);
    lowerQuat *= Quaternion(Vector(0, 0, 0), 3*sp);
    bottom_link->setRot(lowerQuat);


    upperQuat.normalize();
    middleQuat.normalize();
    lowerQuat.normalize();

    glutPostRedisplay();
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