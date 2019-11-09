// Inverse kinematics using pseudo-inverse of Jacobian matrix

// Link with libcugl libglut32 libopengl32 libglu32

#include <iostream>
#include <vector>
#include "include/cugl.h"

using namespace std;
using namespace cugl;

// Current window dimensions
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
    Link(double length, GLfloat *col) : length(length), radius(length/20), col(col)
    {
        bar = gluNewQuadric();
        gluQuadricDrawStyle(bar, GLU_FILL);
        gluQuadricOrientation(bar, GLU_OUTSIDE);
        gluQuadricNormals(bar, GLU_SMOOTH);
    }

    void draw()
    {
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, col);
        glRotated(degrees(angle), 1, 0, 0);
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

    void setRot(double newAngle)
    {
        angle = newAngle;
    }

private:
    double length;
    double radius;
    GLfloat *col;
    double angle;
    vector<Link*> pLinks;
    GLUquadricObj *bar;
};

// The arm has three components
Link* link_1;
const double link_1_length = 25;
double a_1 = -PI/4;

Link* link_2;
const double link_2_length = 20;
double a_2 = -PI/4;

Link* link_3;
const double link_3_length = 15;
double a_3 = -PI/4;

Link* link_4;
const double link_4_length = 10;
double a_4 = -PI/4;

// Target point - moved when the arm has reached it
double tX = link_3_length;
double tY = link_4_length;

// Current position of tip
double x;
double y;

// Control step size
double step = 0.01;

// Choose a random target for the tip to aim at
void chooseTarget()
{
    double rad = 0, ang = 2 * PI * randReal();
    rad = 2 + (link_1_length + link_2_length + link_3_length + link_4_length - 3) * randReal();
    tX = rad * cos(ang);
    tY = rad * sin(ang);
}

// Initialize arm and target
void initialize()
{
    link_1 = new Link(link_1_length, red);
    link_2 = new Link(link_2_length, green);
    link_3 = new Link(link_3_length, blue);
    link_4 = new Link(link_4_length, white);
    link_1->addLink(link_2);
    link_2->addLink(link_3);
    link_3->addLink(link_4);
    chooseTarget();
}

void display (void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslated(0, 0, -200);
    glRotated(90, 0, 1, 0);

    // Show target
    glPushMatrix();
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, blue);
    glTranslated(0, -tY, tX);
    glutSolidSphere(1, 20, 20);
    glPopMatrix();

    // Draw robot arm
    glutSolidSphere(3, 20, 20);
    link_1->draw();

    glutSwapBuffers();
}

void idle()
{
    static double oldDA1 = 0;
    static double oldDA2 = 0;
    static double oldDA3 = 0;
    static double oldDA4 = 0;

    // Current position of tip
    x = link_1_length * cos(a_1) + link_2_length * cos(a_1 + a_2) + link_3_length * cos(a_1 + a_2 + a_3)
            + link_4_length * cos(a_1 + a_2 + a_3 + a_4);
    y = link_1_length * sin(a_1) + link_2_length * sin(a_1 + a_2) + link_3_length * sin(a_1 + a_2 + a_3)
        + link_4_length * sin(a_1 + a_2 + a_3 + a_4);

    // Position of tip relative to target
    double deltaX = tX - x;
    double deltaY = tY - y;

    // If tip is close to target, move the tartget
    double dist = sqrt(deltaX * deltaX + deltaY * deltaY);
    if (dist < 0.1)
    {
        chooseTarget();
        return;
    }

    // Scale deltas according to distance
    double rd = step / dist;
    deltaX *= rd;
    deltaY *= rd;

    // Find partial derivatives
    double dx_da4 = - link_4_length * sin(a_1 + a_2 + a_3 + a_4);
    double dx_da3 = - link_3_length * sin(a_1 + a_2 + a_3) + dx_da4;
    double dx_da2 = - link_2_length * sin(a_1 + a_2) + dx_da3;
    double dx_da1 = - link_1_length * sin(a_1) + dx_da2;

    double dy_da4 = link_4_length * cos(a_1 + a_2 + a_3 + a_4);
    double dy_da3 = link_3_length * cos(a_1 + a_2 + a_3) + dx_da4;
    double dy_da2 = link_2_length * cos(a_1 + a_2) + dx_da3;
    double dy_da1 = link_1_length * cos(a_1) + dx_da2;

    // Set up Jacobian J
    double j[2][4];
    j[0][0] = dx_da1;
    j[0][1] = dx_da2;
    j[0][2] = dx_da3;
    j[0][3] = dx_da4;
    j[1][0] = dy_da1;
    j[1][1] = dy_da2;
    j[1][2] = dy_da3;
    j[1][3] = dy_da4;

    // Compute transposed Jacobian J^T
    double jt[4][2];
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 2; ++c)
            jt[r][c] = j[c][r];

    // Compute product J J^T
    double jjt[2][2];
    for (int r = 0; r < 2; ++r)
        for (int c = 0; c < 2; ++c)
        {
            jjt[r][c] = 0;
            for (int k = 0; k < 4; ++k)
                jjt[r][c] += j[r][k] * jt[k][c];
        }

    // Set angle increments with old values in case there is a singularity
    double deltaA1 = oldDA1;
    double deltaA2 = oldDA2;
    double deltaA3 = oldDA3;
    double deltaA4 = oldDA4;

    // Compute determinant
    double det = jjt[0][0] * jjt[1][1] - jjt[0][1] * jjt[1][0];
    if (det == 0)
    {
        cerr << "Singularity!\n";
    }
    else
    {
        // Determinant is non-zero, so compute inverse
        double jjti[2][2];
        jjti[0][0] =   jjt[1][1] / det;
        jjti[0][1] = - jjt[0][1] / det;
        jjti[1][0] = - jjt[1][0] / det;
        jjti[1][1] =   jjt[0][0] / det;

        // Pseudoinverse = transpose * inverse
        double psi[4][2];
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 2; ++c)
            {
                psi[r][c] = 0;
                for (int k = 0; k < 2; ++k)
                    psi[r][c] += jt[r][k] * jjti[k][c];
            }

        // Obtain angle increments from pseudo-inverse
        deltaA1 = psi[0][0] * deltaX + psi[0][1] * deltaY;
        deltaA2 =   psi[1][0] * deltaX + psi[1][1] * deltaY;
        deltaA3 =   psi[2][0] * deltaX + psi[2][1] * deltaY;
        deltaA4 =   psi[3][0] * deltaX + psi[3][1] * deltaY;

        // Save increments in case we need them
        oldDA1 = deltaA1;
        oldDA2 = deltaA2;
        oldDA3 = deltaA3;
        oldDA4 = deltaA4;
    }

    // Update angles and arm positions
    a_1 += deltaA1;
    link_1->setRot(a_1);

    a_2 += deltaA2;
    link_2->setRot(a_2);

    a_3 += deltaA3;
    link_3->setRot(a_3);

    a_4 += deltaA4;
    link_4->setRot(a_4);

    glutPostRedisplay();
}

void keyboard (unsigned char key, int x, int y)
{
    switch (key)
    {
        case 27:
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
    cout << "COMP 376 Assignment 2 Problem 2 \n" << "ESC Quit";
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(windowWidth, windowHeight);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("COMP 376 Assignment 2 Problem 2");
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
    initialize();
    glutMainLoop();
    return 0;
}
