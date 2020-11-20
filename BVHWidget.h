#ifndef _BVH_WIDGET_H
#define _BVH_WIDGET_H

#include <QGLWidget>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QGridLayout>
#include <QSize>
#include <QString>

#include <vector>
#include <ctime>

#include<glm/glm.hpp>

// class for reading and rendering BVH file
#include "BVH.h"
// class for calculating inverse kinematics
#include "IKinematics.h"

// utility class defining vector object (3 floats) and operations 
#include "Vector.h"
#include "Ball.h"


class IKinematics;

class BVHWidget : public QGLWidget
{
    Q_OBJECT
    public:

    // a struct containing the nescessary data for inverse kinematics
    struct SelectJoint
    {
        // goal position in world space
        glm::vec3 goalPos;
        // index of the joint it refers to
        int index;
    };

    // constructor
    BVHWidget(QWidget *parent);
    // destructor
    ~BVHWidget();


    // slots
    public slots:
    // file i/o handling
    void loadBVHFile(QString fileName);
    void saveBVHFile(QString filename);
    // called whenever the slider chages values
    void animateFigure(int frame);
    // update the playDir variable
    void changePlayDirPlus();
    void changePlayDirMinus();
    // writes ik motion to bvh data
    void saveFrame();
    // update dampening variables
    void setDampening(int state);
    void updateLambda(int newLambda);
    // reset the arcball for convenience
    void resetRotation();


    signals:
    void setTimerInterval(int interval);
    void setMaxFrame(int frameMax);


    protected:
    // Qt opengl functions
	// called when OpenGL context is set up
	void initializeGL();
	// called every time the widget is resized
	void resizeGL(int w, int h);
	// called every time the widget needs painting
	void paintGL();


    // mouse input
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    // convert screen coordinates to world coordinates
    HVect mouseToWorld(float mouseX, float mouseY);
    // check that click is on a joint
    void checkJointClick(float mouseX, float mouseY, const BVH::Joint* joint, glm::mat4 transform);
    // checks if a joint is selected
    bool checkSelected(int index);


    // figure data and rendering
    // render joints as spheres
    void renderJoints(const BVH::Joint* joint, double* motion);
    // recursively write joint data to file out stream
    void writeJoint(std::ofstream& ostream, BVH::Joint* joint, int depth);


    // returns arc ball transform, useful for mouse picking
    glm::mat4 getArcBallTransform();

    public:
    // flag representing drag status
    bool dragging;
    // flag representing rotation status
    bool rotating;
    // flag representing shift click select
    bool shiftSel;
    // modified frame data
    bool modified;
    // (de)activate dampening
    bool dampening;


    // mouse Input
    HVect previousMousePos;
    glm::vec3 currentPos;


    // arc ball data
    BallData objectBall;
    GLfloat translate_x, translate_y;
	GLfloat last_x, last_y;


    // Bvh data
    BVH *bvhObj;
    double *currentMotion;
    std::string BVHfileName;


    // inverse kinematics 
    IKinematics* iKinematics;
    int lambda;


    // animation variables
    int currentFrame;
    int playDir;


    // assumed size of a model for projection
    int dimension;


    // mouse picking varianbles
    float clickRadius;
    int selectedJoint;
    int pressedButton;
    std::vector<SelectJoint*> selectedJoints;


    // widget size
    QSize minimumSizeHint() const;
};

#endif