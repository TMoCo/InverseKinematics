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

    // a positional constraint for inverse kinematics
    struct Constraint
    {
        // position in world space
        glm::vec3 worldPos;
        // vector to pos in world space from joint pos
        glm::vec3 offset;
        // index of the joint it refers to
        int index;
    };

    // constructor
    BVHWidget(QWidget *parent);
    ~BVHWidget();


    // slots
    public slots:
    void loadBVHFile(QString fileName);
    // called whenever the slider chages values
    void animateFigure(int frame);
    void changePlayDir();


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
    void checkConstraintsClick(float mouseX, float mouseY, glm::mat4 transform);
    bool checkConstraint(int index);


    // figure data
    glm::vec3 getJointWorldPos(int index);
    bool testLength(glm::vec3 t, int index);
    void renderJoints(const BVH::Joint* joint);
    void renderConstraints();
    void saveNewFrame(int frameIndex);


    // method for returning arc ball transform, useful for mouse picking
    glm::mat4 getArcBallTransform();


    public:
    // flag representing drag status
    bool dragging;
    // flag representing rotation status
    bool rotating;
    // flag representing shift click select
    bool shiftSel;


    // mouse Input
    HVect previousMousePos;
    glm::vec3 currentPos;


    // arc ball data
    BallData objectBall;
    GLfloat translate_x, translate_y;
	GLfloat last_x, last_y;


    // Bvh data
    BVH *bvhObj;
    std::string BVHfileName;


    // inverse kinematics 
    IKinematics* iKinematics;


    // animation variables
    int currentFrame;
    int playDir;


    // assumed size of a model for projection
    int dimension;


    // mouse picking varianbles
    float clickRadius;
    int selectedJoint;
    int selectedConstraint;
    int pressedButton;
    std::vector<Constraint*> constraints;


    // widget size
    QSize minimumSizeHint() const;
};

#endif