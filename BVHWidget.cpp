#define GLM_ENABLE_EXPERIMENTAL

#include <iostream>
#include <fstream>
#include <math.h>

#include <GL/gl.h>
#include <GL/glu.h>

#include <QApplication>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include "BVHWidget.h"



static float light_position[] = {0.0, 0.0, 1.0, 0.0};		

// Constructor
BVHWidget::BVHWidget(QWidget *parent) : 
    QGLWidget(parent),
    BVHfileName("./bvh/arm.bvh")
{

    // Bvh data
    bvhObj = new BVH(BVHfileName.c_str());
    
    // init Qt flags
    setMouseTracking(true);

    // init animation variables
    currentFrame = 0;
    playDir = 1; // dictates order we read frames (1 = forward, -1 = backward)

    // arbitrary size for a human figure
    dimension = 20;

    // init mouse interaction variables
    selectedJoint = -1;
    selectedConstraint = -1;
    constraints.resize(0);
    clickRadius = 0.5; // the radius of a joint sphere
    pressedButton = 0;

    dragging = false;
    shiftSel = false;

    // init the kinematics object
    iKinematics = new IKinematics(this);

    // place arcball
    Ball_Init(&objectBall);		
    Ball_Place(&objectBall, qOne, dimension);

}

// destructor
BVHWidget::~BVHWidget()
{
    delete bvhObj;
}


//
// OpenGL methods
//

void BVHWidget::initializeGL()
{
    // enable Z-buffering
    glEnable(GL_DEPTH_TEST);

    // set lighting parameters
    glShadeModel(GL_FLAT);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);

    // background is pink
    glClearColor(1.0, 0.7, 0.7, 1.0);
}

// called every time the widget is resized
void BVHWidget::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);

    // set projection matrix to be glOrtho based on zoom & window size
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    float size = 40;

    // stick with orthogonal projection
    float aspectRatio = (float) w / (float) h;

    if (aspectRatio > 1.0)
		glOrtho(-aspectRatio * dimension, aspectRatio * dimension, -dimension, dimension, 0, size);
	else
        glOrtho(-dimension, dimension, -dimension/aspectRatio, dimension/aspectRatio, 0, size);
	
    
}
	
// called every time the widget needs painting
void BVHWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_COLOR_MATERIAL);
    // set lighting on
    glEnable(GL_LIGHTING);

    // set model view matrix based on stored translation, rotation &c.
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // set light position first
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    
    glLineWidth(4);

    // draw axes
    glPushMatrix();
    glDisable(GL_LIGHTING);
    glTranslatef(-19.0, -19.0, 0.0);
    glBegin(GL_LINES);
    glColor4f(1.0, 0.0, 0.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(5.0, 0.0, 0.0);
    glColor4f(0.0, 1.0, 0.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 5.0, 0.0);
    //glEnable(GL_LIGHTING);
    glEnd();
    glPopMatrix();

    // put origin in front of camera which is at origin
    glTranslatef(0.0, 0.0, -dimension);

    /* UNCOMMENT TO USE ARCBALL */
    GLfloat mNow[16];
    Ball_Value(&objectBall, mNow);
	glMultMatrixf(mNow);

    // draw model axes
    glPushMatrix();
    glBegin(GL_LINES);
    glColor4f(1.0, 0.0, 0.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(2.0, 0.0, 0.0);
    glColor4f(0.0, 1.0, 0.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 2.0, 0.0);
    glColor4f(0.0, 0.0, 1.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 2.0);
    glEnable(GL_LIGHTING);
    glEnd();
    glPopMatrix();

    renderConstraints();

    // translate the figure to the origin 
    glTranslatef(-bvhObj->motion[ currentFrame * bvhObj->num_channel     ], 
                 -bvhObj->motion[ currentFrame * bvhObj->num_channel + 1 ], 
                 -bvhObj->motion[ currentFrame * bvhObj->num_channel + 2 ]);


    // render the figure at the current frame
    glColor3f(0.5,0.5,0.5);
    bvhObj->RenderFigure(currentFrame, 1);

    // render the joints as spheres
    renderJoints(bvhObj->GetJoint(0));

    //renderFreeJoints(bvhObj->GetJoint(0))
}

// Keep the width and height of the opengl widget constant for simplicity
QSize BVHWidget::minimumSizeHint() const
{
    return QSize(650, 650);
}

//
// Mouse methods
// 

// start processing mouse event
void BVHWidget::mousePressEvent(QMouseEvent *event)
{
    
    // get the mouse position
    HVect vNow = mouseToWorld(event->localPos().x(), event->localPos().y());
    // initialise the joint index
    selectedJoint = -1;
    selectedConstraint = -1;
    pressedButton = event->button();

    switch(pressedButton)
    {
        case(Qt::LeftButton):
            // check that shift button is pressed
            if (QApplication::keyboardModifiers() == Qt::ShiftModifier)
                shiftSel = true;
            else
                shiftSel = false;

            // check if a joint was found
            checkJointClick(vNow.x, vNow.y, bvhObj->GetJoint(0), getArcBallTransform());
            
            // set dragging to true if selected a joint
            if (selectedJoint != -1)
            {
                dragging = true;
                // shift clicking a joint, check in constraints, add to it if not
                if (shiftSel && !checkConstraint(selectedJoint))
                {
                    Constraint* newConstraint = new Constraint();
                    newConstraint->worldPos = iKinematics->getJointWorldPos(selectedJoint, false);
                    newConstraint->index = selectedJoint;
                    newConstraint->offset = glm::vec3(0.0);
                    constraints.push_back(newConstraint);
                    //selectedConstraint = constraints.size() - 1; 
                }
            }
            
            // checkConstraints, sets selectedConstraint if click found
            checkConstraintsClick(vNow.x, vNow.y, getArcBallTransform());

            if(selectedConstraint != -1)
                dragging = true;
            
            
            break;
        case(Qt::RightButton):
            Ball_Mouse(&objectBall, vNow);
			// start dragging
			Ball_BeginDrag(&objectBall);
            break;
        default:
            break;
    }
    // update the widget
    updateGL();
}

// process mouse movement (deform the grid according the direction we are travelling in)
void BVHWidget::mouseMoveEvent(QMouseEvent *event)
{
    // get mouse movement direction vector (in x y plane)
    HVect vNow = mouseToWorld(event->localPos().x(), event->localPos().y());

    // invert by the rotation in objectBall (the transpose since it is a rotation/orthogonal matrix)
    float rotatedX = objectBall.mNow[0][0]*(vNow.x - previousMousePos.x) + objectBall.mNow[0][1]*(vNow.y - previousMousePos.y) + objectBall.mNow[0][2]*(vNow.z - previousMousePos.z); 
    float rotatedY = objectBall.mNow[1][0]*(vNow.x - previousMousePos.x) + objectBall.mNow[1][1]*(vNow.y - previousMousePos.y) + objectBall.mNow[1][2]*(vNow.z - previousMousePos.z);
    float rotatedZ = objectBall.mNow[2][0]*(vNow.x - previousMousePos.x) + objectBall.mNow[2][1]*(vNow.y - previousMousePos.y) + objectBall.mNow[2][2]*(vNow.z - previousMousePos.z);
    
    glm::vec3 mouseMove(rotatedX, rotatedY, rotatedZ);
    //std::cout << mouseMove.x << " " << mouseMove.y << " " << mouseMove.z << std::endl; 

    switch(pressedButton)
    {
        case(Qt::RightButton):
            Ball_Mouse(&objectBall, vNow);
			Ball_Update(&objectBall);
            break;

        case(Qt::LeftButton):
            if(dragging)
            {
                //std::cout << "dragging" << std::endl;
                // shift select means we are updating the desired position of a single point without immediately using IK
                if(shiftSel)
                {
                    // move the selected constraint without changing the figure with IK
                    //std::cout << "move constraint at " << selectedConstraint << std::endl;
                    constraints[selectedConstraint]->offset += mouseMove;
                    //std::cout << glm::to_string(constraints[selectedConstraint]->offset) << std::endl;
                }
                // get the desired position of the selected joint
                else
                {
                    glm::vec3 goalPos = iKinematics->getJointWorldPos(selectedJoint, false) + mouseMove;
                    iKinematics->computeIK(goalPos, selectedJoint);
                }
                //glm::vec3 t = iKinematics->getJointWorldPos(selectedJoint, false);
                //std::cout << "joint : " << selectedJoint << " pos: " << t.x << " " << t.y << " " << t.z << std::endl;
                //std::cout << goalPos.x << " " << goalPos.y << " " << goalPos.z << std::endl;
                //std::cout << goalPos.x << " " << goalPos.y << " " << goalPos.z << std::endl;
                // test if the goal position is legal (within the maximum length available to the joints)
                // if (testLength(goalPos, selectedJoint))
                    // apply IK to get the new angles
            }
            break;

        default:
            break;
    }    

    updateGL();
    // update mouse position in world space
    previousMousePos = mouseToWorld( event->localPos().x(), event->localPos().y());
}

// stop processing the mouses movement
void BVHWidget::mouseReleaseEvent(QMouseEvent *event)
{
    switch(pressedButton)
    {
        case(Qt::LeftButton):
            dragging = false;
            updateGL();
            break;

        case(Qt::RightButton):
            Ball_EndDrag(&objectBall);
            updateGL();
            break;
        default:
            break;
    }
}

// recursively checks all the joints in the figure to detect whether it has been clicked on
void BVHWidget::checkJointClick(float mouseX, float mouseY, const BVH::Joint* joint, glm::mat4 transform)
{
    // translate by joint offset
    if (joint->parent != NULL)
        transform = glm::translate(transform, glm::vec3((float)joint->offset[ 0 ] , (float)joint->offset[ 1 ], (float)joint->offset[ 2 ]));
    
   
    // rotate by the joint's rotation
    for (int i=0; i<joint->channels.size(); i++ )
	{
		BVH::Channel *  channel = joint->channels[ i ];
		if ( channel->type == BVH::ChannelEnum::X_ROTATION )
			transform = glm::rotate(transform, (float)glm::radians(bvhObj->motion[ channel->index ]), glm::vec3(1.0f, 0.0f, 0.0f));
		else if ( channel->type == BVH::ChannelEnum::Y_ROTATION )
			transform = glm::rotate(transform,(float)glm::radians(bvhObj->motion[ channel->index ]), glm::vec3(0.0f, 1.0f, 0.0f ));
		else if ( channel->type == BVH::ChannelEnum::Z_ROTATION )
			transform = glm::rotate(transform, (float)glm::radians(bvhObj->motion[ channel->index ]), glm::vec3(0.0f, 0.0f, 1.0f ));
	}

    // apply the transformation to the root position to get the current joint's position with respect to the origin
    glm::vec4 jointPos = transform * glm::vec4(0.0, 0.0, 0.0, 1.0);
    
    //std::cout << "joint : " << joint->index << " pos: " << jointPos.x << " " << jointPos.y << " " << jointPos.z << std::endl;
    //glm::vec3 t = iKinematics->getJointWorldPos(joint->index, false);
    //std::cout << "joint : " << joint->index << " pos: " << t.x << " " << t.y << " " << t.z << std::endl;
    //std::cout << "mouse : " << mouseX << " " << mouseY << std::endl;

    // test we are in click radius
    if(glm::distance(glm::vec3(jointPos.x, jointPos.y , jointPos.z), glm::vec3(mouseX, mouseY, jointPos.z)) < clickRadius)
    {
        // no previously selected joints
        if (selectedJoint == -1) 
            selectedJoint = joint->index; 
        // compare z with previously selected joint
        else if (iKinematics->getJointWorldPos(selectedJoint, false).z < jointPos.z)
            selectedJoint = joint->index;

    }

    // recursion for child joints
    for(int index = 0; index < joint->children.size(); index++)
    {
        checkJointClick(mouseX, mouseY, joint->children[index], transform);
    }
}

void BVHWidget::checkConstraintsClick(float mouseX, float mouseY, glm::mat4 transform)
{
    // iterate over constraints
    
    //std::cout << "mouse: " << glm::to_string(glm::vec3(mouseX, mouseY, 0.0)) << std::endl;
    for(int i=0; i<constraints.size(); i++)
    {
        // difference between constraint pos and joint pos
        glm::vec3 diff(constraints[i]->worldPos - iKinematics->getJointWorldPos(constraints[i]->index, false));
        // transform position to be in same system as figure
        glm::vec4 constraintPos = transform * (iKinematics->getJointWorldTransform(constraints[i]->index) * glm::vec4(0.0, 0.0, 0.0, 1.0) + glm::vec4(constraints[i]->offset, 0.0));
        //std::cout << "constraint pos:" << glm::to_string(constraintPos) << std::endl;
        
        if(glm::distance(glm::vec3(constraintPos.x, constraintPos.y , constraintPos.z), glm::vec3(mouseX, mouseY, constraintPos.z)) < clickRadius)
        {
            //std::cout << "found constraint" << std::endl;
            selectedConstraint = i;
        }
    }
    //std::cout << selectedConstraint << std::endl;
}

bool BVHWidget::checkConstraint(int index)
{
    // iterate over constraints set, checking whether joint is a constraint
    for(int i=0; i<constraints.size(); i++)
    {
        if (constraints[i]->index == index)
            return true;    
    }
    //std::cout << "not in constraints" << std::endl;
    return false;
}

// return the mouse click in world x and y, accomodating for aspect ratio
HVect BVHWidget::mouseToWorld(float mouseX, float mouseY)
{
    HVect worldMouse;
    // transorm back into world coordinates, qt mouse event has origin in top left of widget vs bottom left for OpenGL
    worldMouse.x =   (2.0 * mouseX / width() -1) * dimension; // multiply by size?;
    worldMouse.y = (-2.0 * mouseY / height() +1) * dimension; // multiply by size?;
    worldMouse.z = 0;
    // compute aspect ratio
    float aspectRatio = (float) width() / (float) height();
    // accomodate for aspect ratio
    if (aspectRatio > 1.0)
		worldMouse.x *= aspectRatio;
	else
        worldMouse.y *= aspectRatio;
    return worldMouse;
}

// 
bool BVHWidget::testLength(glm::vec3 t, int index)
{
    // start at root of current frame
    glm::vec3 max(bvhObj->motion[ currentFrame*bvhObj->num_channel ], bvhObj->motion[ currentFrame*bvhObj->num_channel + 1 ], bvhObj->motion[ currentFrame*bvhObj->num_channel + 2 ]);
    // translate t by root to move to (0,0,0)
    t -= max;
    BVH::Joint* joint = bvhObj->joints[ index ];

    while(joint->parent != NULL)
    {
        max += glm::vec3( joint->offset[ 0 ], joint->offset[ 1 ], joint->offset[ 2 ]);
        joint = joint->parent;
    }
    // std::cout << max.x << "  " << max.y << "  " << max.z << std::endl;

    float tLength = t.x * t.x + t.y * t.y + t.z * t.z;
    float mLength = max.x * max.x + max.y * max.y + max.z * max.z;
    
    //std::cout << tLength << std::endl;
    //std::cout << mLength << std::endl;

    if (tLength <= 2 * mLength)
        return true;
    else
        return false;
}

//
// Figure methods
//

// render the joints of a figure at the current time frame
void BVHWidget::renderJoints(const BVH::Joint* joint)
{
    glPushMatrix();

    // translate by joint offset
    if (joint->parent == NULL)
        glTranslatef(bvhObj->motion[ currentFrame * bvhObj->num_channel ], 
                    bvhObj->motion[ currentFrame * bvhObj->num_channel + 1 ], 
                    bvhObj->motion[ currentFrame * bvhObj->num_channel + 2 ]);
    else
        glTranslatef(joint->offset[ 0 ] , joint->offset[ 1 ], joint->offset[ 2 ] );
   
    // rotate by the joint's rotation
    for (int i=0; i<joint->channels.size(); i++ )
	{
		BVH::Channel *  channel = joint->channels[ i ];
		if ( channel->type == BVH::ChannelEnum::X_ROTATION )
			glRotatef(bvhObj->motion[ currentFrame * bvhObj->num_channel + channel->index ], 1.0f, 0.0f, 0.0f);
		else if ( channel->type == BVH::ChannelEnum::Y_ROTATION )
			glRotatef(bvhObj->motion[ currentFrame * bvhObj->num_channel + channel->index ], 0.0f, 1.0f, 0.0f );
		else if ( channel->type == BVH::ChannelEnum::Z_ROTATION )
			glRotatef(bvhObj->motion[ currentFrame * bvhObj->num_channel + channel->index ], 0.0f, 0.0f, 1.0f );
	}

    // render the joint with as a sphere with glu 
    static GLUquadricObj *  quad_obj = NULL;
	if ( quad_obj == NULL )
		quad_obj = gluNewQuadric();
	gluQuadricDrawStyle( quad_obj, GLU_FILL );
	gluQuadricNormals( quad_obj, GLU_SMOOTH );

    // switch colour for selected joint
    if (joint->index == selectedJoint)
        glColor3f(1.0,0.0,0.0);
    else
        glColor3f(0.0,0.0,1.0);
    
    gluSphere(quad_obj, 0.5, 10, 10);


    // recursion for child joints
    for(int child = 0; child < joint->children.size(); child++)
    {
        renderJoints(bvhObj->GetJoint(joint->children[ child ]->index));
    }

    glPopMatrix();
}

// overwrties motion data with new frame data (angles from IK)
void BVHWidget::saveNewFrame(int frameIndex)
{
    // loop over channel angles and set current framee's data 
    for( int i=3; i<bvhObj->num_channel; i++)
    {
        float tAngle = (int)iKinematics->tAngles[i-3] % 360 + iKinematics->tAngles[i-3] - (int)iKinematics->tAngles[i-3];
        bvhObj->motion[ frameIndex*bvhObj->num_channel + i] = tAngle;
    }
}


void BVHWidget::renderConstraints()
{
    glPushMatrix();
    static GLUquadricObj *  quad_obj = NULL;
	if ( quad_obj == NULL )
		quad_obj = gluNewQuadric();
	gluQuadricDrawStyle( quad_obj, GLU_FILL );
	gluQuadricNormals( quad_obj, GLU_SMOOTH );

    // constraints are in green
    glColor3f(0.0,1.0,0.0);

    glm::vec4 root(0.0, 0.0, 0.0, 1.0);

    // loop over constraints and render
    for(int i=0; i<constraints.size(); i++)
    {
        //std::cout << constraints[i]->index << std::endl;
        //std::cout << glm::to_string(constraints[i]->offset) << std::endl;
        glm::vec4 pos = iKinematics->getJointWorldTransform(constraints[i]->index) * root + glm::vec4(constraints[i]->offset, 0.0);
        //std::cout << "world pos: " << glm::to_string(iKinematics->getJointWorldPos(constraints[i]->index, false)) <<std::endl;
        // std::cout << "calculated pos: " << glm::to_string(pos) << std::endl;
        glPushMatrix();
        glTranslatef(pos.x, pos.y, pos.z);
        gluSphere(quad_obj, 0.6, 10, 10);
        glPopMatrix();
    }
    glPopMatrix();
}
//
// Slot methods
//

void BVHWidget::loadBVHFile(QString fileName)
{
    bvhObj->Load(fileName.toLocal8Bit().data());
    
    // emit a signal containing the number of frames in the BVH to main window
    emit setTimerInterval(bvhObj->interval * 1000); // send interval value in ms
    emit setMaxFrame(bvhObj->num_frame); // send max frames to slider

    // also reset frame data
    currentFrame = 0;
    // and figure data
    constraints.resize(0);

}

// slot method that is called when we are incrementing the frame in the animation
void BVHWidget::animateFigure(int frame)
{
    // set current frame if within the range (should always be)
    if (bvhObj->num_frame > frame)
        currentFrame = frame;
    // then update the widget 
    updateGL();
}

void BVHWidget::changePlayDir()
{
    
}

// returns the rotation in the arcball as a glm mat4
glm::mat4 BVHWidget::getArcBallTransform()
{
    glm::mat4 transform(1.0);
    float mNow[16];
    Ball_Value(&objectBall, mNow);
    for(int col = 0; col < 4; col++)
        for(int row = 0; row < 3; row++)
            transform[col][row] = mNow[col * 4 + row];
    /*
    std::cout << transform[0][0] << " " << transform[1][0] << " "<< transform[2][0] << " "<< transform[3][0] << std::endl;
    std::cout << transform[0][1] << " " << transform[1][1] << " "<< transform[2][1] << " "<< transform[3][1] << std::endl;
    std::cout << transform[0][2] << " " << transform[1][2] << " "<< transform[2][2] << " "<< transform[3][2] << std::endl;
    std::cout << transform[0][3] << " " << transform[1][3] << " "<< transform[2][3] << " "<< transform[3][3] << std::endl;
    */
    return transform;
}