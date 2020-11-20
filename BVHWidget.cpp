#define GLM_ENABLE_EXPERIMENTAL

#include <iostream>
#include <iomanip>
#include <fstream>
#include <math.h>

#include <GL/gl.h>
#include <GL/glu.h>

#include <QApplication>
#include <QMessageBox>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include "BVHWidget.h"


// directional light along z axis
static float light_position[] = {0.0, 0.0, 1.0, 0.0};		

// Constructor
BVHWidget::BVHWidget(QWidget *parent) : 
    QGLWidget(parent),
    BVHfileName("./bvh/arm.bvh")
{
    // Bvh data
    bvhObj = new BVH(BVHfileName.c_str());

    // handle no bvh file
    if (!bvhObj->is_load_success)
        throw std::runtime_error("Could not find bvh file in bvh folder");
    
    // init Qt flags
    setMouseTracking(true);

    // init animation variables
    currentFrame = 0;
    playDir = 1; // dictates order we read frames (1 = forward, -1 = backward)
    // array for current frame data initialised to first frame
    currentMotion = new double [bvhObj->num_frame*bvhObj->num_channel];
    // copy first frame data into currentMotion
    for (int i=0; i<bvhObj->num_channel; i++)
        currentMotion[i] = bvhObj->motion[i];

    // arbitrary size for a human figure
    dimension = 20;

    // init mouse interaction variables
    selectedJoint = -1;
    selectedJoints.resize(0);
    clickRadius = 0.5; // the radius of a joint sphere
    pressedButton = 0;

    // boolean bvh state flags
    dragging = false;
    shiftSel = false;
    modified = false;
    dampening = false;

    // init the inverse kinematics object and variables
    iKinematics = new IKinematics(this);
    lambda = 1;

    // place arcball
    Ball_Init(&objectBall);		
    Ball_Place(&objectBall, qOne, dimension);

}

// destructor
BVHWidget::~BVHWidget()
{
    delete bvhObj;
    delete currentMotion;
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
    // set viewport to width and height
    glViewport(0, 0, w, h);

    // set projection matrix to be glOrtho based on zoom & window size
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // stick with orthogonal projection
    float aspectRatio = (float) w / (float) h;

    // project according to aspect ratio
    if (aspectRatio > 1.0)
		glOrtho(-aspectRatio * dimension, aspectRatio * dimension, -dimension, dimension, 0, 2 * dimension);
	else
        glOrtho(-dimension, dimension, -dimension/aspectRatio, dimension/aspectRatio, 0, 2 * dimension);
	
    
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
    
    // set line width
    glLineWidth(4);

    // draw x y axes in bottom left
    glPushMatrix();
    glDisable(GL_LIGHTING);
    glTranslatef(-dimension + 0.5, -dimension + 0.5 , 0.0);
    glBegin(GL_LINES);
    glColor4f(1.0, 0.0, 0.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(5.0, 0.0, 0.0);
    glColor4f(0.0, 1.0, 0.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 5.0, 0.0);
    glEnd();
    glPopMatrix();

    // put origin in front of camera which itself is at origin
    glTranslatef(0.0, 0.0, -dimension);

    // multiply by arcball rotation
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

    //renderConstraints();

    // translate the figure to the origin 
    glTranslatef(-currentMotion[0], -currentMotion[1], -currentMotion[2]);

    glColor3f(0.5,0.5,0.5);
    // render the figure at the current frame
    bvhObj->RenderFigure(currentFrame, 1);

    // render the joints as spheres 
    renderJoints(bvhObj->GetJoint(0), currentMotion);
}

// set minimum size to a 650x650 square
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
    // store mouse button for mouse move events
    pressedButton = event->button();
    switch (pressedButton)
    {
        case (Qt::LeftButton):
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
                // create new selected joint object
                SelectJoint* n_joint = new SelectJoint();
                n_joint->index = selectedJoint;
                // shift clicking a joint means we want multiple, check in constraints, add to it if not
                if (shiftSel)
                {
                    if (!checkSelected(selectedJoint))
                        selectedJoints.push_back(n_joint);
                    else
                    {
                        // remove from selected joints
                        std::vector<SelectJoint*>::iterator it = selectedJoints.begin();
                        while (it != selectedJoints.end())
                        {
                            if ((*it)->index == selectedJoint)
                                selectedJoints.erase(it);
                            std::advance(it, 1);
                        }
                    }                
                }
                else
                    selectedJoints.push_back(n_joint);
            }
            break;
        case (Qt::RightButton):
            Ball_Mouse(&objectBall, vNow);
			// start dragging
			Ball_BeginDrag(&objectBall);
            updateGL();
            break;
        default:
            break;
    }
    // update the widget
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
    // store mouse movement vector
    glm::vec3 mouseMove(rotatedX, rotatedY, rotatedZ);
    switch (pressedButton)
    {
        case (Qt::RightButton):
            Ball_Mouse(&objectBall, vNow);
			Ball_Update(&objectBall);
            updateGL();
            break;

        case (Qt::LeftButton):
            if (dragging)
            {
                // shift select means we are updating the desired position of a single point without immediately using IK
                if (shiftSel)
                {
                    // move the selected joints by mouse move without changing the figure with IK
                    for (int i=0; i<selectedJoints.size(); i++)
                        selectedJoints[i]->goalPos = iKinematics->getJointWorldPos(selectedJoints[i]->index) + mouseMove;
                }
                // get the desired position of the selected joint
                else
                {
                    selectedJoints[0]->goalPos = iKinematics->getJointWorldPos(selectedJoints[0]->index) + mouseMove;
                }
                // now compute IK
                iKinematics->computeIK();
                updateGL();
            }
            break;

        default:
            break;
    }    
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
            // no longer dragging let go of selected joints
            selectedJoints.resize(0);
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
        transform = glm::translate(transform, glm::vec3((float)joint->offset[0] , (float)joint->offset[1], (float)joint->offset[2]));   
    // rotate by the joint's rotation
    for (int i=0; i<joint->channels.size(); i++ )
	{
		BVH::Channel* channel = joint->channels[i];
		if (channel->type == BVH::ChannelEnum::X_ROTATION)
			transform = glm::rotate(transform, (float)glm::radians(currentMotion[channel->index]), glm::vec3(1.0f, 0.0f, 0.0f));
		else if (channel->type == BVH::ChannelEnum::Y_ROTATION)
			transform = glm::rotate(transform,(float)glm::radians(currentMotion[channel->index]), glm::vec3(0.0f, 1.0f, 0.0f));
		else if (channel->type == BVH::ChannelEnum::Z_ROTATION)
			transform = glm::rotate(transform, (float)glm::radians(currentMotion[channel->index]), glm::vec3(0.0f, 0.0f, 1.0f));
	}
    // apply the transformation to the root position to get the current joint's position with respect to the origin
    glm::vec4 jointPos = transform * glm::vec4(0.0, 0.0, 0.0, 1.0);
    // test we are in click radius
    if (glm::distance(glm::vec3(jointPos.x, jointPos.y , jointPos.z), glm::vec3(mouseX, mouseY, jointPos.z)) < clickRadius)
    {
        // no previously selected joints
        if (selectedJoint == -1) 
            selectedJoint = joint->index; 
        // compare z with previously selected joint
        else if (iKinematics->getJointWorldPos(selectedJoint).z < jointPos.z)
            selectedJoint = joint->index;
    }
    // recursion for child joints
    for(int i=0; i<joint->children.size(); i++)
        checkJointClick(mouseX, mouseY, joint->children[i], transform);
}

// checks whether a given joint index is currently selected  
bool BVHWidget::checkSelected(int index)
{
    // iterate over constraints set, checking whether joint is a constraint
    for(int i=0; i<selectedJoints.size(); i++)
        if (selectedJoints[i]->index == index)
            return true;    
    return false;
}

// return the mouse click in world x and y, accomodating for aspect ratio
HVect BVHWidget::mouseToWorld(float mouseX, float mouseY)
{
    HVect worldMouse;
    // transorm back into world coordinates, qt mouse event has origin in top left of widget vs bottom left for OpenGL
    worldMouse.x = (2.0 * mouseX / width() - 1) * dimension; // multiply by size?;
    worldMouse.y = (-2.0 * mouseY / height() + 1) * dimension; // multiply by size?;
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
// Figure methods
//

// render the joints of a figure at the current time frame
void BVHWidget::renderJoints(const BVH::Joint* joint, double* motion)
{
    glPushMatrix();
    // translate by joint offset
    if (joint->parent == NULL)
        glTranslatef(motion[0], motion[1], motion[2]);
    else
        glTranslatef(joint->offset[0] , joint->offset[1], joint->offset[2]);
    // rotate by the joint's rotation
    for (int i=0; i<joint->channels.size(); i++)
	{
		BVH::Channel* channel = joint->channels[i];
		if (channel->type == BVH::ChannelEnum::X_ROTATION)
			glRotatef(motion[channel->index], 1.0f, 0.0f, 0.0f);
		else if (channel->type == BVH::ChannelEnum::Y_ROTATION)
			glRotatef(motion[channel->index], 0.0f, 1.0f, 0.0f);
		else if (channel->type == BVH::ChannelEnum::Z_ROTATION)
			glRotatef(motion[channel->index], 0.0f, 0.0f, 1.0f);
	}
    // render the joint with as a sphere with glu 
    static GLUquadricObj* quad_obj = NULL;
	if (quad_obj == NULL)
		quad_obj = gluNewQuadric();
	gluQuadricDrawStyle(quad_obj, GLU_FILL);
	gluQuadricNormals(quad_obj,GLU_SMOOTH);
    // switch colour for selected joint
    if (joint->index == selectedJoint)
        glColor3f(1.0,0.0,0.0);
    else
        glColor3f(0.0,0.0,1.0);
    // place sphere at joint position of r = 0.5, same radius as mouse click radius
    gluSphere(quad_obj, 0.5, 10, 10);
    // recursion for child joints
    for(int child = 0; child < joint->children.size(); child++)
        renderJoints(bvhObj->GetJoint(joint->children[child]->index), motion);
    glPopMatrix();
}

// overwrties motion data with new frame data (angles from IK)
void BVHWidget::saveFrame()
{
    // loop over channel angles and set current frame's data 
    for( int i=0; i<bvhObj->num_channel; i++)
        bvhObj->motion[ currentFrame*bvhObj->num_channel + i] = currentMotion[i];
    updateGL();
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
    // and array for new data
    delete currentMotion;
    // and figure data
    selectedJoints.resize(0);
    // allocate array of new motion angles
    currentMotion = new double [bvhObj->num_frame*bvhObj->num_channel];
    for (int i=0; i<bvhObj->num_channel; i++)
        currentMotion[i] = bvhObj->motion[i];
    updateGL();

}

void BVHWidget::saveBVHFile(QString fileName)
{
    try 
    {
        // open new bvh
        std::ofstream bvhFile;
        // clear bvh file
        bvhFile.open(fileName.toStdString(), std::ofstream::trunc);
        // set decimal precision for values
        bvhFile << std::fixed << std::setprecision(6) ;
        // start by inputing Header data
        bvhFile << "HIERARCHY\n";
        // recusrively write joint data to file
        writeJoint(bvhFile, bvhObj->joints[0], 0);
        // next write motion data
        bvhFile << "MOTION\n";
        bvhFile << "Frames: " << bvhObj->num_frame << '\n';
        bvhFile << "Frame Time: " << bvhObj->interval << '\n';
        // iterate over each frame and write channel data on a single line, separated by space
        for(int i=0; i<bvhObj->num_frame; i++)
        {
            for(int j=0; j<bvhObj->num_channel; j++)
                bvhFile << bvhObj->motion[ i*bvhObj->num_channel + j] << ' ';
            bvhFile << '\n';
        }
    }
    catch(const std::exception& e)
    {
        QMessageBox errorMsg;
        errorMsg.setText("Could not open file.");
        errorMsg.exec();
    }
}

// function for recusrsively writing a joint's data to a BVH file with formatting
void BVHWidget::writeJoint(std::ofstream &ostream, BVH::Joint* joint, int depth)
{
    int i;
    // add indentation
    for(i=0; i<depth; i++)
        ostream << '\t';
    // JOINT NAME ----
    // the joint identifier
    if (joint->parent == NULL)
        ostream << "ROOT ";
    else
        ostream << "JOINT ";
    // the joint name
    ostream << joint->name << '\n';
    // open curly bracket
    for(i=0; i<depth; i++)
        ostream << '\t';
    ostream << "{\n";
    // increment the indentation depth
    depth += 1;
    // JOINT OFFSET ----
    // set the indentation
    for(i=0; i<depth; i++)
        ostream << '\t';
    // the offset values
    ostream << "OFFSET ";
    for (i=0; i<3; i++)
        ostream << joint->offset[i] << ' ';
    ostream << '\n';
    // JOINT CHANNELS ----
    // set the indentation
    for(i=0; i<depth; i++)
        ostream << '\t';
    // the channels
    ostream << "CHANNELS " << joint->channels.size() << ' ';
    // loop over each channel
    for(i=0; i<joint->channels.size(); i++)
    {
        if (joint->channels[i]->type == BVH::X_POSITION)
            ostream << "Xposition ";
        if (joint->channels[i]->type == BVH::Y_POSITION)
            ostream << "Yposition ";
        if (joint->channels[i]->type == BVH::Z_POSITION)
            ostream << "Zposition ";
        if (joint->channels[i]->type == BVH::X_ROTATION)
            ostream << "Xrotation ";
        if (joint->channels[i]->type == BVH::Y_ROTATION)
            ostream << "Yrotation ";
        if (joint->channels[i]->type == BVH::Z_ROTATION)
            ostream << "Zrotation ";
    }
    ostream << '\n';
    // END SITE ----
    if (joint->has_site)
    {
        for(i=0; i<depth; i++)
            ostream << '\t';
        ostream << "End Site\n";
        for(i=0; i<depth; i++)
            ostream << '\t';
        ostream << "{\n";
        for(i=0; i<depth+1; i++)
            ostream << '\t';
        ostream << "OFFSET ";
        for (i=0; i<3; i++)
            ostream << joint->site[i] << ' ';
        ostream << '\n';
        for (i=0; i<depth; i++)
            ostream << '\t';
        ostream << "}\n";
    }
    // recursion depth first for joint children
    for(i=0; i<joint->children.size(); i++)
        writeJoint(ostream, joint->children[i], depth);
    // decrement indent depth
    depth -= 1;
    // close curly bracket
    for (i=0; i<depth; i++)
        ostream << '\t';
    ostream << "}\n";
}

// slot method that is called when we are incrementing the frame in the animation
void BVHWidget::animateFigure(int frame)
{
    // set current frame if within the range (should always be)
    if (bvhObj->num_frame > frame)
        currentFrame = frame;
    // copy frame data into currentMotion
    for (int i=0; i<bvhObj->num_channel; i++)
        currentMotion[i] = bvhObj->motion[ currentFrame*bvhObj->num_channel + i];
    updateGL();
}

// switch damening on and off from checkbox input
void BVHWidget::setDampening(int state)
{
    // implicit cast to boolean
    dampening = state;
}

// update lambda for IK with slider value
void BVHWidget::updateLambda(int newLambda)
{
    lambda = newLambda;
}

// positive play direction is forward
void BVHWidget::changePlayDirPlus()
{
    playDir = 1;
} 

// negative play direction is backward
void BVHWidget::changePlayDirMinus()
{
    playDir = -1;
}

// resets the rotation of the arc ball
void BVHWidget::resetRotation()
{
    // reset object ball data
    Ball_Init(&objectBall);		
    Ball_Place(&objectBall, qOne, dimension);
    updateGL();
}

// 
// Arc Ball
//

// returns the rotation in the arcball as a glm mat4
glm::mat4 BVHWidget::getArcBallTransform()
{
    glm::mat4 transform(1.0);
    float mNow[16];
    Ball_Value(&objectBall, mNow);
    for(int col = 0; col < 4; col++)
        for(int row = 0; row < 3; row++)
            transform[col][row] = mNow[col * 4 + row];
    return transform;
}