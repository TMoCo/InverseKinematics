#ifndef _IKINEMATICS_H
#define _IKINEMATICS_H

#include "BVH.h"
#include "BVHWidget.h"

#include <Eigen/Core>
#include <glm/glm.hpp>

// need a class declaration of BVHwidget for compiling
class BVHWidget;

class IKinematics
{
    public:

    // constructor
    IKinematics(BVHWidget* viewer);
    // destructor
    ~IKinematics();

    // createse a jacobian matrix for n points
    Eigen::MatrixXf computeJacobian();
    // creates the inverse of the jacobian
    Eigen::MatrixXf invertJacobian(Eigen::MatrixXf jacobian);
    // applies inverse kinematics for a combination of points to get difference in angles for invers kinematics
    void computeIK();

    // get the position of a joint in world space
    glm::vec3 getJointWorldPos(int index);
    // get the position of a joint in world space with small change to the angle in desired channel
    glm::vec3 getJointWorldPos(int index, int channelIndex);
    // get the transform of a joint 
    glm::mat4 getJointWorldTransform(int index);
    // returns the rotation of a joint as a mat4
    glm::mat4 getJointRotation(BVH::Joint* joint, int channelIndex);
    

    // for jacobian computation
    float deltaTheta;
    // IK angles difference
    //Eigen::MatrixXf deltaAngles;
    // new angles
    Eigen::VectorXf tAngles; 


    // motion data
    BVHWidget* bvhViewer;

};

#endif
