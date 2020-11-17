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


    IKinematics(BVHWidget* viewer);
    ~IKinematics();

    // IK angles
    Eigen::MatrixXf deltaAngles;
    //  new angles
    Eigen::VectorXf tAngles; 

    // applies inverse kinematics for a single point to get the new angles as an array of doubles
    double* computeAngles();
    // createse a jacobian matrix for n points
    Eigen::MatrixXf computeJacobian(int index, int nPoints);
    Eigen::MatrixXf invertJacobian(Eigen::MatrixXf jacobian);
    void computeIK(glm::vec3 t, int index);
    // return an array of new positions for computing the jacobian

    // get the position of a joint in world space
    glm::vec3 getJointWorldPos(int index, bool test);
    // get the position of a joint in world space with small change to the angle in channel
    glm::vec3 getJointWorldPos(int index, int channelIndex, bool test);
    // returns the rotation of a joint as a mat4
    glm::mat4 getJointRotation(BVH::Joint* joint, int channelIndex);

    void setBvhData(BVH* bvh);
    
    // motion data
    BVHWidget* bvhViewer;

    // for jacobian computation
    float deltaTheta;

};

#endif
