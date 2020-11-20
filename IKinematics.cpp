#include <deque>

#include <glm/gtc/matrix_transform.hpp>
#include <Eigen/LU>

#include "IKinematics.h"

// constructor
IKinematics::IKinematics(BVHWidget* viewer)
{
    // pointer to bvh widget for visualisation
    bvhViewer = viewer;
    // small angle increment
    deltaTheta = 0.01;
}

// destructor
IKinematics::~IKinematics()
{

}

// computes and returns the jacobian from data in bvh viewer
Eigen::MatrixXf IKinematics::computeJacobian()
{
    // unitialised jacobian 
    Eigen::MatrixXf jacobian = Eigen::MatrixXf(3 * (bvhViewer->selectedJoints.size()), bvhViewer->bvhObj->num_channel - 3);
    // loop over the rows (selected joints x,y and z)
    for (int j=0; j<bvhViewer->selectedJoints.size(); j++)
    {
        // get the joint's world position 
        glm::vec3 jointPos1 = getJointWorldPos(bvhViewer->selectedJoints[j]->index);
        // loop over the columns (rotation channels)
        for (int i=3; i<bvhViewer->bvhObj->num_channel; i++)
        {
            // get the position perturbed by delta theta in each channel
            glm::vec3 jointPos2 = getJointWorldPos(bvhViewer->selectedJoints[j]->index, i);
            // set the entries accordingly
            jacobian(j * 3    , i - 3) = (jointPos2.x - jointPos1.x) / deltaTheta;
            jacobian(j * 3 + 1, i - 3) = (jointPos2.y - jointPos1.y) / deltaTheta;
            jacobian(j * 3 + 2, i - 3) = (jointPos2.z - jointPos1.z) / deltaTheta;
        }
    }
    return jacobian;
}

// invert the jacobian matrix
Eigen::MatrixXf IKinematics::invertJacobian(Eigen::MatrixXf jacobian)
{
    // init inverse
    Eigen::MatrixXf invJacobian;
    // compute inverse
    if (bvhViewer->dampening)
        invJacobian = jacobian.transpose() * (jacobian * jacobian.transpose() + pow(bvhViewer->lambda / 50.0, 2.0) * Eigen::MatrixXf::Identity(jacobian.rows(), jacobian.rows())).inverse();
    else
        invJacobian = jacobian.transpose() * (jacobian * jacobian.transpose()).inverse();
    // check matrix can be inverted, otherwise return zero matrix 
    for (int col=0; col<invJacobian.cols(); col++)
        for (int row=0; row<invJacobian.rows(); row++)
            if (isnan(invJacobian(row,col)))
                return Eigen::MatrixXf::Zero(invJacobian.rows(), invJacobian.cols());
    return invJacobian;
}

// overload for getting just the world position 
glm::vec3 IKinematics::getJointWorldPos(int index)
{
    return getJointWorldPos(index, -1);
}

// get the world position and apply a change to the angle if desired (for jacobian)
glm::vec3 IKinematics::getJointWorldPos(int index, int channelIndex)
{
    // a stack of joints that go from desired joint to root
    std::deque<BVH::Joint*> pathToRoot;
    // get the joint at desired index
    BVH::Joint* current = bvhViewer->bvhObj->joints[index];
    // iterate back up the joint tree till we reach the root
    while (current->parent != NULL)
    {
        pathToRoot.push_front(current);
        current = current->parent;
    }
    // include the root
    pathToRoot.push_front(current);
    // start with an angle perturbation of 0
    float perturb = 0;
    // initialise an identity matrix for storing translations and rotations for the joint
    glm::mat4 transform(1.0);
    // loop over the path to the joint starting at the root
    for (int i=0; i<pathToRoot.size(); i++)
    {
        BVH::Joint* joint = pathToRoot[i];
        // then apply translation
        transform = glm::translate(transform, glm::vec3(joint->offset[0], joint->offset[1], joint->offset[2]));
        // loop over the joint's channels   
        for (int j=0; j<joint->channels.size(); j++)
        {
            BVH::Channel* channel = joint->channels[j];
            // add angle perturbation for jacobian computation
            if (channel->index == channelIndex)
                perturb = deltaTheta;
            else
                perturb = 0;
            // store the rotation of the joint in the transform
            if (channel->type == BVH::X_ROTATION)
                transform = glm::rotate( transform, (float)glm::radians(bvhViewer->currentMotion[channel->index] + perturb), glm::vec3(1.0f, 0.0f, 0.0f));
            else if (channel->type == BVH::Y_ROTATION)
                transform = glm::rotate( transform, (float)glm::radians(bvhViewer->currentMotion[channel->index] + perturb), glm::vec3(0.0f, 1.0f, 0.0f));
            else if (channel->type == BVH::Z_ROTATION)
                transform = glm::rotate( transform, (float)glm::radians(bvhViewer->currentMotion[channel->index] + perturb), glm::vec3(0.0f, 0.0f, 1.0f));
        }
    }
    // start at the root
    glm::vec4 jointPos(bvhViewer->currentMotion[0], bvhViewer->currentMotion[1], bvhViewer->currentMotion[2], 1.0);
    // store the transformed root position, i.e the joint's world space position 
    jointPos = transform * jointPos;
    // return it as a vec3
    return glm::vec3(jointPos.x, jointPos.y, jointPos.z);
}

// returns the transform from root to joint world pos for a given joint 
glm::mat4 IKinematics::getJointWorldTransform(int index)
{
    // a stack of joints that go from desired joint to root
    std::deque<BVH::Joint*> pathToRoot;
    // get the joint at desired index
    BVH::Joint* current = bvhViewer->bvhObj->joints[index];
    // stops a the root
    while (current->parent != NULL)
    {
        pathToRoot.push_front(current);
        current = current->parent;
    }
    // include the root
    pathToRoot.push_front(current);
    // initialise an identity matrix for storing translations and rotations for the joint
    glm::mat4 transform(1.0);
    // loop over the path to the joint starting at the root
    for (int i=0; i<pathToRoot.size(); i++)
    {
        BVH::Joint* joint = pathToRoot[i];
        // then apply translation
        transform = glm::translate(transform, glm::vec3(joint->offset[0], joint->offset[1], joint->offset[2]));
        
        for (int j=0; j<joint->channels.size(); j++)
        {
            BVH::Channel* channel = joint->channels[j];
            // store the rotation of the joint in the transform
            if (channel->type == BVH::X_ROTATION)
                transform = glm::rotate( transform, (float)glm::radians(bvhViewer->currentMotion[channel->index]), glm::vec3(1.0f, 0.0f, 0.0f));
            else if (channel->type == BVH::Y_ROTATION)
                transform = glm::rotate( transform, (float)glm::radians(bvhViewer->currentMotion[channel->index]), glm::vec3(0.0f, 1.0f, 0.0f));
            else if (channel->type == BVH::Z_ROTATION)
                transform = glm::rotate( transform, (float)glm::radians(bvhViewer->currentMotion[channel->index]), glm::vec3(0.0f, 0.0f, 1.0f));
        }
    }
    return transform;
}

// compute new angles for a figure with inverse kinematics for any combination joints
void IKinematics::computeIK()
{
    // initialise the vector E = t-c
    Eigen::VectorXf E(3 * bvhViewer->selectedJoints.size());
    // loop over the joints to get the entries in E 
    for (int i=0; i<bvhViewer->selectedJoints.size(); i++)
    {
        glm::vec3 diff = bvhViewer->selectedJoints[i]->goalPos-getJointWorldPos(bvhViewer->selectedJoints[i]->index);
        E(3 * i    ) = diff.x;
        E(3 * i + 1) = diff.y;
        E(3 * i + 2) = diff.z;
    }
    // compute the jacobian and invert it
    Eigen::MatrixXf invJacobian = invertJacobian(computeJacobian());
    // difference in angles 
    Eigen::VectorXf deltaAngles = invJacobian * E;
    // vector for all the new angles 
    tAngles = Eigen::VectorXf(deltaAngles.rows());
    // add delta to current frame's angles to get new angles from IK 
    for (int i=0; i<bvhViewer->bvhObj->num_channel - 3; i++)
        tAngles(i) = bvhViewer->bvhObj->motion[bvhViewer->currentFrame * bvhViewer->bvhObj->num_channel + i + 3] + deltaAngles(i);    
    // then set them in the current motion
    for (int i=0; i<bvhViewer->bvhObj->num_channel - 3; i++)
        bvhViewer->currentMotion[i + 3] += deltaAngles(i);
}


