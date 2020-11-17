#include <deque>

#include <glm/gtc/matrix_transform.hpp>
#include <Eigen/LU>

#include "IKinematics.h"

IKinematics::IKinematics(BVHWidget* viewer)
{
    bvhViewer = viewer;
    // small angle increment
    deltaTheta = 0.0001;

    // initialise 
    //for (int i=0; i<bvhViewer->bvhObj->num_channel-3; i++)
    //    tAngles(i) = bvhViewer->bvhObj->motion[ bvhViewer->currentFrame * bvhViewer->bvhObj->num_channel + i + 3];
}

IKinematics::~IKinematics()
{
    // delete IK data
    //jacobian.resize(0,0);
    //invJacobian.resize(0,0);
    // don't delete motion data since done already by BVH widget
}

// pass the joint position as an argument (we have a method in BVH widget)
Eigen::MatrixXf IKinematics::computeJacobian(int index,  int nPoints)
{
    // unitialised jacobian 
    Eigen::MatrixXf jacobian = Eigen::MatrixXf(3*nPoints, bvhViewer->bvhObj->num_channel-3);
    // get the world position of the joint we want to change
    glm::vec3 jointPos1 = getJointWorldPos(index, false);
    // std::cout << jointPos1.x << " " << jointPos1.y << " " << jointPos1.z << std::endl;
    // loop over the columns
    for(int i = 3; i < bvhViewer->bvhObj->num_channel; i++)
    {
        // get the position perturbed by delta theta 
        //std::cout << "new difference" << std::endl;
        glm::vec3 jointPos2 = getJointWorldPos(index, i, false);
        // std::cout << jointPos2.x << " " << jointPos2.y << " " << jointPos2.z << std::endl;
        //std::cout << jointPos1.x << " " << jointPos1.x << " " << jointPos1.x << std::endl;
        // set the entries accordingly
        jacobian(0,i-3) = (jointPos2.x - jointPos1.x) / deltaTheta;
        jacobian(1,i-3) = (jointPos2.y - jointPos1.y) / deltaTheta;
        jacobian(2,i-3) = (jointPos2.z - jointPos1.z) / deltaTheta;
    }

    //std::cout << "jacobian:\n" << jacobian << std::endl;
    return jacobian;
}

// invert the jacobian matrix
Eigen::MatrixXf IKinematics::invertJacobian(Eigen::MatrixXf jacobian)
{
    Eigen::MatrixXf invJacobian = jacobian.transpose() * (jacobian * jacobian.transpose()).inverse();
    //std::cout << "inverse jacobian:\n " << invJacobian << std::endl;
    //std::cout << (jacobian * jacobian.transpose()).inverse() << std::endl;
    //std::cout << (jacobian * jacobian.transpose()) << std::endl;
    //std::cout << "inverse\n" << invJacobian << std::endl;
    return invJacobian;
}


// overload for getting just the world position 
glm::vec3 IKinematics::getJointWorldPos(int index, bool test)
{
    return getJointWorldPos(index, -1, test);
}

// get the world position and applies a change to the angle if desired (for jacobian)
glm::vec3 IKinematics::getJointWorldPos(int index, int channelIndex, bool test)
{
    // a stack of joints that go from desired joint to root
    std::deque<BVH::Joint*> pathToRoot;
    // get the joint at desired index
    BVH::Joint* current = bvhViewer->bvhObj->joints[ index ];

    // stops a the root
    while(current->parent != NULL)
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
    for( int i=0; i<pathToRoot.size(); i++)
    {
        //std::cout << "joint " << i << std::endl;
        BVH::Joint* joint = pathToRoot[i];
        // then apply translation
        transform = glm::translate(transform, glm::vec3( joint->offset[ 0 ], joint->offset[ 1 ], joint->offset[ 2 ]));
        
        for ( int j=0; j<joint->channels.size(); j++ )
        {
            BVH::Channel *  channel = joint->channels[ j ];

            if (test)
            {
                // store the rotation of the joint in the transform
                if ( channel->type == BVH::X_ROTATION )
                    transform = glm::rotate( transform, (float)glm::radians((int)tAngles[ channel->index - 3 ] % 360 + (tAngles[ channel->index - 3 ] - (int)tAngles[ channel->index - 3 ])), glm::vec3( 1.0f, 0.0f, 0.0f ));
                else if ( channel->type == BVH::Y_ROTATION )
                    transform = glm::rotate( transform, (float)glm::radians((int)tAngles[ channel->index - 3 ] % 360 + (tAngles[ channel->index - 3 ] - (int)tAngles[ channel->index - 3 ])), glm::vec3( 0.0f, 1.0f, 0.0f ));
                else if ( channel->type == BVH::Z_ROTATION )
                    transform = glm::rotate( transform, (float)glm::radians((int)tAngles[ channel->index - 3 ] % 360 + (tAngles[ channel->index - 3 ] - (int)tAngles[ channel->index - 3 ])), glm::vec3( 0.0f, 0.0f, 1.0f ));
            }
            else
            {
                // add angle perturbation for jacobian computation
                if (channel->index == channelIndex)
                    perturb = deltaTheta;
                else
                    perturb = 0;
                
                // store the rotation of the joint in the transform
                if ( channel->type == BVH::X_ROTATION )
                    transform = glm::rotate( transform, (float)glm::radians(bvhViewer->bvhObj->motion[ bvhViewer->currentFrame * bvhViewer->bvhObj->num_channel + channel->index ] + perturb), glm::vec3( 1.0f, 0.0f, 0.0f ));
                else if ( channel->type == BVH::Y_ROTATION )
                    transform = glm::rotate( transform, (float)glm::radians(bvhViewer->bvhObj->motion[ bvhViewer->currentFrame * bvhViewer->bvhObj->num_channel + channel->index ] + perturb), glm::vec3( 0.0f, 1.0f, 0.0f ));
                else if ( channel->type == BVH::Z_ROTATION )
                    transform = glm::rotate( transform, (float)glm::radians(bvhViewer->bvhObj->motion[ bvhViewer->currentFrame * bvhViewer->bvhObj->num_channel + channel->index ] + perturb), glm::vec3( 0.0f, 0.0f, 1.0f ));
            }

        }
    }

    // start at the root
    glm::vec4 jointPos(bvhViewer->bvhObj->motion[ bvhViewer->currentFrame * bvhViewer->bvhObj->num_channel   ], 
                    bvhViewer->bvhObj->motion[ bvhViewer->currentFrame * bvhViewer->bvhObj->num_channel + 1 ], 
                    bvhViewer->bvhObj->motion[ bvhViewer->currentFrame * bvhViewer->bvhObj->num_channel + 2 ], 1.0);
    // store the transformed root position, the joint's world space position 
    jointPos = transform * jointPos;
    // return it as a vec3
    return glm::vec3(jointPos.x, jointPos.y, jointPos.z);
}

// compute the inverse inematics of a joint at given index for desired position t
void IKinematics::computeIK(glm::vec3 t, int index)
{
    // get the vector E = t-c
    glm::vec3 dif = t - getJointWorldPos(index, false);
    Eigen::Vector3f E;
    E(0) = dif.x;
    E(1) = dif.y;
    E(2) = dif.z;

    // compute the jacobian and invert it
    // Eigen::MatrixXf jacobian = computeJacobian(index, 1);
    Eigen::MatrixXf invJacobian = invertJacobian(computeJacobian(index, 1));

    // difference in angles 
    deltaAngles = invJacobian * E;

    tAngles = Eigen::VectorXf(deltaAngles.rows());

    // add delta to current angles to get new angles from IK
    for (int i=0; i<bvhViewer->bvhObj->num_channel-3; i++)
        tAngles(i) = bvhViewer->bvhObj->motion[ bvhViewer->currentFrame * bvhViewer->bvhObj->num_channel + i + 3] + deltaAngles(i);
    
    for (int i=0; i<bvhViewer->bvhObj->num_channel-3; i++)
        bvhViewer->bvhObj->motion[ bvhViewer->currentFrame * bvhViewer->bvhObj->num_channel + i + 3] += deltaAngles(i);
}


