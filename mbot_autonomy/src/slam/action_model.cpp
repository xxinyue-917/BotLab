#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(void)
: k1_(0.02f)
, k2_(0.005f)
, min_dist_(0.00)
, min_theta_(0.00)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd());
}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose2D_t& odometry)
{
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose2D_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if(!initialized_){
        previousPose_ = odometry;
        initialized_ = true;
    }

    // Calculate Action Model Here
    float deltaX = odometry.x - previousPose_.x;
    float deltaY = odometry.y - previousPose_.y;
    float deltaTheta = odometry.theta - previousPose_.theta;
    // std::cout << "dx:" << deltaX << "dy:" << deltaY << "dTheta:" << deltaTheta << std::endl;
    rot1_ = angle_diff(std::atan2(deltaY, deltaX), previousPose_.theta);
    trans_ = std::sqrt(deltaX*deltaX + deltaY*deltaY);

    // If the bot has a giant rotation
    float direction = 1.0;
    if(std::abs(rot1_) > M_PI_2){
        rot1_ = angle_diff(M_PI, rot1_);
        direction = -1.0;
        // std::cout << "------!!!------Theta Too Large------!!!------" << std::endl;
    }
    rot2_ = angle_diff(deltaTheta, rot1_);

    // Check if the bot moved
    // bool moved_ = (std::abs(deltaX) > min_dist_) || (std::abs(deltaY) > min_dist_) || (std::abs(deltaTheta) > min_theta_);
    bool moved_ = (deltaX != 0.0) || (deltaY != 0.0) || (deltaTheta != 0.0);
    
    if(moved_){
        rot1Std_ = std::sqrt(k1_ * std::abs(rot1_));
        transStd_ = std::sqrt(k2_ * std::abs(trans_));
        rot2Std_= std::sqrt(k1_ * std::abs(rot2_));
    }

    trans_ *= direction;
    previousPose_ = odometry;
    utime_ = odometry.utime;
    // std::cout << "odometry.x:" << odometry.x << "odometry.y:" << odometry.y << "odometry.theta:" << odometry.theta << std::endl;
    // std::cout << "theta: " << wrap_to_pi(odometry.theta+rot1_+rot2_) << std::endl;

    // The return here is just the judge if the robot moved
    return moved_;
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample = sample;
    
    float sampledRot1 = std::normal_distribution<>(rot1_, rot1Std_)(numberGenerator_);
    float sampledTrans = std::normal_distribution<>(trans_, transStd_)(numberGenerator_);
    float sampledRot2 = std::normal_distribution<>(rot2_, rot2Std_)(numberGenerator_);

    newSample.pose.x += sampledTrans * std::cos(sample.pose.theta + sampledRot1);
    newSample.pose.y += sampledTrans * std::sin(sample.pose.theta + sampledRot1);
    newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampledRot1 + sampledRot2);
    newSample.pose.utime = utime_;
    newSample.parent_pose = sample.pose;

    return newSample;
}
