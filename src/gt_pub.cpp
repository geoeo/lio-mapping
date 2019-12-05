//
// Created by marc on 05.12.19.
//

#include "gt_pub.h"

void lio::GT_PUB::GtPoseHandler(const geometry_msgs::PoseStamped::ConstPtr &gt_pose){

    gt_trans_.stamp_ = gt_pose->header.stamp;
    gt_trans_.setOrigin(tf::Vector3(gt_pose->pose.position.x,
                                    gt_pose->pose.position.y,
                                    gt_pose->pose.position.z));
    gt_trans_.setRotation(tf::Quaternion(gt_pose->pose.orientation.x, gt_pose->pose.orientation.y, gt_pose->pose.orientation.z, gt_pose->pose.orientation.w));

    tf_broadcaster_.sendTransform(gt_trans_);

}
