// Copyright (c) 2015, Robot Control and Pattern Recognition Group,
// Institute of Control and Computation Engineering
// Warsaw University of Technology
//
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Warsaw University of Technology nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Dawid Seredynski
//

#include <ros/ros.h>
#include "ros/package.h"
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <stdlib.h>
#include <stdio.h>

#include "kdl_conversions/kdl_msg.h"

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <collision_convex_model/collision_convex_model.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

using octomap_msgs::Octomap;

class OctomapFilter {
    ros::NodeHandle nh_;
/*    bool point_cloud_processed_;
    double tolerance_;

    tf::TransformListener tf_listener_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>* m_pointCloudSub;
    tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfPointCloudSub;
    ros::Publisher pub_pc_;

    typedef pcl::PointCloud<pcl::PointXYZRGB> PclPointCloud;
    PclPointCloud pc_;
    ros::Time pc_stamp_;
    std::string pc_frame_id_;
*/

    tf::TransformListener tf_listener_;

    boost::shared_ptr<octomap::OcTree> m_octree;
    boost::shared_ptr<octomap::OcTree> m_unknowntree;

    std_msgs::ColorRGBA m_color;

    ros::Subscriber m_fullMapSub;
    ros::Publisher  m_binaryMapPub;
    ros::Publisher  m_binaryUnknownMapPub, m_markerPub, m_markerPubMerged;

    boost::shared_ptr<self_collision::CollisionModel> col_model_;
    std::vector<KDL::Frame > links_tf_;

public:

    void callback(const Octomap& map)
    {
        m_octree.reset(dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(map)));
        publishAll(map.header.stamp, map.header.frame_id);
    }

    OctomapFilter()
        : nh_()
    {
        m_fullMapSub = nh_.subscribe("octomap_full", 1, &OctomapFilter::callback, this);
        m_binaryUnknownMapPub = nh_.advertise<Octomap>("octomap_unknown_binary", 1, true);
        m_binaryMapPub = nh_.advertise<Octomap>("octomap_merged_binary", 1, true);
        m_markerPub = nh_.advertise<visualization_msgs::MarkerArray>("unknown_cells_vis_array", 1, true);
        m_markerPubMerged = nh_.advertise<visualization_msgs::MarkerArray>("merged_cells_vis_array", 1, true);
/*
        m_unknowntree.reset(new octomap::OcTree(0.025));
        m_unknowntree->updateNode(octomath::Vector3(-3,-3,-3), true, true);
        m_unknowntree->updateNode(octomath::Vector3(-3,-3,3), true, true);
        m_unknowntree->updateNode(octomath::Vector3(-3,3,-3), true, true);
        m_unknowntree->updateNode(octomath::Vector3(-3,3,3), true, true);
        m_unknowntree->updateNode(octomath::Vector3(3,-3,-3), true, true);
        m_unknowntree->updateNode(octomath::Vector3(3,-3,3), true, true);
        m_unknowntree->updateNode(octomath::Vector3(3,3,-3), true, true);
        m_unknowntree->updateNode(octomath::Vector3(3,3,3), true, true);
        m_unknowntree->updateInnerOccupancy();
        std::cout << "writing octomap to .ot file" << std::endl;
        if (!m_unknowntree->write("/home/dseredyn/ws_stero/map.ot")) {
            std::cout << "error" << std::endl;
        }
        std::cout << "done" << std::endl;
*/

        m_unknowntree.reset(new octomap::OcTree(0.1));
        for (double z = 0.0; z < 2.0; z += 0.05) {
            for (double y = -1.5; y < 1.5; y += 0.05) {
                for (double x = -1.5; x < 1.5; x += 0.05) {
                    m_unknowntree->updateNode(octomath::Vector3(x,y,z), true, true);
                }
            }
        }

        m_unknowntree->updateInnerOccupancy();

        // read ROS param
        std::string robot_description_str;
        nh_.getParam("/robot_description", robot_description_str);

        //
        // collision model
        //
        col_model_ = self_collision::CollisionModel::parseURDF(robot_description_str);

        links_tf_.resize(col_model_->getLinksCount());

    }

    ~OctomapFilter() {
    }

    void publishAll(const ros::Time& rostime, const std::string& frame_id) {
        m_color.r = 0;
        m_color.g = 1;
        m_color.b = 0;
        m_color.a = 1;

        int m_treeDepth = m_octree->getTreeDepth();
        int m_maxTreeDepth = m_treeDepth;

        for (double z = 0.0; z < 2.0; z += 0.05) {
            for (double y = -1.5; y < 1.5; y += 0.05) {
                for (double x = -1.5; x < 1.5; x += 0.05) {
                    octomap::OcTreeNode *node = m_octree->search(octomath::Vector3(x,y,z));
                    if (node) {
                        m_unknowntree->updateNode(octomath::Vector3(x,y,z), false, true);
                    }
                }
            }
        }

//    for (octomap::OcTree::leaf_bbx_iterator it = m_octree->begin_leafs_bbx(octomath::Vector3(-2,-2,-2), octomath::Vector3(2,2,2)),
//        end = m_octree->end_leafs_bbx(); it != end; ++it)
/*    for (octomap::OcTree::iterator it = m_octree->begin(),//octomath::Vector3(-2,-2,-2), octomath::Vector3(2,2,2)),
        end = m_octree->end(); it != end; ++it)
    {
//        if (m_unknowntree->isNodeOccupied(*it)){
        m_unknowntree->updateNode(it.getCoordinate(), false, true);
    }
*/

        m_unknowntree->updateInnerOccupancy();

        //bool tf_error = false;
        for (int l_idx = 0; l_idx < col_model_->getLinksCount(); l_idx++) {
            const boost::shared_ptr< self_collision::Link > plink = col_model_->getLink(l_idx);
            if (plink->collision_array.size() == 0) {
                continue;
            }
            tf::StampedTransform tf_W_L;
            try {
                tf_listener_.lookupTransform(frame_id, plink->name, rostime, tf_W_L);
            } catch(tf::TransformException& ex){
                ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
                continue;
                //tf_error = true;
                //break;
            }
            geometry_msgs::TransformStamped tfm_W_L;
            KDL::Frame T_W_L;
            tf::transformStampedTFToMsg(tf_W_L, tfm_W_L);
            tf::transformMsgToKDL(tfm_W_L.transform, T_W_L);
            links_tf_[l_idx] = T_W_L;

            for (self_collision::Link::VecPtrCollision::const_iterator it=plink->collision_array.begin(), end=plink->collision_array.end(); it != end; ++it) {
                self_collision::removeNodesFromOctomap(m_unknowntree, (*it)->geometry, T_W_L * (*it)->origin);
            }
        }

//        if (!tf_error) {
//            self_collision::removeNodesFromOctomap(boost::shared_ptr<octomap::OcTree > &oc, const boost::shared_ptr<Geometry > &geom, const KDL::Frame &T_O_G) {
//        }
        m_unknowntree->updateInnerOccupancy();

        m_unknowntree->prune();

        //
        // merge octomaps
        //
        for (octomap::OcTree::iterator it = m_unknowntree->begin_leafs(),
            end = m_unknowntree->end_leafs(); it != end; ++it)
        {
            if (m_unknowntree->isNodeOccupied(*it)) {
                octomap::OcTreeKey key = m_octree->coordToKey( it.getCoordinate() );
                int steps = it.getSize() / m_octree->getResolution();
                if (steps == 0) {
                    steps = 1;
                }
                for (int i = (-steps)/2; i <= steps/2; ++i) {
                    for (int j = (-steps)/2; j <= steps/2; ++j) {
                        for (int k = (-steps)/2; k <= steps/2; ++k) {
                            m_octree->updateNode(octomap::OcTreeKey(key[0]+i, key[1]+j, key[2]+k), true, true);
                        }
                    }
                }
            }
        }
        m_octree->updateInnerOccupancy();

        m_octree->prune();

        //
        // visualize
        //
        // init markers:
        visualization_msgs::MarkerArray occupiedNodesVis;
        // each array stores all cubes of a different size, one for each depth level:
        occupiedNodesVis.markers.resize(m_treeDepth+1);

        // now, traverse all leafs in the tree:
        for (octomap::OcTree::iterator it = m_unknowntree->begin(m_maxTreeDepth),
            end = m_unknowntree->end(); it != end; ++it)
        {
            if (m_unknowntree->isNodeOccupied(*it)){
                double z = it.getZ();
                double size = it.getSize();
                double x = it.getX();
                double y = it.getY();

                unsigned idx = it.getDepth();
                assert(idx < occupiedNodesVis.markers.size());

                geometry_msgs::Point cubeCenter;
                cubeCenter.x = x;
                cubeCenter.y = y;
                cubeCenter.z = z;

                occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
            }
        }

        for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
            double size = m_unknowntree->getNodeSize(i);

            occupiedNodesVis.markers[i].header.frame_id = frame_id;
            occupiedNodesVis.markers[i].header.stamp = rostime;
            occupiedNodesVis.markers[i].ns = "map";
            occupiedNodesVis.markers[i].id = i;
            occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
            occupiedNodesVis.markers[i].scale.x = size;
            occupiedNodesVis.markers[i].scale.y = size;
            occupiedNodesVis.markers[i].scale.z = size;
            occupiedNodesVis.markers[i].color = m_color;


            if (occupiedNodesVis.markers[i].points.size() > 0)
              occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
            else
              occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
        }

        m_markerPub.publish(occupiedNodesVis);

        //
        // visualize merged
        //
        // init markers:
        visualization_msgs::MarkerArray occupiedNodesVisMerged;
        // each array stores all cubes of a different size, one for each depth level:
        occupiedNodesVisMerged.markers.resize(m_treeDepth+1);

        // now, traverse all leafs in the tree:
        for (octomap::OcTree::iterator it = m_octree->begin(m_maxTreeDepth),
            end = m_octree->end(); it != end; ++it)
        {
            if (m_octree->isNodeOccupied(*it)){
                double z = it.getZ();
                double size = it.getSize();
                double x = it.getX();
                double y = it.getY();

                unsigned idx = it.getDepth();
                assert(idx < occupiedNodesVisMerged.markers.size());

                geometry_msgs::Point cubeCenter;
                cubeCenter.x = x;
                cubeCenter.y = y;
                cubeCenter.z = z;

                occupiedNodesVisMerged.markers[idx].points.push_back(cubeCenter);
            }
        }

        for (unsigned i= 0; i < occupiedNodesVisMerged.markers.size(); ++i){
            double size = m_octree->getNodeSize(i);

            occupiedNodesVisMerged.markers[i].header.frame_id = frame_id;
            occupiedNodesVisMerged.markers[i].header.stamp = rostime;
            occupiedNodesVisMerged.markers[i].ns = "map";
            occupiedNodesVisMerged.markers[i].id = i;
            occupiedNodesVisMerged.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
            occupiedNodesVisMerged.markers[i].scale.x = size;
            occupiedNodesVisMerged.markers[i].scale.y = size;
            occupiedNodesVisMerged.markers[i].scale.z = size;
            occupiedNodesVisMerged.markers[i].color = m_color;


            if (occupiedNodesVisMerged.markers[i].points.size() > 0)
              occupiedNodesVisMerged.markers[i].action = visualization_msgs::Marker::ADD;
            else
              occupiedNodesVisMerged.markers[i].action = visualization_msgs::Marker::DELETE;
        }

        m_markerPubMerged.publish(occupiedNodesVisMerged);


        Octomap map;
        map.header.frame_id = frame_id;
        map.header.stamp = rostime;
        if (octomap_msgs::binaryMapToMsg(*boost::dynamic_pointer_cast<octomap::OcTree >(m_octree), map))
            m_binaryMapPub.publish(map);
        else
            ROS_ERROR("Error serializing OctoMap");

        Octomap map2;
        map2.header.frame_id = frame_id;
        map2.header.stamp = rostime;
        if (octomap_msgs::binaryMapToMsg(*boost::dynamic_pointer_cast<octomap::OcTree >(m_unknowntree), map2))
            m_binaryUnknownMapPub.publish(map2);
        else
            ROS_ERROR("Error serializing OctoMap");
    }

    void spin() {
        ros::spin();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_filter");
    OctomapFilter of;
    of.spin();
    return 0;
}


