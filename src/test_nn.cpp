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
/*#include "ros/package.h"
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
*/
#include <string>
#include <stdlib.h>
#include <stdio.h>

#include "kdl_conversions/kdl_msg.h"
/*
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
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
*/
#include <collision_convex_model/collision_convex_model.h>
#include <kin_dyn_model/kin_model.h>

double uniform(double min, double max) {
    return double(rand())/RAND_MAX*(max-min) + min;
}

class Neuron {
public:
    void setValue(double v);
    void calculate();
    double getValue() const;

    std::vector<std::pair<Neuron*, double > > neurons_weights_;
    double bias_;
private:
    double v_;
};

void Neuron::setValue(double v) {
    v_ = v;
}

void Neuron::calculate() {
    double sum = 0;
    for (int i = 0; i < neurons_weights_.size(); ++i) {
        sum += neurons_weights_[i].first->getValue() * neurons_weights_[i].second;
    }
    v_ = 1.0/(1.0+exp(-sum-bias_));
}

double Neuron::getValue() const {
    return v_;
}

class Network {
public:

    void addLayer(size_t count) {
        std::vector<std::shared_ptr<Neuron > > new_layer;
        for (int i = 0; i < count; ++i) {
            std::shared_ptr<Neuron > n(new Neuron());
            if (layers_.size() > 0) {
                std::vector<std::shared_ptr<Neuron > > &prev_layer = layers_[layers_.size()-1];
                for (int j = 0; j < prev_layer.size(); ++j) {
                    n->neurons_weights_.push_back( std::pair<Neuron*, double >(prev_layer[j].get(), uniform(-1,1)) );
                }
            }
            n->bias_ = uniform(-1,1);
            new_layer.push_back( n );
        }
    }

    bool setInput(size_t idx, double v) {
        if (idx >= layers_[0].size()) {
            return false;
        }
        layers_[0][idx]->setValue(v);
        return true;
    }

    bool getOutput(size_t idx, double &v) {
        if (idx >= layers_[layers_.size()-1].size()) {
            return false;
        }
        v = layers_[layers_.size()-1][idx]->getValue();
        return true;
    }

private:
    std::vector<std::vector<std::shared_ptr<Neuron > > > layers_;
};

// http://neuralnetworksanddeeplearning.com/chap1.html

class TestNN {
    ros::NodeHandle nh_;

    boost::shared_ptr<KinematicModel> kin_model_;
    boost::shared_ptr<self_collision::CollisionModel> col_model_;
    std::vector<KDL::Frame > links_fk_;

    // ROS parameters
    std::vector<std::string > articulated_joint_names_;

public:

    TestNN()
        : nh_("~")
    {
    }

    ~TestNN() {
    }

    bool load() {
        std::string robot_description_str;
        nh_.getParam("/robot_description", robot_description_str);

        nh_.getParam("articulated_joint_names", articulated_joint_names_);
        if (articulated_joint_names_.size() == 0) {
            ROS_ERROR_STREAM( "ROS parameter 'articulated_joint_names' is not set" );
            return false;
        }

        //
        // collision model
        //
        col_model_ = self_collision::CollisionModel::parseURDF(robot_description_str);
        if (!col_model_) {
            ROS_ERROR_STREAM( "Could not parse '/robot_description' ROS param" );
            return false;
        }

        links_fk_.resize(col_model_->getLinksCount());

        kin_model_.reset( new KinematicModel(robot_description_str, articulated_joint_names_) );

        return true;
    }

    void spin() {

        Network n;
        n.addLayer(articulated_joint_names_.size());
        n.addLayer(articulated_joint_names_.size());
        n.addLayer(col_model_->getLinksCount());
        n.addLayer(col_model_->getLinksCount());

        Eigen::VectorXd q(articulated_joint_names_.size());

        for (int q_idx = 0; q_idx < articulated_joint_names_.size(); ++q_idx) {
            q(q_idx) = uniform( kin_model_->getLowerLimit(q_idx), kin_model_->getUpperLimit(q_idx));
            n.setInput(q_idx, q(q_idx));
        }
        kin_model_->calculateFkAll(q);

        // calculate forward kinematics for all links
        for (int l_idx = 0; l_idx < col_model_->getLinksCount(); l_idx++) {
            links_fk_[l_idx] = kin_model_->getFrame(col_model_->getLinkName(l_idx));
        }
/*
        const KDL::Frame &T_B_H = kin_model_->getFrame(camera_frame_id_);
        KDL::Frame T_H_B = T_B_H.Inverse();

        // frustrum boundaries
        double rx, ry, rz;
        rz = 1.0;
        rx = tan(fov_x_) * rz;
        ry = tan(fov_y_) * rz;
        // boundary planes normals (normals point outside)
        KDL::Vector n1(rz, 0, -rx), n2(-rz, 0, -rx), n3(0, rz, -ry), n4(0, -rz, -ry);
        n1.Normalize();
        n2.Normalize();
        n3.Normalize();
        n4.Normalize();

        size_t vertex_idx_base = 0;
        std::vector<KDL::Vector > vec_v;
        std::vector<Eigen::Vector3i > vec_f;
        std::vector<KDL::Vector > vec_fn;

        size_t max_num_vertices = 0;
        size_t max_num_faces = 0;
        for (int l_idx = 0; l_idx < col_model_->getLinksCount(); ++l_idx) {
            for (int c_idx = 0; c_idx < collision_meshes_[l_idx].size(); ++c_idx) {
                const aiScene *m = collision_meshes_[l_idx][c_idx].second;
                max_num_vertices += getNumVertices(m);
                max_num_faces += getNumFaces(m);
            }
        }
        vec_v.reserve(max_num_vertices);
        vec_f.reserve(max_num_faces);
        vec_fn.reserve(max_num_faces);

        // create mesh of all visible object surfaces
        for (int l_idx = 0; l_idx < col_model_->getLinksCount(); ++l_idx) {
            const KDL::Frame &T_B_L = links_fk_[l_idx];
            KDL::Frame T_H_L = T_H_B * T_B_L;

            // broadphase check using bounding sphere
            double bs_radius = link_bounding_spheres_[l_idx].first;
            const KDL::Vector &bs_center_L = link_bounding_spheres_[l_idx].second;
            const KDL::Vector bs_center_H = T_H_L * bs_center_L;
            if (KDL::dot(bs_center_H, n1) > bs_radius || KDL::dot(bs_center_H, n2) > bs_radius ||
                    KDL::dot(bs_center_H, n3) > bs_radius || KDL::dot(bs_center_H, n4) > bs_radius) {
                continue;
            }

            for (int c_idx = 0; c_idx < collision_meshes_[l_idx].size(); ++c_idx) {
                const KDL::Frame &T_L_M = collision_meshes_[l_idx][c_idx].first;
                KDL::Frame T_H_M = T_H_L * T_L_M;
                const aiScene *m = collision_meshes_[l_idx][c_idx].second;

                for (int m_idx = 0; m_idx < m->mNumMeshes; ++m_idx) {
                    aiMesh *mesh = m->mMeshes[m_idx];
                    for (int v_idx = 0; v_idx < mesh->mNumVertices; ++v_idx) {
                        const aiVector3D &v = mesh->mVertices[v_idx];
                        vec_v.push_back(T_H_M * KDL::Vector(v.x, v.y, v.z));
                    }
                    for (int f_idx = 0; f_idx < mesh->mNumFaces; ++f_idx) {
                        if (mesh->mFaces[f_idx].mNumIndices != 3) {
                            ROS_ERROR_STREAM( "Face is not triangle!" );
                        }
                        unsigned int i1 = mesh->mFaces[f_idx].mIndices[0]+vertex_idx_base;
                        unsigned int i2 = mesh->mFaces[f_idx].mIndices[1]+vertex_idx_base;
                        unsigned int i3 = mesh->mFaces[f_idx].mIndices[2]+vertex_idx_base;
                        KDL::Vector v12 = vec_v[i2] - vec_v[i1];
                        KDL::Vector v13 = vec_v[i3] - vec_v[i1];
                        KDL::Vector n = v12 * v13;

//                        if (n.z() > 0) {    // the face is pointing outwards camera
//                            continue;
//                        }
                        if (n.Norm() < 0.0000001) {      // the face is too small
                            continue;
                        }
                        n.Normalize();

                        // the whole face is outside frustrum
                        if ((KDL::dot(vec_v[i1], n1) > 0 && KDL::dot(vec_v[i2], n1) > 0 && KDL::dot(vec_v[i3], n1) > 0) ||
                                (KDL::dot(vec_v[i1], n2) > 0 && KDL::dot(vec_v[i2], n2) > 0 && KDL::dot(vec_v[i3], n2) > 0) ||
                                (KDL::dot(vec_v[i1], n3) > 0 && KDL::dot(vec_v[i2], n3) > 0 && KDL::dot(vec_v[i3], n3) > 0) ||
                                (KDL::dot(vec_v[i1], n4) > 0 && KDL::dot(vec_v[i2], n4) > 0 && KDL::dot(vec_v[i3], n4) > 0)) {
                            continue;
                        }
                        vec_f.push_back(Eigen::Vector3i(i1, i2, i3));
                        vec_fn.push_back(n);
                    }
                    vertex_idx_base += mesh->mNumVertices;
                }
            }
        }
        std::vector<KDL::Vector > pc_H;

        std::vector<double > rays(pc_width_*pc_height_, 1000000.0);
        for (int f_idx = 0; f_idx < vec_f.size(); ++f_idx) {
            const KDL::Vector &v1 = vec_v[vec_f[f_idx](0)];
            const KDL::Vector &v2 = vec_v[vec_f[f_idx](1)];
            const KDL::Vector &v3 = vec_v[vec_f[f_idx](2)];

            // calculate face angular extent
            double min_tg_x = std::min<double >(v1.x()/v1.z(), std::min<double >(v2.x()/v2.z(), v3.x()/v3.z()));
            double max_tg_x = std::max<double >(v1.x()/v1.z(), std::max<double >(v2.x()/v2.z(), v3.x()/v3.z()));
            double min_tg_y = std::min<double >(v1.y()/v1.z(), std::min<double >(v2.y()/v2.z(), v3.y()/v3.z()));
            double max_tg_y = std::max<double >(v1.y()/v1.z(), std::max<double >(v2.y()/v2.z(), v3.y()/v3.z()));

            double t;

            for (int x = 0; x < pc_width_; ++x) {
                double rx, ry, rz;
                rz = 1.0;
                rx = tan(fov_x_) * rz * (x * 2 - pc_width_) / pc_width_ / 2.0;
                double tg_x = rx/rz;
                if (tg_x < min_tg_x || tg_x > max_tg_x) {
                    continue;
                }
                for (int y = 0; y < pc_height_; ++y) {
                    ry = tan(fov_y_) * rz * (y * 2 - pc_height_) / pc_height_ / 2.0;
                    double tg_y = ry/rz;
                    if (tg_y < min_tg_y || tg_y > max_tg_y) {
                        continue;
                    }
                    KDL::Vector dir_H(rx, ry, rz);
                    dir_H.Normalize();
                    if (KDL::dot(dir_H, vec_fn[f_idx]) < 0 && rayTriangleIntersect(KDL::Vector(), dir_H, v1, v2, v3, vec_fn[f_idx], t) && t > 0.01) {
                        if (rays[x+y*pc_width_] > t) {
                            rays[x+y*pc_width_] = t;
                        }
                    }
                }
            }
        }

        for (int x = 0; x < pc_width_; ++x) {
            double rx, ry, rz;
            rz = 1.0;
            rx = tan(fov_x_) * rz * (x * 2 - pc_width_) / pc_width_ / 2.0;
            for (int y = 0; y < pc_height_; ++y) {
                ry = tan(fov_y_) * rz * (y * 2 - pc_height_) / pc_height_ / 2.0;
                KDL::Vector dir_H(rx, ry, rz);
                dir_H.Normalize();
                if (rays[x+y*pc_width_] != 1000000.0) {
                    pc_H.push_back(dir_H*rays[x+y*pc_width_]);
                }
            }
        }

        return pc_H;
*/
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sim_visible_pointcloud");
    TestNN pc;
    if (!pc.load()) {
        return -1;
    }
    pc.spin();
    return 0;
}


