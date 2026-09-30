#include "model/model.hpp"
#include <fstream>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

namespace krlsim
{

    static Isometry3 pose_to_eigen(const urdf::Pose& pose)
    {
        Isometry3 out = Isometry3::Identity();
        out.translation() << pose.position.x, pose.position.y, pose.position.z;
        Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
        out.linear() = q.normalized().toRotationMatrix();
        return out;
    }

    static JointType joint_to_enum(const int& type)
    {
        switch (type)
        {
            case urdf::Joint::REVOLUTE:   return JointType::REVOLUTE;
            case urdf::Joint::PRISMATIC:  return JointType::PRISMATIC;
            case urdf::Joint::CONTINUOUS: return JointType::CONTINUOUS;
            case urdf::Joint::FIXED:      return JointType::FIXED;
            case urdf::Joint::FLOATING:   return JointType::FLOATING;
            case urdf::Joint::PLANAR:     return JointType::PLANAR;
            default:                      return JointType::UNKNOWN;
        }
    }

    Model::Model(const std::string& name, std::shared_ptr<Logger> logger) : name(name), logger(std::move(logger))
    {}

    void Model::parseRobotDescription(const std::string& robot_description)
    {

        auto urdf_model = urdf::parseURDF(robot_description);
        if (!urdf_model) { logger->log(LogType::ERROR, "Failed to parse robot description."); return; }
        
        auto root_link = urdf_model->getRoot();
        if (!root_link) { logger->log(LogType::ERROR, "Robot description has no root link."); return; }

        links.clear();
        joints.clear();

        auto process_link = [&](auto& self, urdf::LinkConstSharedPtr urdf_link, int parent_link_idx, urdf::JointSharedPtr in_joint) -> void
        {
            int current_link_idx = links.size();
            LinkDescriptor link_descriptor;
            link_descriptor.link_index = current_link_idx;
            link_descriptor.name = urdf_link->name;

            std::vector<urdf::CollisionSharedPtr> cols;
            if (!urdf_link->collision_array.empty()) cols = urdf_link->collision_array;
            else if (urdf_link->collision) cols.push_back(urdf_link->collision);

            for (const auto& col : cols)
            {
                if (!col || !col->geometry) continue;
                CollisionGeometry collision_geometry;
                collision_geometry.origin = pose_to_eigen(col->origin);
                if (auto box = std::dynamic_pointer_cast<urdf::Box>(col->geometry))
                {
                    collision_geometry.geometry = BoxGeometry{Vector3(box->dim.x, box->dim.y, box->dim.z)};
                } else if (auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(col->geometry))
                {
                    collision_geometry.geometry = SphereGeometry{sphere->radius};
                } else if (auto cyl = std::dynamic_pointer_cast<urdf::Cylinder>(col->geometry))
                {
                    collision_geometry.geometry = CylinderGeometry{cyl->length, cyl->radius};
                } else if (auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(col->geometry))
                {
                    collision_geometry.geometry = MeshGeometry{mesh->filename, Vector3(mesh->scale.x, mesh->scale.y, mesh->scale.z)};
                }
                link_descriptor.collisions.push_back(collision_geometry);
            }
            links.push_back(link_descriptor);

            if (in_joint && parent_link_idx >= 0)
            {
                JointDescriptor joint_descriptor;
                joint_descriptor.joint_index = joints.size();
                joint_descriptor.name = in_joint->name;
                joint_descriptor.type = joint_to_enum(in_joint->type);
                joint_descriptor.origin = pose_to_eigen(in_joint->parent_to_joint_origin_transform);
                joint_descriptor.parent_link_index = parent_link_idx;
                joint_descriptor.child_link_index = current_link_idx;
                joint_descriptor.axis = Vector3(in_joint->axis.x, in_joint->axis.y, in_joint->axis.z);

                if (in_joint->limits)
                {
                    joint_descriptor.limits.lower = in_joint->limits->lower;
                    joint_descriptor.limits.upper = in_joint->limits->upper;
                    joint_descriptor.limits.velocity = in_joint->limits->velocity;
                    joint_descriptor.limits.effort = in_joint->limits->effort;
                }
                joints.push_back(joint_descriptor);
            }

            for (const auto& child_joint : urdf_link->child_joints)
            {
                auto child_link = urdf_model->getLink(child_joint->child_link_name);
                if (child_link)
                {
                    self(self, child_link, current_link_idx, child_joint);
                }
            }
        };

        process_link(process_link, root_link, -1, nullptr);
        logger->log(LogType::LOG, "Robot description parsed successfully.");
    }

    void Model::parseURDF(const std::string& urdf_file_path)
    {
        auto urdf_model = urdf::parseURDFFile(urdf_file_path);
        if (!urdf_model) { logger->log(LogType::ERROR, "Failed to parse URDF file: " + urdf_file_path); return; }
        
        std::ifstream urdf_file(urdf_file_path);
        if (!urdf_file.is_open()) { logger->log(LogType::ERROR, "Failed to open URDF file: " + urdf_file_path); return; }
        std::string robot_description((std::istreambuf_iterator<char>(urdf_file)), std::istreambuf_iterator<char>());
        
        parseRobotDescription(robot_description);
        logger->log(LogType::LOG, "URDF file parsed successfully.");
    }

    std::string Model::toJSON()
    {
        if (links.empty() && joints.empty()) { logger->log(LogType::WARNING, "Robot model not loaded."); return ""; }

        std::stringstream ss;
        ss << "{\n";
        ss << "\tname: " << name << ",\n";
        ss << "\tlinks: [\n";
        for (LinkDescriptor link_descriptor : links)
        {
            ss << "\t\t{\n";
            ss << "\t\t\tlink_index: " << link_descriptor.link_index << ",\n";
            ss << "\t\t\tname: " << link_descriptor.name << "\n";
            ss << "\t\t},\n";
        }
        ss << "\t],\n";
        ss << "\tjoints: [\n";
        for (JointDescriptor joint_descriptor : joints)
        {
            ss << "\t\t{\n";
            ss << "\t\t\tjoint_index: " << joint_descriptor.joint_index << ",\n";
            ss << "\t\t\tname: " << joint_descriptor.name << "\n";
            ss << "\t\t},\n";
        }
        ss << "\t]\n";
        ss << "}";

        logger->log(LogType::DEBUG, "Model description (JSON):\n" + ss.str());
        return ss.str();
    }

}