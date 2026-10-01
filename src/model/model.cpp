#include <fstream>
#include <iomanip>
#include <sstream>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include "model/model.hpp"

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
            int current_link_index = links.size();
            LinkDescriptor link_descriptor;
            link_descriptor.link_index = current_link_index;
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
                joint_descriptor.child_link_index = current_link_index;
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
                    self(self, child_link, current_link_index, child_joint);
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

    void Model::visualize(const std::string& prefix) const
    {
        if (links.empty()) { logger->log(LogType::WARNING, "Robot model not loaded."); return; }

        LogType log_type = (logger->getPriority() == LogType::DEBUG) ? LogType::DEBUG : LogType::LOG;

        std::stringstream ss;
        ss << prefix << "Model [name: " << name
           << ", links: " << links.size()
           << ", joints: " << joints.size() << "]\n";

        auto vector3_to_string = [](const Vector3& v)
        {
            std::stringstream vs;
            vs << std::fixed << std::setprecision(3);
            vs << v.x() << ", " << v.y() << ", " << v.z();
            return vs.str();
        };

        auto isometry3_to_string = [&](const Isometry3& i)
        {
            std::stringstream is;
            is << std::fixed << std::setprecision(3);
            Vector3 t = i.translation();
            Vector3 r = i.rotation().eulerAngles(0, 1, 2);
            is << vector3_to_string(t) << ", " << vector3_to_string(r);
            return is.str();
        };

        auto joint_type_to_string = [](JointType type)
        {
            switch(type) {
                case JointType::FIXED: return "FIXED";
                case JointType::REVOLUTE: return "REVOLUTE";
                case JointType::PRISMATIC: return "PRISMATIC";
                case JointType::CONTINUOUS: return "CONTINUOUS";
                case JointType::FLOATING: return "FLOATING";
                case JointType::PLANAR: return "PLANAR";
                default: return "UNKNOWN";
            }
        };

        auto collision_geometry_to_string = [&](const CollisionGeometry& collision)
        {
            std::stringstream gs;
            gs << std::fixed << std::setprecision(3);
            if (log_type == LogType::DEBUG) gs << "[Origin: " << isometry3_to_string(collision.origin) << "] ";
            gs << "[Geometry: ";
            if (std::holds_alternative<BoxGeometry>(collision.geometry)) {
                BoxGeometry b = std::get<BoxGeometry>(collision.geometry);
                gs << "Box(" << b.size.x() << ", " << b.size.y() << ", " << b.size.z() << ")";
            } else if (std::holds_alternative<SphereGeometry>(collision.geometry)) {
                gs << "Sphere(r=" << std::get<SphereGeometry>(collision.geometry).radius << ")";
            } else if (std::holds_alternative<CylinderGeometry>(collision.geometry)) {
                CylinderGeometry c = std::get<CylinderGeometry>(collision.geometry);
                gs << "Cylinder(r=" << c.radius << ", l=" << c.length << ")";
            } else if (std::holds_alternative<MeshGeometry>(collision.geometry)) {
                gs << "Mesh(scale= " << vector3_to_string(std::get<MeshGeometry>(collision.geometry).scale) << ")";
            } else {
                gs << "Unknown";
            }
            gs << "]";
            return gs.str();
        };

        auto get_child_joints = [&](int link_index)
        {
            std::vector<const JointDescriptor*> child_joints;
            for (const auto& j : joints) {
                if (j.parent_link_index == link_index) child_joints.push_back(&j);
            }
            return child_joints;
        };

        std::string base_indent(((log_type == LogType::DEBUG) ? 5 : 3) + 3 + prefix.length(), ' ');

        auto visualize_subtree = [&](auto& self, int current_link_index, const std::string& indent) -> void
        {
            const auto& link = links[current_link_index];
            auto child_joints = get_child_joints(current_link_index);
            
            size_t total_children = link.collisions.size() + child_joints.size();
            size_t child_index = 0;

            for (const auto& col : link.collisions)
            {
                bool is_last = (child_index == total_children - 1);
                std::string branch = is_last ? "└── " : "├── ";
                ss << indent << branch << "Collision: "
                << collision_geometry_to_string(col) << "\n";
                child_index++;
            }

            for (const auto* j : child_joints)
            {
                bool is_last = (child_index == total_children - 1);
                std::string branch = is_last ? "└── " : "├── ";
                std::string next_indent = indent + (is_last ? "    " : "│   ");

                ss << indent << branch << "Joint: " << j->name;
                if (log_type == LogType::DEBUG)
                {
                    ss << std::fixed << std::setprecision(3)
                       << " [ID: " << j->joint_index << "]"
                       << " [Origin: " << isometry3_to_string(j->origin) << "]"
                       << " [Type: " << joint_type_to_string(j->type) << "]"
                       << " [Axis: " << vector3_to_string(j->axis) << "]"
                       << " [Limits: lower=" << j->limits.lower << ", upper=" << j->limits.upper << ","
                       << " velocity=" << j->limits.velocity << ", effort=" << j->limits.effort << "]";
                }
                ss << "\n";

                if (j->child_link_index >= 0 && j->child_link_index < static_cast<int>(links.size())) {
                    ss << next_indent << "└── " << "Link: " << links[j->child_link_index].name;
                    if (log_type == LogType::DEBUG) ss << " [ID: " << j->child_link_index << "]";
                    ss << "\n";
                    self(self, j->child_link_index, next_indent + "    ");
                }
                child_index++;
            }
        };

        int root_link_index = 0;
        if (root_link_index >= 0)
        {
            ss << base_indent << "└── Link: " << links[root_link_index].name;
            if (log_type == LogType::DEBUG) ss << " [ID: " << root_link_index << "]";
            ss << "\n";
            visualize_subtree(visualize_subtree, root_link_index, base_indent + "    ");
        }

        logger->log(log_type, ss.str());
    }

}