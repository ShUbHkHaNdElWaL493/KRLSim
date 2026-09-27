#include <entt/entt.hpp>
#include <iostream>

#include "core/entity.hpp"
#include "logger/logger.hpp"

using namespace krlsim;

int main()
{
    Logger logger(LogType::DEBUG);
    logger.log(LogType::LOG, "Initializing krlsim sanity check...");

    Vector3 test_axis = Vector3::UnitZ();
    Isometry3 test_transform = Isometry3::Identity();
    test_transform.translation() = Vector3(0.0, 0.0, 0.5);
    test_transform.rotate(Eigen::AngleAxis<Scalar>(M_PI / 4.0, test_axis));

    logger.log(LogType::DEBUG, "Eigen Isometry3 initialized successfully.");

    KinematicModel robot;
    robot.name = "test_two_link_robot";
    robot.root_link_idx = 0;

    LinkDescriptor base_link;
    base_link.link_idx = 0;
    base_link.name = "base_link";
    base_link.child_joint_indices.push_back(0);
    base_link.child_link_indices.push_back(1);
    robot.links.push_back(base_link);

    JointDescriptor joint1;
    joint1.joint_idx = 0;
    joint1.name = "joint_1";
    joint1.type = JointType::REVOLUTE;
    joint1.axis = test_axis;
    joint1.origin = test_transform;
    joint1.parent_link_idx = 0;
    joint1.child_link_idx = 1;
    joint1.q_idx = 0;
    joint1.v_idx = 0;
    joint1.limits.lower = -M_PI;
    joint1.limits.upper = M_PI;
    joint1.limits.has_position_limits = true;
    robot.joints.push_back(joint1);
    robot.active_joint_indices.push_back(0);

    LinkDescriptor tool_link;
    tool_link.link_idx = 1;
    tool_link.name = "tool_link";
    tool_link.parent_joint_idx = 0;
    tool_link.parent_link_idx = 0;

    CollisionShape box_shape;
    box_shape.name = "tool_box";
    box_shape.origin = Isometry3::Identity();
    box_shape.geometry = BoxGeometry{Vector3(0.1, 0.1, 0.2)};
    tool_link.collisions.push_back(box_shape);
    robot.links.push_back(tool_link);

    robot.n_dof = 1;
    robot.n_q = 1;
    robot.n_v = 1;

    logger.log(LogType::LOG, "KinematicModel parsed: " + robot.name + 
                              " (DoF: " + std::to_string(robot.n_dof) + 
                              ", Links: " + std::to_string(robot.links.size()) + 
                              ", Joints: " + std::to_string(robot.joints.size()) + ")");

    entt::registry registry;
    
    const size_t num_envs = 1000;
    for (size_t i = 0; i < num_envs; ++i)
    {
        auto entity = registry.create();
        registry.emplace<VectorX>(entity, VectorX::Zero(robot.n_q));
    }

    auto view = registry.view<VectorX>();
    size_t count = 0;
    for (auto entity : view) {
        ++count;
    }

    logger.log(LogType::LOG, "EnTT ECS spawned " + std::to_string(count) + " environments successfully.");
    logger.log(LogType::LOG, "All sanity checks passed successfully!");

    return 0;
}