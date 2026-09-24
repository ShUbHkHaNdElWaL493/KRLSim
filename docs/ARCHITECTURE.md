# System Architecture

To build this purely kinematic C++ simulation engine that can safely and efficiently feed a future RL wrapper, the architecture must focus strictly on state tracking, spatial math, and memory management.

Here are the core internal functionalities the simulator must possess:

## Scene and Kinematic Tree Parser
The engine needs a robust parser to ingest the scene description file (e.g., URDF, MJCF, or a custom format). It must extract:

* **Topology:** The parent-child hierarchy of links and joints.
* **Joint Constraints:** Degrees of freedom (DoF), axis of rotation/translation, and strict joint limits.
* **Geometry Data:** Pointers to primitive shapes (boxes, spheres) or complex 3D meshes used specifically for collision checking, distinct from visual meshes.

## State-Structure Separation (Memory Architecture)
To support multi-threading and GPU acceleration without memory duplication, the simulator must strictly separate immutable data from mutable data.

* **Immutable Structure:** The kinematic tree, mesh data, and environment layout remain in a shared, read-only memory space.
* **Mutable State:** The current joint positions, link poses, and dynamically moving obstacle poses are stored in contiguous memory arrays (often struct-of-arrays or SoA format). This allows multiple CPU threads or GPU kernels to process thousands of independent states simultaneously against the same read-only world structure.

## High-Performance Kinematic Solvers
Because the engine ignores forces and mass, the mathematical solvers are its primary engine.

* **Forward Kinematics (FK):** A highly optimized matrix multiplication pipeline to instantly compute the 3D poses of all links and the end-effector given a set of joint angles.
* **Kinematic Jacobian Computation:** Calculating the Jacobian matrix to map joint velocities to end-effector velocities, which is critical if the future RL wrapper will use velocity-level control.
* **Inverse Kinematics (IK) (Optional but standard):** A numerical solver (like Jacobian pseudo-inverse or Levenberg-Marquardt) to convert desired Cartesian coordinates back into joint angles.

## Hybrid CPU/GPU Collision and Geometry Pipeline
This is typically the most computationally expensive part of a kinematic simulator.

* **Broad-Phase Pruning:** Bounding Volume Hierarchies (BVH) or spatial hashing to quickly eliminate object pairs that are too far apart to collide.
* **Narrow-Phase Detection:** Exact mesh-to-mesh or primitive-to-primitive intersection tests (e.g., GJK/EPA algorithms) to confirm collisions.
* **Continuous Distance Queries:** Calculating the exact shortest distance between the robot and environmental geometries, rather than just returning a boolean true/false.
* **Compute Dispatcher:** A mechanism to detect hardware and route heavy batch collision queries to the GPU (via CUDA/Compute Shaders) while keeping single-thread or low-batch queries on the CPU.

## Thread-Safe Mutation API (The "Step" and "Reset" Mechanisms)
The C++ core must expose a strict, thread-safe internal API for the future wrapper to call.

* **State Injection:** Functions to instantaneously overwrite joint angles or obstacle positions (the "reset") without triggering any memory reallocation or file parsing.
* **Kinematic Integration:** A stepping function that takes velocity commands, applies them over a given time delta ($\Delta t$), enforces joint limits mathematically, and updates the state arrays.