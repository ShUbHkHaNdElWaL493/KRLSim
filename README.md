# KRLSim

A lightweight, purely kinematic physics and collision engine designed as a high-performance backend for Reinforcement Learning. KRLSim is engineered to decouple simulation from training loops, exposing a generic, parallelized C++ core that external RL wrappers can consume natively or via Python.

---

## Foundation

C++ Core + Nanobind forms the foundation. By isolating the kinematics and collision logic from the Gym/RL abstraction, KRLSim remains agnostic to the training framework. It leverages zero-copy memory exchange (DLPack) via nanobind, allowing PyTorch or JAX to read simulation states directly from CPU/GPU memory without serialization overhead.

---

## Core Libraries

### Kinematics & Geometry

* Pinocchio: Used strictly for spatial algebra, Forward Kinematics (FK), and Jacobians. We bypass its rigid-body dynamics solvers entirely to maintain maximum step speed.
* Eigen3: Core linear algebra backend for matrix-vector operations.

### Collision Detection

* Coal (formerly HPP-FCL): Evaluates distance queries and continuous collision detection (CCD) between primitive bounding volumes (capsules, spheres) attached to the kinematic chain.

### Parallelism & Memory

* CPU / GPU Concurrency: OpenMP handles CPU thread-pool parallelization for independent environments. If scaling to GPU, batched state updates utilize custom CUDA kernels.
* Language Bridge: nanobind provides the C++-to-Python bindings, utilizing DLPack to pass environment states as raw tensors directly to the RL wrapper.

---

## System Architecture

KRLSim acts as the middle-layer provider. The RL wrapper handles the Gymnasium API, while KRLSim strictly handles vectorized state progression and collision boolean returns.

```
+-------------------------------------------------------------------------------+
|                              External RL Wrapper                              |
|           (Handles Gymnasium API, Reward Logic, Policy Extraction)            |
+-------------------------------------------------------------------------------+
                                        | (DLPack Zero-Copy Tensors)
+-------------------------------------------------------------------------------+
|                         KRLSim Python Bindings (nanobind)                     |
|                   Exposes Batched `step_kinematics()` API                     |
+-------------------------------------------------------------------------------+
                                        | (C-ABI)
+-------------------------------------------------------------------------------+
|                       Parallel Environment Coordinator                        |
|        CPU Mode: OpenMP Thread Pool   |   GPU Mode: Batched CUDA Dispatch     |
+-------------------------------------------------------------------------------+
                                        |
+-------------------------------------------------------------------------------+
|                               Core C++ Engine                                 |
|   +-----------------------+   +-------------------+   +--------------------+  |
|   |  URDF / Scene Parser  |   | Pinocchio (FK/IK) |   | Coal/FCL Collision |  |
|   +-----------------------+   +-------------------+   +--------------------+  |
+-------------------------------------------------------------------------------+
```

---

## Proposed Directory Structure

```
krlsim/
├── CMakeLists.txt                  # Root build configuration for C++ & bindings
├── pyproject.toml                  # Python package configuration (scikit-build-core)
├── README\.md
├── docker/                         
│   ├── Dockerfile.cpu
│   └── Dockerfile.gpu
├── cmake/                          
│   └── Dependencies.cmake
├── src/                            # Core C++ source code
│   ├── engine/                     
│   │   ├── kinematics.hpp
│   │   ├── kinematics.cpp          # Pinocchio FK and Jacobian wrappers
│   │   ├── collision.hpp
│   │   ├── collision.cpp           # Coal distance and collision checks
│   │   ├── state_manager.hpp       # Handles flat tensor layouts for DLPack
│   │   └── state_manager.cpp
│   ├── parallel/                   
│   │   ├── cpu_batcher.hpp         # OpenMP environment stepping
│   │   ├── cpu_batcher.cpp
│   │   └── cuda_batcher.cu         # Optional GPU batched FK updates
│   └── bindings/                   
│       └── python_bindings.cpp     # nanobind interface logic
├── include/                        # Public C++ headers
│   └── krlsim/
│       ├── engine.hpp
│       └── types.hpp
├── python/                         # Python library
│   └── krlsim/
│       ├── \_\_init\_\_.py
│       ├── simulator\.py            # Single instance wrapper
│       └── batched_simulator.py    # Multi-instance tensor wrapper
└── tests/                          
    ├── cpp/                        # Catch2/GTest for C++ math & collision validation
    │   └── test_kinematics.cpp
    └── python/                     # Pytest for nanobind tensor exchange
        └── test_tensor_exchange.py
```