# ShapeOp: Unified Optimization for Geometry Processing

ShapeOp is a lightweight C++ library for static and dynamic geometry processing, originally developed by researchers at LGG EPFL. 

**Note on this Repository:** This version of the library is a **re-engineered and modernized fork**. While the core mathematical algorithms remain the work of the original authors (Sofien Bouaziz, Mario Deuss, Bailin Deng, et al.), this repository contains significant architectural updates and modernization work performed by Chaman Singh Verma to ensure compatibility with 2026-standard development environments.

---

## 🛠 Modernization & Re-engineering
This fork was created to address long-standing compatibility issues and to bring the library up to modern engineering standards. Key updates include:

*   **Standard Upgrades:** Ported from C++11 to **C++17** to support modern compiler toolchains (AppleClang 21, GCC 14+).
*   **Dependency Modernization:** Re-engineered the build system (CMake 3.5+) to support **Eigen 5.0.1+** (using semantic versioning).
*   **Architectural Flexibility:** Introduced a **Pluggable Solver API** (`Solver::setSolver`). Users can now swap the underlying linear system solver (Cholesky, Conjugate Gradient, SOR, etc.) at runtime without re-compiling the core library.
*   **Automated Verification:** Implemented a comprehensive suite of **23 unit tests** using the **Google Test (GTest)** framework, covering all geometric constraints and solvers.
*   **Python 3 Support:** Fully modernized the SWIG-generated bindings and example scripts for **Python 3.14+**.
*   **Bug Fixes:** Resolved several legacy issues, including illegal internal header inclusions, missing `<cassert>` headers, and outdated Eigen detection logic.

---

## 🚀 Quick Start

### 1. Build the Project
```bash
mkdir build && cd build
cmake ..
make
```

### 2. Run the Unit Tests
```bash
./tests/unit_tests
```

### 3. Explore Examples
We have provided several new examples to showcase the library's capabilities:
*   **C++ Cloth Simulation:** `./examples/cloth_simulation` (Dynamic physics)
*   **C++ Capabilities Showcase:** `./examples/capabilities_showcase` (Advanced constraints)
*   **Python Catenary Curve:** `PYTHONPATH=libShapeOp/bindings/python python3 examples/catenary_curve.py`
*   **Triangle Area Equalization:** `./examples/area_equalization` (Mesh regularization)
*   **Equilateral Regularization:** `./examples/equilateral_regularization` (Mesh quality)
*   **Cyclic Quads:** `./examples/circular_quads` (Architectural geometry)
*   **As-Similar-As-Possible Mapping:** `./examples/as_similar_as_possible_mapping source.off target.off` (Shape transfer)
*   **As-Rigid-As-Possible Mapping:** `./examples/as_rigid_as_possible_mapping source.off target.off` (Rigid transfer)
*   **As-Conformal-As-Possible Mapping:** `./examples/as_conformal_as_possible_mapping source.off target.off` (Angle transfer)
*   **As-Area-Preserving Mapping:** `./examples/as_area_preserving_as_possible_mapping source.off target.off` (Area transfer)
*   **As-Smooth-As-Possible Mapping:** `./examples/as_smooth_as_possible_mapping source.off target.off` (Smoothness transfer)

---

## 📚 Documentation
For detailed technical documentation, please refer to the `doc/` directory. You can generate the HTML documentation using Doxygen:
```bash
cd doc
doxygen DoxyShapeOp
```

## 📜 Credits & License
The original ShapeOp library was a joint collaboration between researchers at LGG EPFL and others. This project is subject to the terms of the **Mozilla Public License v. 2.0**.

**Original Contributors:**
- ShapeOp Solver: Sofien Bouaziz, Mario Deuss, Bailin Deng
- Rhino/Kangaroo: Anders Holden Deleuran, Mario Deuss, Bailin Deng, Daniel Piker
- Project Management: Mario Deuss, Mark Pauly

**Re-engineering & Modernization:** 
Chaman Singh Verma
