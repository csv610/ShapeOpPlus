# ShapeOpPlus: Modernized Optimization for Geometry Processing

ShapeOp is a lightweight C++ library for static and dynamic geometry processing, originally developed by researchers at LGG EPFL. 

**Note on this Repository:** This version of the library is a **re-engineered and modernized fork** named **ShapeOpPlus**. While the core mathematical algorithms remain the work of the original authors (Sofien Bouaziz, Mario Deuss, Bailin Deng, et al.), this repository contains significant architectural updates and modernization work performed by **Chaman Singh Verma** to ensure compatibility with 2026-standard production environments.

---

## 🛠 Modernization & Re-engineering (Contributions)
This fork was created to address long-standing compatibility issues and to bring the library up to modern engineering standards. Key contributions include:

*   **Standard Upgrades:** Ported the entire codebase from C++11 to **C++17** to leverage modern standard library features and compiler optimizations.
*   **Dependency Modernization:** Re-engineered the build system (CMake 3.5+) to support **Eigen 5.0.1+** using semantic versioning. Fixed outdated detection logic that previously blocked modern Eigen support.
*   **Architectural Flexibility:** Introduced a **Pluggable Solver API** (`Solver::setSolver`). This allows users to hot-swap linear system solvers (**Cholesky, Conjugate Gradient, SOR, MINRES**) at runtime based on mesh scale and numerical requirements.
*   **Automated Verification:** Developed a comprehensive suite of **24 unit tests** using the **Google Test (GTest)** framework. Every geometric constraint and linear solver is now empirically verified for mathematical correctness.
*   **Automated CI/CD:** Integrated **GitHub Actions** to automatically build the project and run the full test suite on **Ubuntu** and **macOS** on every commit.
*   **Python 3 Support:** Fully modernized the SWIG-generated bindings and all simulation scripts for **Python 3.14+**.
*   **Geometric Extensions:** Implemented the `OrientationConstraint` (previously only theoretical in the documentation), enabling projection-based orientation control.
*   **Robustness Fixes:** Conducted a root-cause analysis and resolved numerical stability issues in the **MINRES** iterative solver (addressing Eigen-specific `NaN` generation).

### 📐 Functional Engineering Examples
A suite of **11 professional-grade CLI tools** was developed to serve as both functional verification and user "recipes" for common geometry processing tasks. All tools now accept external **OFF mesh files** as input.

1.  **Mesh Quality & Regularization:**
    *   `area_equalization`: Regularizes meshes so all triangles have identical surface area.
    *   `equilateral_regularization`: Forces triangles toward perfect equilateral shapes for high-quality meshing.
2.  **Architectural Rationalization:**
    *   `circular_quads`: Forces quadrilateral mesh faces to be cyclic (all 4 vertices on a single circle).
    *   `quad_planarization`: Planarizes curved quad meshes, a critical task for architectural glass and panel fabrication.
3.  **Advanced Mapping Suite:**
    *   A complete set of "As-X-As-Possible" mapping tools (**Similar, Rigid, Conformal, Area-Preserving, and Smooth**) to transfer local geometric properties between meshes with different boundaries.
4.  **Geometric Optimization:**
    *   `minimal_surface`: Computes the "soap film" minimal surface spanning an arbitrary 3D boundary.
    *   `fair_mesh_design`: Performs global surface smoothing by minimizing dihedral bending energy.
    *   `geodesic_path`: Computes the shortest path (geodesic) between points constrained to a 3D surface (e.g., a sphere).

---

## 📦 External Libraries & Versions

| Library | Version | Source | Notes |
|---------|---------|--------|-------|
| **Eigen3** | 5.0.1 | System (Homebrew) | Linear algebra |
| **GLFW** | 3.4 | System | Window/input handling |
| **GLEW** | 2.2.0 | System | OpenGL extensions |
| **OpenGL** | System | System | Graphics library |
| **OpenGP** | 1.0 | Bundled | Header-only |
| **OpenMP** | 5.2 | System (optional) | Parallelization |
| **Google Test** | 1.14.0 | FetchContent | Unit testing |
| **Python** | 3.14+ | System | Bindings |

### System Requirements
- **macOS**: Eigen3 available via `brew install eigen`
- **Ubuntu**: Eigen3 available via `sudo apt-get install libeigen3-dev`
- **Windows**: Eigen3 included via vcpkg or manual install

---

## 🚀 Quick Start

### 1. Build the Project
```bash
mkdir build && cd build
cmake ..
make -j4
```

### 2. Run the Unit Tests
```bash
./tests/unit_tests
```

### 3. CLI Usage
All examples are production-ready tools that accept OFF mesh files. They automatically identify and fix boundary nodes to preserve global geometry.
```bash
# General Optimization: ./example input.off [output.off]
./examples/minimal_surface boundary.off result.off

# Mapping/Transfer: ./example source.off target.off [output.off]
./examples/as_similar_as_possible_mapping source.off target.off result.off
```

---

## 📚 Documentation
Detailed technical documentation is available via **GitHub Pages**. Alternatively, you can generate it locally using Doxygen:
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
