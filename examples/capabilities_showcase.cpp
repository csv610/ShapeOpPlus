/**
 * ShapeOp Capabilities Showcase
 * 
 * This example demonstrates a variety of advanced constraints and features
 * available in the ShapeOp library.
 */

#include "Solver.h"
#include "Constraint.h"
#include "Force.h"
#include "LSSolver.h"
#include <iostream>
#include <vector>
#include <iomanip>

using namespace ShapeOp;

void print_points(const std::string& title, const Matrix3X& p) {
    std::cout << "--- " << title << " ---" << std::endl;
    for (int i = 0; i < p.cols(); ++i) {
        std::cout << "P" << i << ": " << p.col(i).transpose() << std::endl;
    }
    std::cout << std::endl;
}

// 1. Laplacian Smoothing
// Demonstrates the UniformLaplacianConstraint to smooth a mesh (or a line of points).
void demo_laplacian_smoothing() {
    Matrix3X p(3, 3);
    // 3 points in a line, but the middle point is offset (a "bump")
    p << 0.0, 1.0, 2.0,
         0.0, 1.0, 0.0, // Middle point is pulled up to y=1
         0.0, 0.0, 0.0;
         
    Solver s;
    s.setPoints(p);
    
    // Fix the endpoints using ClosenessConstraint with high weight
    s.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{0}, 100.0, p));
    s.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{2}, 100.0, p));
    
    // Laplacian constraint on the middle point to its neighbors
    // Format: {center_vertex_id, neighbor_1_id, neighbor_2_id, ...}
    std::vector<int> laplacian_ids = {1, 0, 2}; 
    s.addConstraint(std::make_shared<UniformLaplacianConstraint>(laplacian_ids, 1.0, p, false));
    
    s.initialize();
    s.solve(20);
    print_points("Laplacian Smoothing (Middle point's Y should flatten towards 0.0)", s.getPoints());
}

// 2. Volume Preservation
// Demonstrates the VolumeConstraint on a tetrahedron.
void demo_volume_preservation() {
    Matrix3X p(3, 4);
    // A simple tetrahedron
    p << 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;
         
    Solver s;
    s.setPoints(p);
    
    // Add volume constraint (tries to maintain the initial volume)
    std::vector<int> ids = {0, 1, 2, 3};
    s.addConstraint(std::make_shared<VolumeConstraint>(ids, 1.0, p));
    
    // Pull the apex (point 3) upwards
    auto pull = std::make_shared<ClosenessConstraint>(std::vector<int>{3}, 0.5, p);
    pull->setPosition(Vector3(0.0, 0.0, 2.0));
    s.addConstraint(pull);
    
    // Anchor the base (points 0, 1, 2)
    s.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{0}, 10.0, p));
    s.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{1}, 10.0, p));
    s.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{2}, 10.0, p));

    s.initialize();
    s.solve(50);
    print_points("Volume Preservation (Apex pulled up, base adjusts to maintain volume)", s.getPoints());
}

// 3. Rigid Shape Matching
// Demonstrates forcing a set of points to match a target shape rigidly.
void demo_rigid_matching() {
    Matrix3X p(3, 3);
    // Initial arbitrary points
    p << 0.0, 1.0, 0.0,
         0.0, 0.0, 1.0,
         0.0, 0.0, 0.0;
         
    Solver s;
    s.setPoints(p); 
    
    // The target shape we want to match (a larger right triangle)
    Matrix3X target(3, 3);
    target << 0.0, 2.0, 0.0,
              0.0, 0.0, 2.0,
              0.0, 0.0, 0.0;
              
    std::vector<int> ids = {0, 1, 2};
    // false = Rigid (no scaling allowed, only rotation/translation)
    // true = Similarity (scaling allowed)
    auto rigid = std::make_shared<SimilarityConstraint>(ids, 1.0, p, true); 
    
    std::vector<Matrix3X> shapes = {target};
    rigid->setShapes(shapes);
    s.addConstraint(rigid);
    
    // Soft anchor to keep it from flying away
    s.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{0}, 0.1, p));
    
    s.initialize();
    s.solve(20);
    print_points("Similarity Matching (Points should match the 2.0-scaled target shape)", s.getPoints());
}

// 4. Geometric Primitive Fitting
// Demonstrates forcing points to lie on a geometric primitive (a Circle).
void demo_geometric_fitting() {
    Matrix3X p(3, 4);
    // Points vaguely in a circle shape, but noisy
    p << 1.1, 0.0, -0.9, 0.0,
         0.0, 0.8, 0.0, -1.2,
         0.0, 0.0, 0.0, 0.0;
         
    Solver s;
    s.setPoints(p);
    
    std::vector<int> ids = {0, 1, 2, 3};
    s.addConstraint(std::make_shared<CircleConstraint>(ids, 1.0, p));
    
    s.initialize();
    s.solve(30);
    print_points("Circle Fitting (Points should adjust to form a perfect circle)", s.getPoints());
}

// 5. Alternate Solvers and Vertex Forces
// Demonstrates swapping the linear solver and applying forces to specific vertices.
void demo_custom_solver_and_forces() {
    Matrix3X p(3, 2);
    p << 0.0, 1.0, 
         0.0, 0.0, 
         0.0, 0.0;
    Solver s;
    s.setPoints(p);
    
    s.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{0}, 10.0, p));
    // An edge connecting them
    s.addConstraint(std::make_shared<EdgeStrainConstraint>(std::vector<int>{0, 1}, 1.0, p));
    
    // Apply a specific force to Vertex 1 (pulling it in +X)
    s.addForces(std::make_shared<VertexForce>(Vector3(5.0, 0.0, 0.0), 1));
    
    // Swap the solver to Conjugate Gradient (CG)
    s.setSolver(std::make_shared<CGSolver>()); 
    
    // Forces require dynamic initialization (or at least setting up the mass matrix)
    s.initialize(true, 1.0, 0.9, 0.1);
    s.solve(20);
    print_points("Custom Solver [CG] & VertexForce (P1 is pulled by a force in +X)", s.getPoints());
}

int main() {
    std::cout << "=======================================" << std::endl;
    std::cout << "   ShapeOp Capabilities Showcase       " << std::endl;
    std::cout << "=======================================\n" << std::endl;
    
    std::cout << std::fixed << std::setprecision(3);
    
    demo_laplacian_smoothing();
    demo_volume_preservation();
    demo_rigid_matching();
    demo_geometric_fitting();
    demo_custom_solver_and_forces();
    
    return 0;
}
