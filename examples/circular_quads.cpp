/**
 * ShapeOp Example: Cyclic Quadrilateral Mesh Regularization
 * 
 * This example demonstrates how to force every quadrilateral element in a 
 * 2D mesh to be "cyclic" (meaning all 4 vertices lie on a common circle).
 * 
 * Concepts covered:
 * 1. Read a quadrilateral mesh from an OFF file.
 * 2. Applying CircleConstraint to sets of 4 vertices.
 * 3. Fixing boundaries to preserve the global mesh layout.
 */

#include "common.h"
#include "Constraint.h"
#include <iostream>
#include <vector>
#include <iomanip>

using namespace ShapeOp;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " input.off [output.off]" << std::endl;
        return 1;
    }

    std::string input_file = argv[1];
    std::string output_file = (argc > 2) ? argv[2] : "out.off";

    Mesh mesh;
    if (!readOFF(input_file, mesh)) {
        std::cerr << "Error: Could not read " << input_file << std::endl;
        return 1;
    }

    // Check if all faces are quads
    for (const auto& face : mesh.faces) {
        if (face.size() != 4) {
            std::cerr << "Error: Circular quads example requires a quadrilateral mesh." << std::endl;
            return 1;
        }
    }

    // 2. Setup Solver and Constraints
    Solver solver;
    solver.setPoints(mesh.vertices);

    // Add CircleConstraint to each quad
    Scalar weight = 1.0;
    for (const auto& face : mesh.faces) {
        std::vector<int> ids = {face[0], face[1], face[2], face[3]};
        solver.addConstraint(std::make_shared<CircleConstraint>(ids, weight, mesh.vertices));
    }

    // 3. Anchor the boundary points strictly (weight 1000.0)
    std::set<int> boundary = findBoundaryVertices(mesh);
    for (int idx : boundary) {
        solver.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{idx}, 1000.0, mesh.vertices));
    }

    // 4. Solve
    std::cout << "Regularizing quads to be cyclic (all 4 vertices on a circle)..." << std::endl;
    solver.initialize();
    solver.solve(100);

    mesh.vertices = solver.getPoints();
    writeOFF(output_file, mesh);
    std::cout << "Saved results to " << output_file << std::endl;

    return 0;
}
