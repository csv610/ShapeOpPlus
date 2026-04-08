/**
 * ShapeOp Example: Minimal Surface
 * 
 * This example demonstrates how to find a minimal surface (soap film)
 * given a fixed boundary. We use AreaConstraint with target area 0.0
 * on all triangles.
 */

#include "common.h"
#include "Constraint.h"
#include <iostream>
#include <vector>
#include <string>

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

    // Check if all faces are triangles
    for (const auto& face : mesh.faces) {
        if (face.size() != 3) {
            std::cerr << "Error: Minimal surface example requires a triangle mesh." << std::endl;
            return 1;
        }
    }

    Solver solver;
    solver.setPoints(mesh.vertices);

    // 2. Add AreaConstraint to all triangles with target area 0.0
    Scalar area_weight = 1.0;
    for (const auto& t : mesh.faces) {
        std::vector<int> ids = {t[0], t[1], t[2]};
        solver.addConstraint(std::make_shared<AreaConstraint>(ids, area_weight, mesh.vertices, 0.0, 0.0));
    }

    // 3. Fix boundary nodes using ClosenessConstraint
    Scalar fix_weight = 1000.0;
    std::set<int> boundary = findBoundaryVertices(mesh);
    for (int idx : boundary) {
        solver.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{idx}, fix_weight, mesh.vertices));
    }

    // 4. Initialize and solve
    solver.initialize();
    std::cout << "Solving for minimal surface..." << std::endl;
    solver.solve(100); 

    mesh.vertices = solver.getPoints();
    writeOFF(output_file, mesh);
    std::cout << "Result saved to: " << output_file << std::endl;

    return 0;
}
