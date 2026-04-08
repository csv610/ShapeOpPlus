/**
 * ShapeOp Example: Quad Planarization
 * 
 * This example demonstrates how to force all faces of a quadrilateral mesh
 * to be planar using PlaneConstraint.
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

    Solver solver;
    solver.setPoints(mesh.vertices);

    // 2. Add PlaneConstraint to every face
    Scalar plane_weight = 10.0;
    for (const auto& face : mesh.faces) {
        if (face.size() >= 3) {
            solver.addConstraint(std::make_shared<PlaneConstraint>(face, plane_weight, mesh.vertices));
        }
    }

    // 3. Fix boundary nodes
    Scalar fix_weight = 1000.0;
    std::set<int> boundary = findBoundaryVertices(mesh);
    for (int idx : boundary) {
        solver.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{idx}, fix_weight, mesh.vertices));
    }

    // 4. Initialize and solve
    solver.initialize();
    std::cout << "Planarizing mesh..." << std::endl;
    solver.solve(500);

    mesh.vertices = solver.getPoints();
    writeOFF(output_file, mesh);
    std::cout << "Result saved to: " << output_file << std::endl;

    return 0;
}
