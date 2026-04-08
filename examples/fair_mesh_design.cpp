/**
 * ShapeOp Example: Fair Mesh Design
 * 
 * This example demonstrates surface smoothing by minimizing dihedral angles
 * between adjacent triangles using BendingConstraint with target bend 0.0.
 */

#include "common.h"
#include "Constraint.h"
#include <iostream>
#include <vector>
#include <string>
#include <map>

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
            std::cerr << "Error: Fair mesh design example requires a triangle mesh." << std::endl;
            return 1;
        }
    }

    Solver solver;
    solver.setPoints(mesh.vertices);

    // 2. Add BendingConstraint to all adjacent triangle pairs
    std::map<std::pair<int, int>, std::vector<int>> edgeToOpposite;
    for (size_t f_idx = 0; f_idx < mesh.faces.size(); ++f_idx) {
        const auto& face = mesh.faces[f_idx];
        for (int i = 0; i < 3; ++i) {
            int v1 = face[i];
            int v2 = face[(i + 1) % 3];
            int v3 = face[(i + 2) % 3];
            if (v1 > v2) std::swap(v1, v2);
            edgeToOpposite[{v1, v2}].push_back(v3);
        }
    }

    Scalar bending_weight = 1.0;
    for (const auto& item : edgeToOpposite) {
        if (item.second.size() == 2) {
            int v1 = item.first.first;
            int v2 = item.first.second;
            int v3 = item.second[0];
            int v4 = item.second[1];
            solver.addConstraint(std::make_shared<BendingConstraint>(std::vector<int>{v1, v2, v3, v4}, bending_weight, mesh.vertices, 0.0, 0.0));
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
    std::cout << "Smoothing mesh..." << std::endl;
    solver.solve(100);

    mesh.vertices = solver.getPoints();
    writeOFF(output_file, mesh);
    std::cout << "Result saved to: " << output_file << std::endl;

    return 0;
}
