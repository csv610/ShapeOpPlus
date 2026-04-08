/**
 * ShapeOp Example: Equilateral Triangle Regularization
 * 
 * This example demonstrates how to:
 * 1. Read a triangle mesh from an OFF file.
 * 2. Create a "target" equilateral triangle shape.
 * 3. Use SimilarityConstraint to force every triangle in the mesh to 
 *    match the equilateral target (allowing for rotation, translation, and scaling).
 * 4. Fix boundary points to maintain the overall domain.
 */

#include "common.h"
#include "Constraint.h"
#include <iostream>
#include <vector>
#include <iomanip>
#include <cmath>

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
            std::cerr << "Error: Equilateral regularization example requires a triangle mesh." << std::endl;
            return 1;
        }
    }

    // 2. Define the Target Shape: A perfect equilateral triangle
    // Sides of length 1.0
    Matrix3X target_equilateral(3, 3);
    target_equilateral << 0.0, 1.0, 0.5,
                          0.0, 0.0, std::sqrt(3.0) / 2.0,
                          0.0, 0.0, 0.0;
    std::vector<Matrix3X> target_shapes = {target_equilateral};

    // 3. Setup Solver and Constraints
    Solver solver;
    solver.setPoints(mesh.vertices);

    // Add SimilarityConstraint to each triangle
    for (const auto& t : mesh.faces) {
        std::vector<int> ids = {t[0], t[1], t[2]};
        // allowScaling = true (Similarity), weight = 1.0
        auto sim_cons = std::make_shared<SimilarityConstraint>(ids, 1.0, mesh.vertices, true);
        sim_cons->setShapes(target_shapes);
        solver.addConstraint(sim_cons);
    }

    // 4. Anchor the boundary points strictly
    std::set<int> boundary = findBoundaryVertices(mesh);
    for (int idx : boundary) {
        solver.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{idx}, 1000.0, mesh.vertices));
    }

    // 5. Solve
    std::cout << "Regularizing mesh to equilateral triangles..." << std::endl;
    solver.initialize();
    solver.solve(100);

    mesh.vertices = solver.getPoints();
    writeOFF(output_file, mesh);
    std::cout << "Saved results to " << output_file << std::endl;

    return 0;
}
