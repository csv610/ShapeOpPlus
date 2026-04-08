/**
 * ShapeOp Example: Geodesic Path
 * 
 * This example demonstrates how to find a geodesic path on a sphere.
 * We use SphereConstraint to keep points on a sphere and 
 * EdgeStrainConstraint with target length 0.0 to pull the rope tight.
 */

#include "common.h"
#include "Constraint.h"
#include <iostream>
#include <vector>
#include <string>
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

    Solver solver;
    solver.setPoints(mesh.vertices);

    // 2. Add SphereConstraint to keep ALL points on the same sphere
    std::vector<int> all_ids(mesh.vertices.cols());
    for (int i = 0; i < (int)all_ids.size(); ++i) all_ids[i] = i;
    solver.addConstraint(std::make_shared<SphereConstraint>(all_ids, 1.0, mesh.vertices));

    // 3. Add EdgeStrainConstraint with target length 0.0 to pull the rope tight
    Scalar edge_weight = 1.0;
    for (const auto& face : mesh.faces) {
        if (face.size() == 2) {
            solver.addConstraint(std::make_shared<EdgeStrainConstraint>(face, edge_weight, mesh.vertices, 0.0, 0.0));
        } else {
            for (size_t i = 0; i < face.size(); ++i) {
                std::vector<int> edge = {face[i], face[(i + 1) % face.size()]};
                solver.addConstraint(std::make_shared<EdgeStrainConstraint>(edge, edge_weight, mesh.vertices, 0.0, 0.0));
            }
        }
    }

    // 4. Fix boundary nodes (start and end of the path)
    std::set<int> boundary = findBoundaryVertices(mesh);
    for (int idx : boundary) {
        solver.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{idx}, 100.0, mesh.vertices));
    }

    // 5. Initialize and solve
    solver.initialize();
    std::cout << "Finding geodesic path..." << std::endl;
    solver.solve(100);

    mesh.vertices = solver.getPoints();
    writeOFF(output_file, mesh);
    std::cout << "Result saved to: " << output_file << std::endl;

    return 0;
}
