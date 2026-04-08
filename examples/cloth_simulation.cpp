/**
 * ShapeOp Example: Cloth Simulation
 * 
 * This example demonstrates how to:
 * 1. Read a mesh from an OFF file.
 * 2. Add structural "Edge" constraints (springs) for all edges.
 * 3. Add "Closeness" constraints to fix boundary points in space.
 * 4. Apply external forces (Gravity).
 * 5. Run a dynamic (time-integrated) simulation.
 */

#include "common.h"
#include "Constraint.h"
#include "Force.h"
#include <iostream>
#include <iomanip>
#include <set>

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

    // 2. Add structural constraints (Edges)
    Scalar weight = 1.0;
    std::set<std::pair<int, int>> unique_edges;
    for (const auto& face : mesh.faces) {
        for (size_t i = 0; i < face.size(); ++i) {
            int v1 = face[i];
            int v2 = face[(i + 1) % face.size()];
            if (v1 > v2) std::swap(v1, v2);
            unique_edges.insert({v1, v2});
        }
    }

    for (const auto& edge : unique_edges) {
        std::vector<int> ids = {edge.first, edge.second};
        solver.addConstraint(std::make_shared<EdgeStrainConstraint>(ids, weight, mesh.vertices));
    }

    // 3. Fix the boundary nodes
    std::set<int> boundary = findBoundaryVertices(mesh);
    for (int idx : boundary) {
        solver.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{idx}, 100.0, mesh.vertices));
    }

    // 4. Add Gravity (pointing in -Z direction)
    solver.addForces(std::make_shared<GravityForce>(Vector3(0.0, 0.0, -9.81)));

    // 5. Initialize the dynamic solver
    // Parameters: dynamic=true, masses=1.0, damping=0.95, timestep=0.1
    solver.initialize(true, 1.0, 0.95, 0.1);

    std::cout << "Starting Simulation (10 steps)..." << std::endl;
    for (int i = 0; i < 10; ++i) {
        solver.solve(10);
    }

    mesh.vertices = solver.getPoints();
    writeOFF(output_file, mesh);
    std::cout << "Saved results to " << output_file << std::endl;

    return 0;
}
