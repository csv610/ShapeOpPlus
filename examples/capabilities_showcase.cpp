/**
 * ShapeOp Capabilities Showcase (Refactored)
 * 
 * This example demonstrates the core optimization capability of the ShapeOp library:
 * Laplacian smoothing on an arbitrary input mesh.
 */

#include "Solver.h"
#include "Constraint.h"
#include "common.h"
#include <iostream>
#include <vector>
#include <iomanip>

using namespace ShapeOp;

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " input.off [output.off]" << std::endl;
        return 1;
    }

    std::string inputFile = argv[1];
    std::string outputFile = (argc > 2) ? argv[2] : "showcase_result.off";

    Mesh mesh;
    if (!readOFF(inputFile, mesh)) {
        std::cerr << "Error reading mesh: " << inputFile << std::endl;
        return 1;
    }

    Solver s;
    s.setPoints(mesh.vertices);
    
    // 1. Identify and fix boundaries
    std::set<int> boundary = findBoundaryVertices(mesh);
    for (int v_id : boundary) {
        s.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{v_id}, 1000.0, mesh.vertices));
    }

    // 2. Build adjacency for Laplacian smoothing
    std::vector<std::set<int>> adj(mesh.vertices.cols());
    for (const auto& face : mesh.faces) {
        for (size_t i = 0; i < face.size(); ++i) {
            int v1 = face[i];
            int v2 = face[(i + 1) % face.size()];
            adj[v1].insert(v2);
            adj[v2].insert(v1);
        }
    }

    // 3. Add Laplacian constraints
    for (int i = 0; i < (int)adj.size(); ++i) {
        if (adj[i].empty()) continue;
        std::vector<int> ids = {i};
        for (int neighbor : adj[i]) ids.push_back(neighbor);
        s.addConstraint(std::make_shared<UniformLaplacianConstraint>(ids, 1.0, mesh.vertices, false));
    }
    
    std::cout << "Running Laplacian Smoothing on " << inputFile << "..." << std::endl;
    s.initialize();
    s.solve(20);
    
    mesh.vertices = s.getPoints();
    writeOFF(outputFile, mesh);
    std::cout << "Result saved to: " << outputFile << std::endl;

    return 0;
}
