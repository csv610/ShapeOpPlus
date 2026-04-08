/**
 * ShapeOp Example: As-Smooth-As-Possible Mapping
 * 
 * This example demonstrates how to transfer the "Laplacian structure" 
 * (local smoothness/curvature) of one mesh (Source) to another mesh (Target) 
 * with a different boundary.
 * 
 * Logic:
 * 1. Read Source and Target meshes (OFF format).
 * 2. Identify boundary vertices of the Target mesh and fix them.
 * 3. For each vertex in the Target mesh:
 *    - Calculate its local Laplacian (centroid of its neighbors) in the Source mesh.
 *    - Apply a UniformLaplacianConstraint to the Target vertex, using the Source 
 *      relative displacement as the target.
 * 4. Solve and save to "result_smooth.off".
 */

#include "common.h"
#include "Constraint.h"

using namespace ShapeOp;

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " source.off target.off [output.off]" << std::endl;
        return 1;
    }

    std::string sourceFile = argv[1], targetFile = argv[2];
    std::string outputFile = (argc > 3) ? argv[3] : "result_smooth.off";

    Mesh sourceMesh, targetMesh;
    if (!readOFF(sourceFile, sourceMesh) || !readOFF(targetFile, targetMesh)) {
        std::cerr << "Error reading meshes." << std::endl;
        return 1;
    }

    Solver solver;
    solver.setPoints(targetMesh.vertices);

    // Build adjacency list for Laplacian
    std::vector<std::set<int>> adj(targetMesh.vertices.cols());
    for (const auto& face : targetMesh.faces) {
        for (size_t i = 0; i < face.size(); ++i) {
            int v1 = face[i];
            int v2 = face[(i + 1) % face.size()];
            adj[v1].insert(v2);
            adj[v2].insert(v1);
        }
    }

    // 1. Add Laplacian (Smoothness) constraints for each vertex
    for (int i = 0; i < (int)adj.size(); ++i) {
        if (adj[i].empty()) continue;

        std::vector<int> ids;
        ids.push_back(i); // Center vertex first
        for (int neighbor : adj[i]) ids.push_back(neighbor);

        // Laplacian constraint on TARGET mesh vertices
        // We use the SOURCE mesh to determine the "rest" Laplacian displacement
        // ShapeOp's LaplacianConstraint uses 'positions' in constructor to store the rest state
        auto lap = std::make_shared<UniformLaplacianConstraint>(ids, 1.0, sourceMesh.vertices, false);
        solver.addConstraint(lap);
    }

    // 2. Fix boundary nodes
    std::set<int> boundary = findBoundaryVertices(targetMesh);
    for (int v_id : boundary) {
        solver.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{v_id}, 1000.0, targetMesh.vertices));
    }

    // 3. Solve
    std::cout << "Optimizing for smooth (Laplacian-preserving) mapping..." << std::endl;
    solver.initialize();
    solver.solve(100);

    // 4. Save
    Mesh resultMesh = targetMesh;
    resultMesh.vertices = solver.getPoints();
    writeOFF(outputFile, resultMesh);
    std::cout << "Result saved to: " << outputFile << std::endl;

    return 0;
}
