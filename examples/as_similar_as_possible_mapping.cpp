/**
 * ShapeOp Example: Shape Transfer (Similarity-based Mesh Deformation)
 * 
 * This example demonstrates how to transfer the "local shape" of one mesh (Source)
 * to another mesh (Target) with a different boundary, while keeping the 
 * Target's boundary fixed.
 * 
 * Logic:
 * 1. Read Source and Target meshes (OFF format).
 * 2. Identify boundary vertices of the Target mesh.
 * 3. For each triangle in the Target mesh, apply a SimilarityConstraint
 *    using the corresponding triangle from the Source mesh as the "target shape".
 * 4. Fix boundary vertices of the Target mesh.
 * 5. Solve and save the resulting mesh to "result.off".
 */

#include "common.h"
#include "Constraint.h"

using namespace ShapeOp;

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " source.off target.off [output.off]" << std::endl;
        std::cout << "Note: Source and Target must have the same topology (same faces/connectivity)." << std::endl;
        return 1;
    }

    std::string sourceFile = argv[1];
    std::string targetFile = argv[2];
    std::string outputFile = (argc > 3) ? argv[3] : "result.off";

    Mesh sourceMesh, targetMesh;
    if (!readOFF(sourceFile, sourceMesh)) {
        std::cerr << "Error reading source mesh: " << sourceFile << std::endl;
        return 1;
    }
    if (!readOFF(targetFile, targetMesh)) {
        std::cerr << "Error reading target mesh: " << targetFile << std::endl;
        return 1;
    }

    if (sourceMesh.vertices.cols() != targetMesh.vertices.cols() || sourceMesh.faces.size() != targetMesh.faces.size()) {
        std::cerr << "Error: Source and Target meshes must have the same number of vertices and faces." << std::endl;
        return 1;
    }

    Solver solver;
    solver.setPoints(targetMesh.vertices);

    // 1. Add Similarity constraints for each triangle
    // We use the shape from the Source mesh as the target for the Target mesh
    for (size_t i = 0; i < targetMesh.faces.size(); ++i) {
        const auto& face = targetMesh.faces[i];
        if (face.size() != 3) continue; // Only triangles supported for this simple example

        // The target shape is the triangle from the SOURCE mesh
        Matrix3X source_tri(3, 3);
        source_tri.col(0) = sourceMesh.vertices.col(face[0]);
        source_tri.col(1) = sourceMesh.vertices.col(face[1]);
        source_tri.col(2) = sourceMesh.vertices.col(face[2]);

        // Constraint on TARGET mesh vertices
        std::vector<int> ids = {face[0], face[1], face[2]};
        auto sim = std::make_shared<SimilarityConstraint>(ids, 1.0, targetMesh.vertices, true);
        sim->setShapes({source_tri});
        solver.addConstraint(sim);
    }

    // 2. Identify and fix boundary nodes of the Target mesh
    std::set<int> boundary = findBoundaryVertices(targetMesh);
    std::cout << "Found " << boundary.size() << " boundary vertices. Fixing them..." << std::endl;
    for (int v_id : boundary) {
        solver.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{v_id}, 1000.0, targetMesh.vertices));
    }

    // 3. Solve
    std::cout << "Optimizing internal vertices..." << std::endl;
    solver.initialize();
    solver.solve(100);

    // 4. Save result
    Mesh resultMesh = targetMesh;
    resultMesh.vertices = solver.getPoints();
    writeOFF(outputFile, resultMesh);
    std::cout << "Result saved to: " << outputFile << std::endl;

    return 0;
}
