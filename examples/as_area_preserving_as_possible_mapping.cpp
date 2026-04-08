/**
 * ShapeOp Example: As-Area-Preserving-As-Possible Mapping
 * 
 * This example demonstrates how to transfer the "local areas" 
 * of one mesh (Source) to another mesh (Target) with a different boundary.
 * 
 * Logic:
 * 1. Read Source and Target meshes (OFF format).
 * 2. Identify boundary vertices of the Target mesh and fix them.
 * 3. For each triangle in the Target mesh:
 *    - Calculate the area of the corresponding Source triangle.
 *    - Apply an AreaConstraint to the Target triangle, forcing its area 
 *      to match the Source area.
 * 4. Solve and save to "result_area.off".
 */

#include "common.h"
#include "Constraint.h"
#include <cmath>

using namespace ShapeOp;

// Helper to compute triangle area
Scalar computeArea(const Vector3& p0, const Vector3& p1, const Vector3& p2) {
    Vector3 v1 = p1 - p0;
    Vector3 v2 = p2 - p0;
    return 0.5 * v1.cross(v2).norm();
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " source.off target.off [output.off]" << std::endl;
        return 1;
    }

    std::string sourceFile = argv[1], targetFile = argv[2];
    std::string outputFile = (argc > 3) ? argv[3] : "result_area.off";

    Mesh sourceMesh, targetMesh;
    if (!readOFF(sourceFile, sourceMesh) || !readOFF(targetFile, targetMesh)) {
        std::cerr << "Error reading meshes." << std::endl;
        return 1;
    }

    Solver solver;
    solver.setPoints(targetMesh.vertices);

    // 1. Add Area Preservation constraints for each triangle
    for (const auto& face : targetMesh.faces) {
        if (face.size() != 3) continue;

        // Area in Source
        Scalar sourceArea = computeArea(sourceMesh.vertices.col(face[0]), 
                                        sourceMesh.vertices.col(face[1]), 
                                        sourceMesh.vertices.col(face[2]));
        
        // Initial Area in Target (rest state for the constraint)
        Scalar targetInitArea = computeArea(targetMesh.vertices.col(face[0]), 
                                            targetMesh.vertices.col(face[1]), 
                                            targetMesh.vertices.col(face[2]));

        // Target ratio
        Scalar target_ratio = sourceArea / targetInitArea;

        // Apply AreaConstraint to the Target triangle
        std::vector<int> ids = {face[0], face[1], face[2]};
        solver.addConstraint(std::make_shared<AreaConstraint>(ids, 1.0, targetMesh.vertices, target_ratio, target_ratio));
    }

    // 2. Fix boundary nodes
    std::set<int> boundary = findBoundaryVertices(targetMesh);
    for (int v_id : boundary) {
        solver.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{v_id}, 1000.0, targetMesh.vertices));
    }

    // 3. Solve
    std::cout << "Optimizing for area-preserving mapping..." << std::endl;
    solver.initialize();
    solver.solve(100);

    // 4. Save
    Mesh resultMesh = targetMesh;
    resultMesh.vertices = solver.getPoints();
    writeOFF(outputFile, resultMesh);
    std::cout << "Result saved to: " << outputFile << std::endl;

    return 0;
}
