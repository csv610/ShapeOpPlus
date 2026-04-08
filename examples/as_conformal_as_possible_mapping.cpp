/**
 * ShapeOp Example: As-Conformal-As-Possible Mapping
 * 
 * This example demonstrates how to transfer the "conformal structure" (angles) 
 * of one mesh (Source) to another mesh (Target) with a different boundary, 
 * using explicit AngleConstraints.
 * 
 * Logic:
 * 1. Read Source and Target meshes (OFF format).
 * 2. Identify boundary vertices of the Target mesh and fix them.
 * 3. For each triangle in the Target mesh:
 *    - Calculate the 3 interior angles of the corresponding Source triangle.
 *    - Apply 3 AngleConstraints to the Target triangle, forcing its angles 
 *       to match the Source angles (minAngle = maxAngle = SourceAngle).
 * 4. Solve and save to "result_conformal.off".
 */

#include "common.h"
#include "Constraint.h"
#include <cmath>

using namespace ShapeOp;

// Helper to compute angle at p0 between p1 and p2
Scalar computeAngle(const Vector3& p0, const Vector3& p1, const Vector3& p2) {
    Vector3 v1 = (p1 - p0).normalized();
    Vector3 v2 = (p2 - p0).normalized();
    return std::acos(std::clamp(v1.dot(v2), -1.0, 1.0));
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " source.off target.off [output.off]" << std::endl;
        return 1;
    }

    std::string sourceFile = argv[1], targetFile = argv[2];
    std::string outputFile = (argc > 3) ? argv[3] : "result_conformal.off";

    Mesh sourceMesh, targetMesh;
    if (!readOFF(sourceFile, sourceMesh) || !readOFF(targetFile, targetMesh)) {
        std::cerr << "Error reading meshes." << std::endl;
        return 1;
    }

    Solver solver;
    solver.setPoints(targetMesh.vertices);

    // 1. Add Conformal (Angle) constraints for each triangle
    for (const auto& face : targetMesh.faces) {
        if (face.size() != 3) continue;

        // Vertices in Source
        Vector3 s0 = sourceMesh.vertices.col(face[0]);
        Vector3 s1 = sourceMesh.vertices.col(face[1]);
        Vector3 s2 = sourceMesh.vertices.col(face[2]);

        // Compute 3 angles from the Source triangle
        Scalar a0 = computeAngle(s0, s1, s2);
        Scalar a1 = computeAngle(s1, s0, s2);
        Scalar a2 = computeAngle(s2, s0, s1);

        // Apply AngleConstraints to the Target triangle
        // Weight 1.0. We set min=max to force the exact angle.
        solver.addConstraint(std::make_shared<AngleConstraint>(std::vector<int>{face[0], face[1], face[2]}, 1.0, targetMesh.vertices, a0, a0));
        solver.addConstraint(std::make_shared<AngleConstraint>(std::vector<int>{face[1], face[0], face[2]}, 1.0, targetMesh.vertices, a1, a1));
        solver.addConstraint(std::make_shared<AngleConstraint>(std::vector<int>{face[2], face[0], face[1]}, 1.0, targetMesh.vertices, a2, a2));
    }

    // 2. Fix boundary nodes
    std::set<int> boundary = findBoundaryVertices(targetMesh);
    for (int v_id : boundary) {
        solver.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{v_id}, 1000.0, targetMesh.vertices));
    }

    // 3. Solve
    std::cout << "Optimizing for conformal (angle-preserving) mapping..." << std::endl;
    solver.initialize();
    solver.solve(100);

    // 4. Save
    Mesh resultMesh = targetMesh;
    resultMesh.vertices = solver.getPoints();
    writeOFF(outputFile, resultMesh);
    std::cout << "Result saved to: " << outputFile << std::endl;

    return 0;
}
