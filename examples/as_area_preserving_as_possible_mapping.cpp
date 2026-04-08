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

#include "Solver.h"
#include "Constraint.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <cmath>

using namespace ShapeOp;

struct Mesh {
    Matrix3X vertices;
    std::vector<std::vector<int>> faces;
};

// Minimal OFF Reader
bool readOFF(const std::string& filename, Mesh& mesh) {
    std::ifstream file(filename);
    if (!file.is_open()) return false;
    std::string header;
    file >> header;
    if (header != "OFF") return false;
    int nV, nF, nE;
    file >> nV >> nF >> nE;
    mesh.vertices.resize(3, nV);
    for (int i = 0; i < nV; ++i) {
        file >> mesh.vertices(0, i) >> mesh.vertices(1, i) >> mesh.vertices(2, i);
    }
    mesh.faces.resize(nF);
    for (int i = 0; i < nF; ++i) {
        int nV_in_face;
        file >> nV_in_face;
        mesh.faces[i].resize(nV_in_face);
        for (int j = 0; j < nV_in_face; ++j) file >> mesh.faces[i][j];
    }
    return true;
}

// Minimal OFF Writer
void writeOFF(const std::string& filename, const Mesh& mesh) {
    std::ofstream file(filename);
    file << "OFF\n" << mesh.vertices.cols() << " " << mesh.faces.size() << " 0\n";
    for (int i = 0; i < mesh.vertices.cols(); ++i) {
        file << mesh.vertices(0, i) << " " << mesh.vertices(1, i) << " " << mesh.vertices(2, i) << "\n";
    }
    for (const auto& face : mesh.faces) {
        file << face.size();
        for (int v : face) file << " " << v;
        file << "\n";
    }
}

// Identify boundary vertices
std::set<int> findBoundaryVertices(const Mesh& mesh) {
    std::map<std::pair<int, int>, int> edgeCount;
    for (const auto& face : mesh.faces) {
        for (size_t i = 0; i < face.size(); ++i) {
            int v1 = face[i], v2 = face[(i + 1) % face.size()];
            if (v1 > v2) std::swap(v1, v2);
            edgeCount[{v1, v2}]++;
        }
    }
    std::set<int> boundary;
    for (const auto& edge : edgeCount) {
        if (edge.second == 1) {
            boundary.insert(edge.first.first);
            boundary.insert(edge.first.second);
        }
    }
    return boundary;
}

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
