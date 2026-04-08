/**
 * ShapeOp Example: As-Rigid-As-Possible Mapping
 * 
 * This example demonstrates how to transfer the "local rigid shape" of one mesh (Source)
 * to another mesh (Target) with a different boundary, while keeping the 
 * Target's boundary fixed.
 * 
 * Logic:
 * 1. Read Source and Target meshes (OFF format).
 * 2. Identify boundary vertices of the Target mesh.
 * 3. For each triangle in the Target mesh, apply a Rigid constraint
 *    (SimilarityConstraint with allowScaling=false) using the corresponding 
 *    triangle from the Source mesh as the "target shape".
 * 4. Fix boundary vertices of the Target mesh using high-weight ClosenessConstraints.
 * 5. Solve and save the resulting mesh to "result_rigid.off".
 */

#include "Solver.h"
#include "Constraint.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <set>

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
        for (int j = 0; j < nV_in_face; ++j) {
            file >> mesh.faces[i][j];
        }
    }
    return true;
}

// Minimal OFF Writer
void writeOFF(const std::string& filename, const Mesh& mesh) {
    std::ofstream file(filename);
    file << "OFF\n";
    file << mesh.vertices.cols() << " " << mesh.faces.size() << " 0\n";
    for (int i = 0; i < mesh.vertices.cols(); ++i) {
        file << mesh.vertices(0, i) << " " << mesh.vertices(1, i) << " " << mesh.vertices(2, i) << "\n";
    }
    for (const auto& face : mesh.faces) {
        file << face.size();
        for (int v : face) file << " " << v;
        file << "\n";
    }
}

// Identify boundary vertices (vertices belonging to edges used only once)
std::set<int> findBoundaryVertices(const Mesh& mesh) {
    std::map<std::pair<int, int>, int> edgeCount;
    for (const auto& face : mesh.faces) {
        for (size_t i = 0; i < face.size(); ++i) {
            int v1 = face[i];
            int v2 = face[(i + 1) % face.size()];
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

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " source.off target.off [output.off]" << std::endl;
        std::cout << "Note: Source and Target must have the same topology." << std::endl;
        return 1;
    }

    std::string sourceFile = argv[1];
    std::string targetFile = argv[2];
    std::string outputFile = (argc > 3) ? argv[3] : "result_rigid.off";

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

    // 1. Add Rigid constraints for each triangle
    // A Rigid constraint is a SimilarityConstraint with allowScaling = false.
    for (size_t i = 0; i < targetMesh.faces.size(); ++i) {
        const auto& face = targetMesh.faces[i];
        if (face.size() != 3) continue;

        Matrix3X source_tri(3, 3);
        source_tri.col(0) = sourceMesh.vertices.col(face[0]);
        source_tri.col(1) = sourceMesh.vertices.col(face[1]);
        source_tri.col(2) = sourceMesh.vertices.col(face[2]);

        std::vector<int> ids = {face[0], face[1], face[2]};
        // allowScaling = false -> Rigid
        auto rigid = std::make_shared<SimilarityConstraint>(ids, 1.0, targetMesh.vertices, false);
        rigid->setShapes({source_tri});
        solver.addConstraint(rigid);
    }

    // 2. Identify and fix boundary nodes strictly
    std::set<int> boundary = findBoundaryVertices(targetMesh);
    std::cout << "Found " << boundary.size() << " boundary vertices. Fixing them..." << std::endl;
    for (int v_id : boundary) {
        solver.addConstraint(std::make_shared<ClosenessConstraint>(std::vector<int>{v_id}, 1000.0, targetMesh.vertices));
    }

    // 3. Solve
    std::cout << "Optimizing for rigid mapping..." << std::endl;
    solver.initialize();
    solver.solve(100);

    // 4. Save result
    Mesh resultMesh = targetMesh;
    resultMesh.vertices = solver.getPoints();
    writeOFF(outputFile, resultMesh);
    std::cout << "Result saved to: " << outputFile << std::endl;

    return 0;
}
