#ifndef EXAMPLES_COMMON_H
#define EXAMPLES_COMMON_H

#include "Solver.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <algorithm>

namespace ShapeOp {

struct Mesh {
    Matrix3X vertices;
    std::vector<std::vector<int>> faces;
};

// Minimal OFF Reader
inline bool readOFF(const std::string& filename, Mesh& mesh) {
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
inline void writeOFF(const std::string& filename, const Mesh& mesh) {
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
inline std::set<int> findBoundaryVertices(const Mesh& mesh) {
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

} // namespace ShapeOp

#endif
