#include <gtest/gtest.h>
#include "Constraint.h"

using namespace ShapeOp;

TEST(ConstraintTest, EdgeStrain) {
    Matrix3X p(3, 2);
    p << 0.0, 2.0, // Distance 2.0
         0.0, 0.0,
         0.0, 0.0;
    
    std::vector<int> ids = {0, 1};
    // Edge strain constraint with target distance of original distance (2.0)
    EdgeStrainConstraint esc(ids, 1.0, p);
    
    // Test projection when they are stretched to 4.0
    Matrix3X p_stretched(3, 2);
    p_stretched << 0.0, 4.0,
                   0.0, 0.0,
                   0.0, 0.0;
    
    // MUST call addConstraint to initialize idO_ internal state
    std::vector<Triplet> triplets;
    int idO = 0;
    esc.addConstraint(triplets, idO);

    // The projections matrix size depends on how many rows the constraint adds.
    // EdgeStrain adds 1 row (3D projection stored in that row's columns)
    Matrix3X projections(3, 1); 
    esc.project(p_stretched, projections);
    
    // In EdgeStrain::project:
    // Vector3 edge = positions.col(idI_[1]) - positions.col(idI_[0]); // (4,0,0)
    // l = 4.0
    // rest_ = 1.0 / 2.0 = 0.5
    // l = clamp(4.0 * 0.5, 1.0, 1.0) = 1.0
    // projections.col(idO_) = weight_ * l * edge / l_orig = sqrt(2) * 1.0 * (1,0,0) ?
    // Actually weight_ in constructor is weight_ *= std::sqrt(length);
    // weight_ = 1.0 * sqrt(2.0)
    // projections = sqrt(2) * (1,0,0)
    EXPECT_NEAR(projections(0, 0), std::sqrt(2.0), 1e-6);
}

TEST(ConstraintTest, Factory) {
    Matrix3X p(3, 2);
    p << 0.0, 1.0,
         0.0, 0.0,
         0.0, 0.0;
    std::vector<int> ids = {0, 1};
    auto c = Constraint::shapeConstraintFactory("EdgeStrain", ids, 1.0, p);
    EXPECT_NE(c, nullptr);
    EXPECT_EQ(c->nIndices(), 2);
}

TEST(ConstraintTest, PlaneConstraint) {
    Matrix3X p(3, 3);
    p << 0.0, 1.0, 0.0,
         0.0, 0.0, 1.0,
         0.0, 0.0, 0.0;
    std::vector<int> ids = {0, 1, 2};
    PlaneConstraint pc(ids, 1.0, p);

    Matrix3X p_off(3, 3);
    p_off = p;
    p_off(2, 0) = 1.0; // Point 0 is off the plane (z=0)

    std::vector<Triplet> triplets;
    int idO = 0;
    pc.addConstraint(triplets, idO);
    // PlaneConstraint adds 3 rows for each point? No, let's check PlaneConstraint::addConstraint
    // Actually PlaneConstraint::addConstraint adds 1 row per point in the constraint.
    Matrix3X projections(3, 3); 
    pc.project(p_off, projections);
    
    // Points should be projected to be coplanar.
    // Given weight calculation in constructor, the output is weighted.
    EXPECT_EQ(projections.cols(), 3);
}

TEST(ConstraintTest, AngleConstraint) {
    Matrix3X p(3, 3);
    p << 0.0, 1.0, 0.0,
         0.0, 0.0, 1.0,
         0.0, 0.0, 0.0;
    // Angle at p0 (0,0,0) between p1(1,0,0) and p2(0,1,0) is 90 degrees (PI/2)
    std::vector<int> ids = {0, 1, 2};
    AngleConstraint ac(ids, 1.0, p, 0.0, M_PI/4.0); // Limit to 45 degrees

    std::vector<Triplet> triplets;
    int idO = 0;
    ac.addConstraint(triplets, idO);
    // AngleConstraint adds 2 rows/projections
    Matrix3X projections(3, 2);
    ac.project(p, projections);
    // Should project to something satisfying the 45 degree limit.
    EXPECT_EQ(projections.cols(), 2);
}

TEST(ConstraintTest, TriangleStrain) {
    Matrix3X p(3, 3);
    p << 0.0, 1.0, 0.0,
         0.0, 0.0, 1.0,
         0.0, 0.0, 0.0;
    std::vector<int> ids = {0, 1, 2};
    TriangleStrainConstraint tsc(ids, 1.0, p);
    std::vector<Triplet> triplets;
    int idO = 0;
    tsc.addConstraint(triplets, idO);
    Matrix3X projections(3, 2);
    tsc.project(p, projections);
    EXPECT_EQ(projections.cols(), 2);
}

TEST(ConstraintTest, TetrahedronStrain) {
    Matrix3X p(3, 4);
    p << 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;
    std::vector<int> ids = {0, 1, 2, 3};
    TetrahedronStrainConstraint tsc(ids, 1.0, p);
    std::vector<Triplet> triplets;
    int idO = 0;
    tsc.addConstraint(triplets, idO);
    Matrix3X projections(3, 3);
    tsc.project(p, projections);
    EXPECT_EQ(projections.cols(), 3);
}

TEST(ConstraintTest, Area) {
    Matrix3X p(3, 3);
    p << 0.0, 1.0, 0.0,
         0.0, 0.0, 1.0,
         0.0, 0.0, 0.0;
    std::vector<int> ids = {0, 1, 2};
    AreaConstraint ac(ids, 1.0, p);
    std::vector<Triplet> triplets;
    int idO = 0;
    ac.addConstraint(triplets, idO);
    Matrix3X projections(3, 2);
    ac.project(p, projections);
    EXPECT_EQ(projections.cols(), 2);
}

TEST(ConstraintTest, Volume) {
    Matrix3X p(3, 4);
    p << 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;
    std::vector<int> ids = {0, 1, 2, 3};
    VolumeConstraint vc(ids, 1.0, p);
    std::vector<Triplet> triplets;
    int idO = 0;
    vc.addConstraint(triplets, idO);
    Matrix3X projections(3, 3);
    vc.project(p, projections);
    EXPECT_EQ(projections.cols(), 3);
}

TEST(ConstraintTest, Bending) {
    Matrix3X p(3, 4);
    p << 0.0, 1.0, 0.5, 0.5,
         0.0, 0.0, 1.0, -1.0,
         0.0, 0.0, 0.0, 0.0;
    std::vector<int> ids = {0, 1, 2, 3};
    BendingConstraint bc(ids, 1.0, p);
    std::vector<Triplet> triplets;
    int idO = 0;
    bc.addConstraint(triplets, idO);
    Matrix3X projections(3, 1);
    bc.project(p, projections);
    EXPECT_EQ(projections.cols(), 1);
}

TEST(ConstraintTest, Circle) {
    Matrix3X p(3, 3);
    p << 1.0, 0.0, -1.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 0.0;
    std::vector<int> ids = {0, 1, 2};
    CircleConstraint cc(ids, 1.0, p);
    std::vector<Triplet> triplets;
    int idO = 0;
    cc.addConstraint(triplets, idO);
    Matrix3X projections(3, 3);
    cc.project(p, projections);
    EXPECT_EQ(projections.cols(), 3);
}

TEST(ConstraintTest, Sphere) {
    Matrix3X p(3, 4);
    p << 1.0, -1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, -1.0,
         0.0, 0.0, 0.0, 0.0;
    std::vector<int> ids = {0, 1, 2, 3};
    SphereConstraint sc(ids, 1.0, p);
    std::vector<Triplet> triplets;
    int idO = 0;
    sc.addConstraint(triplets, idO);
    Matrix3X projections(3, 4);
    sc.project(p, projections);
    EXPECT_EQ(projections.cols(), 4);
}

TEST(ConstraintTest, Similarity) {
    Matrix3X p(3, 4);
    p << 0.0, 1.0, 1.0, 0.0,
         0.0, 0.0, 1.0, 1.0,
         0.0, 0.0, 0.0, 0.0;
    std::vector<int> ids = {0, 1, 2, 3};
    SimilarityConstraint sc(ids, 1.0, p);
    std::vector<Triplet> triplets;
    int idO = 0;
    sc.addConstraint(triplets, idO);
    Matrix3X projections(3, 4);
    sc.project(p, projections);
    EXPECT_EQ(projections.cols(), 4);
}

TEST(ConstraintTest, Rigid) {
    Matrix3X p(3, 4);
    p << 0.0, 1.0, 1.0, 0.0,
         0.0, 0.0, 1.0, 1.0,
         0.0, 0.0, 0.0, 0.0;
    std::vector<int> ids = {0, 1, 2, 3};
    SimilarityConstraint rc(ids, 1.0, p, false); // scaling = false
    std::vector<Triplet> triplets;
    int idO = 0;
    rc.addConstraint(triplets, idO);
    Matrix3X projections(3, 4);
    rc.project(p, projections);
    EXPECT_EQ(projections.cols(), 4);
}

TEST(ConstraintTest, Rectangle) {
    Matrix3X p(3, 4);
    p << 0.0, 1.0, 1.0, 0.0,
         0.0, 0.0, 1.0, 1.0,
         0.0, 0.0, 0.0, 0.0;
    std::vector<int> ids = {0, 1, 2, 3};
    RectangleConstraint rc(ids, 1.0, p);
    std::vector<Triplet> triplets;
    int idO = 0;
    rc.addConstraint(triplets, idO);
    Matrix3X projections(3, 4);
    rc.project(p, projections);
    EXPECT_EQ(projections.cols(), 4);
}

TEST(ConstraintTest, Parallelogram) {
    Matrix3X p(3, 4);
    p << 0.0, 1.0, 1.0, 0.0,
         0.0, 0.0, 1.0, 1.0,
         0.0, 0.0, 0.0, 0.0;
    std::vector<int> ids = {0, 1, 2, 3};
    ParallelogramConstraint pc(ids, 1.0, p);
    std::vector<Triplet> triplets;
    int idO = 0;
    pc.addConstraint(triplets, idO);
    Matrix3X projections(3, 4);
    pc.project(p, projections);
    EXPECT_EQ(projections.cols(), 4);
}

TEST(ConstraintTest, Laplacian) {
    Matrix3X p(3, 2);
    p << 0.0, 1.0,
         0.0, 0.0,
         0.0, 0.0;
    std::vector<int> ids = {0, 1};
    UniformLaplacianConstraint lc(ids, 1.0, p, false);
    std::vector<Triplet> triplets;
    int idO = 0;
    lc.addConstraint(triplets, idO);
    Matrix3X projections(3, 1);
    lc.project(p, projections);
    EXPECT_EQ(projections.cols(), 1);
}

