#include <Eigen/Sparse>
#include <unsupported/Eigen/IterativeSolvers>
#include <iostream>

using namespace Eigen;

int main() {
    SparseMatrix<double> A(3, 3);
    A.insert(0, 0) = 1.0;
    A.insert(1, 1) = 1.0;
    A.insert(2, 2) = 1.0;
    
    SparseMatrix<double> N = A.transpose() * A;

    MINRES<SparseMatrix<double>, Lower, IdentityPreconditioner> minres;
    minres.compute(N);

    VectorXd b(3);
    b << 1.0, 1.0, 1.0;
    
    VectorXd x = minres.solve(b);
    std::cout << "MINRES solution: " << x.transpose() << std::endl;
    std::cout << "MINRES info: " << minres.info() << std::endl;

    MINRES<SparseMatrix<double>, Lower, DiagonalPreconditioner<double>> minres_diag;
    minres_diag.compute(N);
    VectorXd x_diag = minres_diag.solve(b);
    std::cout << "MINRES (Diag) solution: " << x_diag.transpose() << std::endl;
    
    return 0;
}
