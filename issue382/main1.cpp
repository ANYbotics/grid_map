#include <Eigen/Core>

int main() {

    Eigen::VectorXd v(10);
    v[0] = 0.1;
    v[1] = 0.2;
    v(0) = 0.3;
    v(1) = 0.4;
    return 0;
}