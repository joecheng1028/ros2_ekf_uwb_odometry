#include <iostream>
#include <Eigen/Dense>


int main(){

    Eigen::Vector3d tag (3, 6, 0);
    
    std::vector<Eigen::Vector3d> anchors={
        {4, 4, 2},
        {4, 8, 2},
        {2, 4, 2},
        {2, 8, 2}
    };

    int n = anchors.size();

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n,3);
    Eigen::VectorXd r = Eigen::VectorXd::Zero(n);

    
    for (int i = 0; i < n; i++){
        Eigen::Vector3d diffs = anchors[i] - tag;
        r(i) = diffs.norm();
        H.row(i) = diffs / r(i);
    }
    
    Eigen::Matrix3d HtH = H.transpose() * H;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(HtH, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd sig = svd.singularValues(); // vector containing all sigular value, not sigma (mxn)
    Eigen::MatrixXd sig_plus = Eigen::MatrixXd::Zero(sig.size(), sig.size());
    for (int i = 0; i < sig.size(); i++){
        if (sig(i) > 1e-10){
            sig_plus(i,i) = 1.0 / sig(i);    // Diagonal matrix in SVD, S^-1 -> all entries inverted
        }
    }

    Eigen::MatrixXd Q = svd.matrixV() * sig_plus * svd.matrixU().transpose();

    double PDOP = sqrt(Q(0,0) + Q(1,1) + Q(2,2));

    // std::cout << "H:\n" << H << "\n\nr:\n" << r << "\n";
    std::cout << "PDOP: " << PDOP << std::endl;

    return 0;

}