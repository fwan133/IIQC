#include <Eigen/Geometry>

#include <iostream>

Eigen::Matrix4d
solveSIM3(const std::vector<Eigen::Isometry3d> &Ts_source, const std::vector<Eigen::Isometry3d> &Ts_target) {
    /// Data check
    if (Ts_source.size() != Ts_target.size()) {
        throw std::runtime_error("[Error]: The size of input source and target is not the same.");
    }

    /// Fundamental data
    std::vector<Eigen::Quaterniond> qs_source_target;
    std::vector<Eigen::Vector3d> ts_source;
    std::vector<Eigen::Vector3d> ts_target;
    Eigen::Vector3d source_centroid(0, 0, 0);
    Eigen::Vector3d target_centroid(0, 0, 0);

    int num_poses = Ts_source.size();
    for (int i = 0; i < num_poses; i++) {
        qs_source_target.push_back(Eigen::Quaterniond((Ts_source[i].inverse() * Ts_target[i]).linear()));
        ts_source.push_back(Ts_source[i].translation());
        ts_target.push_back(Ts_target[i].translation());
        source_centroid += Ts_source[i].translation();
        target_centroid += Ts_target[i].translation();
    }

    source_centroid /= num_poses;
    target_centroid /= num_poses;

    /// Calculate the rotation
    Eigen::Matrix3d RotationMatrix;

    Eigen::Quaterniond averageQuaternion(0.0, 0.0, 0.0, 0.0);
    for (const Eigen::Quaterniond &q: qs_source_target) {
        averageQuaternion.coeffs() += q.coeffs();
    }
    averageQuaternion.coeffs() /= static_cast<double>(qs_source_target.size());

    RotationMatrix = averageQuaternion.toRotationMatrix();

    /// Calculate the scale
    double scaling_factor;

    Eigen::MatrixXd centered_source(3, num_poses);
    Eigen::MatrixXd centered_target(3, num_poses);
    for (int i = 0; i < num_poses; i++) {
        centered_source.col(i) = ts_source[i] - source_centroid;
        centered_target.col(i) = ts_target[i] - target_centroid;
    }

    if (num_poses == 1) {
        scaling_factor = 1;
    } else {
        scaling_factor = (centered_target.norm() / centered_source.norm());
    }

    /// Calculate the translation
    Eigen::Vector3d TranslationVector = target_centroid - scaling_factor * RotationMatrix * source_centroid;

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = scaling_factor * RotationMatrix;
    T.block<3, 1>(0, 3) = TranslationVector;

    return T;
}

int main(int argc, char **argv){
    /// Define transformation parameters
    double scale = 2.0;
    Eigen::Quaterniond rotation(0.7071068, 0.7071068, 0, 0); // No rotation for simplicity
    Eigen::Vector3d translation(3.0, 2.0, 3.0);
    Eigen::Matrix4d sim3Transform = Eigen::Matrix4d::Identity();
    sim3Transform.block<3, 3>(0, 0) = scale * rotation.toRotationMatrix();
    sim3Transform.block<3, 1>(0, 3) = translation;

    /// Define the source Isodometry
    Eigen::Vector3d pos_1(2.0, 4.0, 6.0);
    Eigen::Quaterniond ori_1(0.8535534, 0.3535534, 0.3535534, 0.1464466);
    Eigen::Isometry3d T_1 = Eigen::Isometry3d::Identity();
    T_1.prerotate(ori_1);
    T_1.pretranslate(pos_1);

    Eigen::Vector3d pos_2(2.0, 4.0, 4.0);
    Eigen::Quaterniond ori_2(0.6532815 , 0.2705981, 0.6532815, 0.2705981);
    Eigen::Isometry3d T_2 = Eigen::Isometry3d::Identity();
    T_1.prerotate(ori_1);
    T_1.pretranslate(pos_1);

    std::vector<Eigen::Isometry3d> Ts_source;
//    Ts_source.push_back(T_1);
    Ts_source.push_back(T_2);

    /// Calculate the target Isodomety
    std::vector<Eigen::Isometry3d> Ts_target;
    for (auto &T:Ts_source){
        Eigen::Matrix4d T_source = T.matrix();
        Eigen::Matrix4d T_target_tem = sim3Transform * T_source;
        Eigen::Isometry3d T_target;
        T_target.matrix()=T_target_tem;
        Ts_target.push_back(T_target);
    }

    /// Calculate the SIM3 transformation
    Eigen::Matrix4d Sim3_cal=solveSIM3(Ts_source,Ts_target);
    std::cout << "The SIM3 transformation matrix is \n" << Sim3_cal << std::endl;

    return 0;
}