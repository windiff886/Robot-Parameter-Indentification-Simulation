/**
 * @file dynamics_diagnostic.cpp
 * @brief 诊断 RobotDynamics 与 MuJoCo 数据不一致的原因
 * 
 * 对比以下几个计算源：
 * 1. RobotDynamics (DH参数实现)
 * 2. MuJoCoPandaDynamics (MuJoCo XML参数实现)
 * 3. 实验数据中记录的扭矩
 */

#include "identification/data_loader.hpp"
#include "mujoco_panda_dynamics.hpp"
#include "robot/franka_panda.hpp"
#include "robot/robot_dynamics.hpp"
#include <Eigen/Core>
#include <iomanip>
#include <iostream>

void printVector(const std::string& name, const Eigen::VectorXd& v) {
    std::cout << "  " << std::setw(25) << std::left << name << ": [";
    for (int i = 0; i < v.size(); ++i) {
        std::cout << std::setw(10) << std::fixed << std::setprecision(4) << v(i);
        if (i < v.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

void printMatrix(const std::string& name, const Eigen::MatrixXd& M, int rows = 3) {
    std::cout << "  " << name << " (first " << rows << " rows):" << std::endl;
    for (int i = 0; i < std::min(rows, (int)M.rows()); ++i) {
        std::cout << "    [";
        for (int j = 0; j < M.cols(); ++j) {
            std::cout << std::setw(10) << std::fixed << std::setprecision(4) << M(i, j);
            if (j < M.cols() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
}

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    std::cout << std::string(70, '=') << std::endl;
    std::cout << "  DYNAMICS MODEL DIAGNOSTIC" << std::endl;
    std::cout << std::string(70, '=') << std::endl;

    // ====================================================================
    // 1. 初始化两个动力学模型
    // ====================================================================
    auto robot_model = robot::createFrankaPanda();
    robot::RobotDynamics robot_dynamics(*robot_model);
    mujoco_dynamics::MuJoCoPandaDynamics mujoco_dynamics;

    // ====================================================================
    // 2. 加载数据
    // ====================================================================
    std::string data_file = "/home/windiff/Code/Simulation/data/benchmark_data_2026-01-18_15-54-50.csv";
    DataLoader loader;
    ExperimentData data = loader.loadCSV(data_file, 7);
    std::cout << "\nLoaded " << data.n_samples << " samples." << std::endl;

    // ====================================================================
    // 3. 选取测试样本
    // ====================================================================
    std::vector<std::size_t> test_indices = {100, 500, 1000, 2000, 3000, 5000};
    
    double sum_sq_error_rob = 0.0;
    double sum_sq_error_mj = 0.0;
    std::size_t total_elements = 0;

    for (std::size_t sample_idx : test_indices) {
        if (sample_idx >= data.n_samples) continue;

        Eigen::VectorXd q = data.q.row(sample_idx).transpose();
        Eigen::VectorXd qd = data.qd.row(sample_idx).transpose();
        Eigen::VectorXd qdd = data.qdd.row(sample_idx).transpose();
        Eigen::VectorXd tau_recorded = data.tau.row(sample_idx).transpose();

        std::cout << "\n" << std::string(70, '-') << std::endl;
        std::cout << "Sample " << sample_idx << " (t = " << data.time[sample_idx] << " s)" << std::endl;
        std::cout << std::string(70, '-') << std::endl;

        printVector("q", q);
        printVector("qd", qd);
        printVector("qdd", qdd);
        printVector("tau (recorded)", tau_recorded);

        // ====================================================================
        // 4. RobotDynamics 计算
        // ====================================================================
        std::cout << "\n  [RobotDynamics - DH Parameters]" << std::endl;
        
        Eigen::MatrixXd M_rob = robot_dynamics.computeInertiaMatrix(q);
        Eigen::MatrixXd C_rob = robot_dynamics.computeCoriolisMatrix(q, qd);
        Eigen::VectorXd G_rob = robot_dynamics.computeGravityVector(q);
        
        // 基础动力学 (无 armature/damping)
        Eigen::VectorXd tau_rob_base = M_rob * qdd + C_rob * qd + G_rob;
        
        // 添加 armature 和 damping (按 main.cpp 的方式)
        Eigen::VectorXd tau_rob = tau_rob_base;
        for (int k = 0; k < 7; ++k) {
            tau_rob(k) += 0.1 * qdd(k);  // armature
            tau_rob(k) += 1.0 * qd(k);    // damping
        }
        
        printVector("M*qdd + C*qd + G", tau_rob_base);
        printVector("+ armature*qdd + damp*qd", tau_rob);

        // ====================================================================
        // 5. MuJoCoPandaDynamics 计算
        // ====================================================================
        std::cout << "\n  [MuJoCoPandaDynamics - MuJoCo XML]" << std::endl;
        
        Eigen::MatrixXd M_mj = mujoco_dynamics.computeInertiaMatrix(q);
        Eigen::MatrixXd C_mj = mujoco_dynamics.computeCoriolisMatrix(q, qd);
        Eigen::VectorXd G_mj = mujoco_dynamics.computeGravityVector(q);
        
        // MuJoCoPandaDynamics::computeInverseDynamics 已包含 armature，但减去 damping
        Eigen::VectorXd tau_mj_fn = mujoco_dynamics.computeInverseDynamics(q, qd, qdd);
        
        // 手动计算 (M 已包含 armature)
        Eigen::VectorXd tau_mj_base = M_mj * qdd + C_mj * qd + G_mj;
        
        // 修正：damping 应该加上而不是减去
        Eigen::VectorXd tau_mj_corrected = tau_mj_base;
        for (int k = 0; k < 7; ++k) {
            tau_mj_corrected(k) += 1.0 * qd(k);  // damping (加上)
        }
        
        printVector("M*qdd + C*qd + G", tau_mj_base);
        printVector("(函数输出, -damp)", tau_mj_fn);
        printVector("(修正后, +damp)", tau_mj_corrected);

        // ====================================================================
        // 6. 误差分析
        // ====================================================================
        std::cout << "\n  [Error Analysis]" << std::endl;
        
        Eigen::VectorXd err_rob = tau_rob - tau_recorded;
        Eigen::VectorXd err_mj = tau_mj_corrected - tau_recorded;
        
        double rmse_rob = std::sqrt(err_rob.squaredNorm() / 7.0);
        double rmse_mj = std::sqrt(err_mj.squaredNorm() / 7.0);
        
        printVector("Rob Error", err_rob);
        printVector("MJ Error", err_mj);
        std::cout << "  RobotDynamics RMSE: " << rmse_rob << " Nm" << std::endl;
        std::cout << "  MuJoCoDynamics RMSE: " << rmse_mj << " Nm" << std::endl;
        
        sum_sq_error_rob += err_rob.squaredNorm();
        sum_sq_error_mj += err_mj.squaredNorm();
        total_elements += 7;

        // ====================================================================
        // 7. 对比惯量矩阵对角线
        // ====================================================================
        if (sample_idx == 1000) {
            std::cout << "\n  [Inertia Matrix Diagonal Comparison]" << std::endl;
            std::cout << "  RobotDynamics M diag: [";
            for (int i = 0; i < 7; ++i) {
                std::cout << std::setw(8) << std::fixed << std::setprecision(4) << M_rob(i, i);
                if (i < 6) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
            
            std::cout << "  MuJoCoDyn M diag:     [";
            for (int i = 0; i < 7; ++i) {
                std::cout << std::setw(8) << std::fixed << std::setprecision(4) << M_mj(i, i);
                if (i < 6) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
            
            std::cout << "  Difference:           [";
            for (int i = 0; i < 7; ++i) {
                std::cout << std::setw(8) << std::fixed << std::setprecision(4) << (M_rob(i, i) - M_mj(i, i));
                if (i < 6) std::cout << ", ";
            }
            std::cout << "]" << std::endl;

            // 重力向量对比
            std::cout << "\n  [Gravity Vector Comparison]" << std::endl;
            printVector("RobotDynamics G", G_rob);
            printVector("MuJoCoDynamics G", G_mj);
            printVector("Difference", G_rob - G_mj);
        }
    }

    // ====================================================================
    // 8. 总结
    // ====================================================================
    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "  SUMMARY" << std::endl;
    std::cout << std::string(70, '=') << std::endl;
    
    double overall_rmse_rob = std::sqrt(sum_sq_error_rob / total_elements);
    double overall_rmse_mj = std::sqrt(sum_sq_error_mj / total_elements);
    
    std::cout << "  Overall RobotDynamics RMSE:    " << overall_rmse_rob << " Nm" << std::endl;
    std::cout << "  Overall MuJoCoPandaDyn RMSE:   " << overall_rmse_mj << " Nm" << std::endl;
    
    if (overall_rmse_mj < 0.5) {
        std::cout << "\n  [OK] MuJoCoPandaDynamics matches recorded data well!" << std::endl;
    } else {
        std::cout << "\n  [WARNING] Both models have significant errors!" << std::endl;
        std::cout << "  Possible causes:" << std::endl;
        std::cout << "    1. Recorded tau may NOT be inverse dynamics result" << std::endl;
        std::cout << "    2. Tau may be the COMMANDED torque, not the actual one" << std::endl;
        std::cout << "    3. Actuator model adds additional forces" << std::endl;
    }

    // ====================================================================
    // 9. 检查记录的 tau 是否为 0 (早期样本)
    // ====================================================================
    std::cout << "\n" << std::string(70, '-') << std::endl;
    std::cout << "  CHECKING EARLY SAMPLES (tau = 0?)" << std::endl;
    std::cout << std::string(70, '-') << std::endl;
    
    int zero_tau_count = 0;
    for (std::size_t i = 0; i < std::min(data.n_samples, (std::size_t)100); ++i) {
        double tau_norm = data.tau.row(i).norm();
        if (tau_norm < 1e-6) {
            ++zero_tau_count;
        }
    }
    std::cout << "  First 100 samples: " << zero_tau_count << " have tau ≈ 0" << std::endl;
    
    if (zero_tau_count > 0) {
        std::cout << "  [WARNING] Zero torque samples detected! Check data recording." << std::endl;
    }

    return 0;
}
