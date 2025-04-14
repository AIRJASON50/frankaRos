// 设置笛卡尔空间阻抗控制参数（刚度矩阵）
cartesian_stiffness_.setIdentity();
// 设置位置刚度（前3x3矩阵）
cartesian_stiffness_.topLeftCorner(3, 3) << 100.0 * Eigen::Matrix3d::Identity();
// 设置方向刚度（后3x3矩阵），从10.0增加到30.0
cartesian_stiffness_.bottomRightCorner(3, 3) << 30.0 * Eigen::Matrix3d::Identity();

// 设置阻尼矩阵（用于稳定系统）
cartesian_damping_.setIdentity();
// 设置位置阻尼
cartesian_damping_.topLeftCorner(3, 3) << 2.0 * sqrt(100.0) * Eigen::Matrix3d::Identity();
// 设置方向阻尼，相应调整
cartesian_damping_.bottomRightCorner(3, 3) << 2.0 * sqrt(30.0) * Eigen::Matrix3d::Identity(); 