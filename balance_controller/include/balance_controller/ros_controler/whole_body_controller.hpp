#include <string>

#include <kindr/Core>
#include <quadruped_model/common/typedefs.hpp>
#include "Eigen/Sparse"
#include "Eigen/SparseCore"
#include "Eigen/Dense"
#include <Eigen/Geometry>
#include <Eigen/Core>
#include "Eigen/Eigen"
#include "Eigen/LU"
#include <memory>
#include <geometry_msgs/Accel.h>
// whole body model
#include <WholeBodyKinematic.h>
#include <WholeBodyDynamic.h>
#include <RigidBodyDynamics.h>
#include <utils/URDF.h>
// free_gait
#include <free_gait_core/TypeDefs.hpp>
#include <free_gait_core/executor/State.hpp>
#include <free_gait_msgs/RobotState.h>
#include "sim_assiants/FootContacts.h"
// ros controller
#include <ros/ros.h>
#include <ros/package.h>
#include "controller_interface/controller.h"
#include "balance_controller/ros_controler/robot_state_interface.hpp"
#include "balance_controller/ros_controler/gazebo_state_hardware_interface.hpp"
#include <pluginlib/class_list_macros.hpp>
// solver
#include "ooqp_eigen_interface/QuadraticProblemFormulation.hpp"
#include "ooqp_eigen_interface/OoqpEigenInterface.hpp"

using namespace std;
using namespace romo;
using namespace quadruped_model;



namespace balance_controller {

typedef std::unordered_map<free_gait::LimbEnum, bool, EnumClassHash> LimbFlag;
typedef std::unordered_map<free_gait::LimbEnum, free_gait::Vector, EnumClassHash> LimbVector;
typedef std::vector<free_gait::LimbEnum> LimbSet; // LF RF LH RH
typedef std::vector<free_gait::BranchEnum> BranchSet;
typedef std::vector<std::string> LimbSelector;
typedef std::map<std::string, Eigen::VectorXd> LimbVectorXd;
typedef std::map<free_gait::LimbEnum, Force> ExternalForece;

class WholeBodyController : public controller_interface::Controller<hardware_interface::RobotStateInterface>
{


public:

    WholeBodyController();
//    WholeBodyController(const ros::NodeHandle& node_handle,
//           std::shared_ptr<free_gait::State> robot_state);
    ~WholeBodyController();


    // Info of leg and foot.
    struct LegInfo
    {
        bool isPartOfForceDistribution_;
        bool isLoadConstraintActive_;
        int indexInStanceLegList_;
        int startIndexInVectorX_;
        Force desiredContactForce_;
        Vector firstDirectionOfFrictionPyramidInWorldFrame_;
        Vector secondDirectionOfFrictionPyramidInWorldFrame_;
        Vector normalDirectionOfFrictionPyramidInWorldFrame_;
        // Assumed friction coefficient (mu).
        double frictionCoefficient_;
        double loadFactor_;
    };

    bool init(hardware_interface::RobotStateInterface* hardware,
              ros::NodeHandle& node_handle);

    void update(const ros::Time& time, const ros::Duration& period);

    void starting(const ros::Time& time);

    void stopping(const ros::Time& time);

    bool loadModelFromURDF();

    bool prepareOptimazation(const LinearAcceleration& linear_a, const AngularAcceleration& angular_a);

    bool solveOptimazation();

    bool resetOptimazation();

    bool addPhysicalConsistencyConstraints();

    bool addStanceAndSwingLegConstraints();

    bool addStanceConditionConstraints();

    bool addSwingTaskConstraints();

    bool addFrictionConeConstraints();

    bool addTorqueLimitConstraints();

    bool addJointKinematicsLimitConstraints();

    bool prepareLegLoading();

    void desiredCommandCallback(const free_gait_msgs::RobotStateConstPtr& robot_state_msg);

    void footContactsCallback(const sim_assiants::FootContactsConstPtr& foot_contacts);

    void contactStateMachine();

    void computeJdotQdot(LimbVectorXd& jacd_qd,
                         Position& base_pos, RotationQuaternion& base_orientation,
                         LinearVelocity& base_linear_vel, LocalAngularVelocity& base_angular_vel,
                         JointPositions& joint_pos, JointVelocities& joint_vel, LimbSelector limbset);

    const Eigen::MatrixXd computeJointSpaceInertialMatrix(Position& base_pos,
                                                          RotationQuaternion& base_orientation,
                                                          JointPositions& joint_pos);
    const Eigen::MatrixXd getInertialBaseMatrix();

    const Eigen::MatrixXd getInertialJointMatrix();

    const Eigen::VectorXd computeNonlinearEffectsForce(Position& base_pos, RotationQuaternion& base_orientation,
                                                       LinearVelocity& base_linear_vel, LocalAngularVelocity& base_angular_vel,
                                                       JointPositions& joint_pos, JointVelocities& joint_vel);

    const Eigen::VectorXd getNonlinearBaseEffectsForce();

    const Eigen::VectorXd getNonlinearJointEffectsForce();

    const Eigen::MatrixXd computeLegJacobian(Position& base_pos, RotationQuaternion& base_orientation,
                                          JointPositions& joint_pos, free_gait::LimbEnum limb);

    const Eigen::MatrixXd computeJacobian(Position& base_pos, RotationQuaternion& base_orientation,
                                           JointPositions& joint_pos, LimbSelector limb_set);

    const Eigen::MatrixXd getFloatingJacobian();

    const Eigen::MatrixXd getFixedJacobian();

    const JointTorques computeFloatingBaseInverseDynamics(Position& base_pos, RotationQuaternion& base_orientation,
                                                             LinearVelocity& base_linear_vel, LocalAngularVelocity& base_angular_vel,
                                                             JointPositions& joint_pos, JointVelocities& joint_vel,
                                                             LinearAcceleration& base_linear_acc, AngularAcceleration& base_angular_acc,
                                                             JointAccelerations& joint_acc, ExternalForece& ext_force);
    const JointTorques getFeedforwardJointTorque();

    void setLimbJointTorque();

    bool optimazationed_;

private:

    ros::NodeHandle node_handle_;

    // Number of legs in stance phase
    int nLegsInForceDistribution_;
    // Dof of a leg, 3
    int footDof_;
    // Number of variables to optimize (size of x, n = nTranslationalDofPerFoot_ * nLegsInStance_)
    int n_;
    // Robot mass
    double robotMass_;
    // min joint torque uesd to optimization, 12 x 1
    Eigen::VectorXd minTorque_;
    // max joint torque used to optimization, 12 x 1
    Eigen::VectorXd maxTorque_;
    // The matrix G in the optimization formulation (nElementsInStackedVirtualForceTorqueVector_ x n).
    Eigen::SparseMatrix<double, Eigen::RowMajor> G_; // todo when can use sparseMatrix?
    // The vector g in the optimization formulation (stacked vector of reference linear acceleration and angular acceleration).
    Eigen::VectorXd g_;
    // Stacked generalized accelerations and the contact forces as decision variables. (size of 6 + 12 + 3 x nLegsInForceDistribution_)
    Eigen::VectorXd x_;

    // Weighting matrix for the ground reaction forces (regularizer).
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> W_; // (18 + n_) x (18 + n_)

    Eigen::DiagonalMatrix<double, Eigen::Dynamic> H_;
    // Weighting matrix for the desired virtual forces and torques.
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> S_; // 6 x 6
    // Inequality constraint matrix
    Eigen::SparseMatrix<double, Eigen::RowMajor> D_;
    // Upper and lower limits vectors of inequality constraint
    Eigen::VectorXd d_, f_;
    // Equality constraint matrix
    Eigen::SparseMatrix<double, Eigen::RowMajor> C_;
    // Vector of equality constraint
    Eigen::VectorXd c_;

    // whole body model
    //wbc::WholeBodyKinematic wkin;
    std::shared_ptr<wbc::WholeBodyKinematic> wkin;
    std::shared_ptr<wbc::WholeBodyDynamic> wdyn;
    //wbc::WholeBodyDynamic wdyn;
    wbc::rbd::BodyID body_id_;

    RigidBodyDynamics::Model rbd;

    // robot dynamics
    // M(q)
    Eigen::MatrixXd inertial_matrix;// 18 x 18
    Eigen::MatrixXd inertial_base_matrix;// 6 x 6
    Eigen::MatrixXd inertial_joint_matrix;// 12 x 12
    // h(qdot, q)
    Eigen::VectorXd nonlinear_effect;// 18 x 1
    // todo should not as class member?
    Eigen::VectorXd nonlinear_base_effect;// 6 x 1
    Eigen::VectorXd nonlinear_joint_effect;// 12 x 1
    // J(q)
    Eigen::MatrixXd jacobian;// n_ x 18, n_ is number of leg in stance
    Eigen::MatrixXd floating_jac;// n_ x 6
    Eigen::MatrixXd fixed_jac;// n_ x 12


    // free_gait
    std::map<free_gait::LimbEnum, LegInfo> legInfos_;


    // desired motion command
    Position base_desired_position;
    RotationQuaternion base_desired_rotation;
    LinearVelocity base_desired_linear_velocity;
    LocalAngularVelocity base_desired_angular_velocity;
    LinearAcceleration base_desired_linear_acceleration;
    AngularAcceleration base_desired_angular_acceleration;

    //refer base acceleration after PD
    LinearAcceleration base_reference_linear_acceleration;
    AngularAcceleration base_reference_angular_acceleration;

    // optimal base and joint acceleration, use for save optimazation result
    LinearAcceleration base_optimal_linear_acceleration;
    AngularAcceleration base_optimal_angular_acceleration;
    JointAccelerations joint_optimal_acceleration;
    std::map<free_gait::LimbEnum, Force> ground_force_;

    // robot handle
    hardware_interface::RobotStateHandle robot_state_handle;
    std::shared_ptr<free_gait::State> robot_state_;

    std::vector<std::string> joint_names;
    unsigned int n_joints;

    // PD acceleration gain
    //Eigen::Matrix3d Kr, Dr, Kw, Dw;
    Eigen::Vector3d Kr, Dr, Kw, Dw;

    // free_gait
    LimbFlag real_contact_;
    LimbVector real_contact_force_;
    std::vector<std::string> limbset_;
    std::vector<free_gait::LimbEnum> limbs_; // LF RF LH RH
    std::vector<free_gait::BranchEnum> branches_;



};

}
