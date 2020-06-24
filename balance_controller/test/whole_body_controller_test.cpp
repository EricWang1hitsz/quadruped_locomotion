#include <balance_controller/ros_controler/whole_body_controller.hpp>
#include <ros/ros.h>
#include <quadruped_model/common/typedefs.hpp>
#include <free_gait_core/TypeDefs.hpp>
#include <memory>


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "whole_body_controller_test");
    ros::NodeHandle nh("~");
    Position base_pos(0, 0, 0);
    RotationQuaternion base_w(0, 0, 0, 1);
    JointPositions joint_pos;
    joint_pos << 0.0, 0.75, -1.5,
                 0.0, -0.75, 1.5,
                 0.0, 0.75, -1.5,
                 0.0, -0.75, 1.5;
    std::cout << joint_pos << std::endl;

    LinearVelocity base_vel(0, 0, 0);
    LocalAngularVelocity base_vel_w(0, 0, 0);

    JointVelocities joint_vel;
    joint_vel << 0, 0, 0,
                 0, 0, 0,
                 0, 0, 0,
                 0, 0, 0;
    std:: cout << joint_vel << std::endl;

    std::shared_ptr<free_gait::State> state_;
//    std::shared_ptr<free_gait::State> state;
//    state_.reset(new free_gait::State);
//    std::vector<free_gait::LimbEnum> limbs_; // LF RF LH RH
//    std::vector<free_gait::BranchEnum> branches_;

//    limbs_.push_back(free_gait::LimbEnum::LF_LEG);
//    limbs_.push_back(free_gait::LimbEnum::RF_LEG);
//    limbs_.push_back(free_gait::LimbEnum::LH_LEG);
//    limbs_.push_back(free_gait::LimbEnum::RH_LEG);

//    branches_.push_back(free_gait::BranchEnum::BASE);
//    branches_.push_back(free_gait::BranchEnum::LF_LEG);
//    branches_.push_back(free_gait::BranchEnum::RF_LEG);
//    branches_.push_back(free_gait::BranchEnum::LH_LEG);
//    branches_.push_back(free_gait::BranchEnum::RH_LEG);
    ROS_INFO("initilizing...");
//    state_->Initialize();
//    state_->initialize(limbs_, branches_);
//    state_.reset(new free_gait::State);


//    for(unsigned int i = 0; i < 4; i++)
//    {
//        state_->setSupportLeg(static_cast<free_gait::LimbEnum>(i), true);
//        state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(i), Vector(0, 0, 1));
//    }
    ROS_INFO("test  1  here");
    state_->setPositionWorldToBaseInWorldFrame(base_pos);
    state_->setOrientationBaseToWorld(base_w);
    state_->setJointPositions(joint_pos);
    ROS_INFO("test  1  here");
    state_->setLinearVelocityBaseInWorldFrame(base_vel);
    state_->setAngularVelocityBaseInBaseFrame(base_vel_w);
    state_->setJointVelocities(joint_vel);
    ROS_INFO("test here");

    // swing leg foot acceleration
//    LinearAcceleration foot_acceleration;
//    foot_acceleration << 1, 1, 1;
//    state_->setTargetFootAccelerationInBaseForLimb(foot_acceleration, free_gait::LimbEnum::LF_LEG);
//    state_->setTargetFootAccelerationInBaseForLimb(foot_acceleration, free_gait::LimbEnum::RF_LEG);
//    state_->setTargetFootAccelerationInBaseForLimb(foot_acceleration, free_gait::LimbEnum::LH_LEG);
//    state_->setTargetFootAccelerationInBaseForLimb(foot_acceleration, free_gait::LimbEnum::RH_LEG);

    std::cout << " test here 1 " << std::endl;

//    balance_controller::WholeBodyController wbc(state_);
    auto wbc = std::shared_ptr<balance_controller::WholeBodyController>(new balance_controller::WholeBodyController(nh, state_));

    LinearAcceleration reference_acc;
    reference_acc << 1, 0, 0;
    AngularAcceleration reference_w;
    reference_w << 0.0, 0.0, 0.0;



//    LimbSelector limbset_;
    // quadruped robot foot order
//    limbset_.push_back("lf_foot");
//    limbset_.push_back("rf_foot");
//    limbset_.push_back("lh_foot");
//    limbset_.push_back("rh_foot");
    // HyQ foot order
//    limbset_.push_back("lf_foot");
//    limbset_.push_back("lh_foot");
//    limbset_.push_back("rf_foot");
//    limbset_.push_back("rh_foot");

    // tset joint space inertial matrix
//    std::cout << "--------------------joint space inertial matrix----------------" << std::endl;
//    Eigen::MatrixXd joint_space_inertial_matrix;
//    joint_space_inertial_matrix = wbc->computeJointSpaceInertialMatrix(base_pos, base_w, joint_pos);
    std::cout << fixed << setprecision(3) << std::endl;
//    std::cout << joint_space_inertial_matrix << std::endl;
//    std::cout << "-------------------------inertial base matrix----------------" << std::endl;
//    std::cout << wbc->getInertialBaseMatrix() << std::endl;
//    std::cout << "-------------------------inertial joint matrix----------------" << std::endl;
//    std::cout << wbc->getInertialJointMatrix() << std::endl;

//    // test nonlinear effect force matrix
//    std::cout << "--------------------nonlinear effect force matrix----------------" << std::endl;
//    Eigen::VectorXd nonlinear_effect_force;
//    nonlinear_effect_force = wbc.computeNonlinearEffectsForce(base_pos, base_w, base_vel, base_vel_w, joint_pos, joint_vel);
//    std::cout << nonlinear_effect_force << std::endl;
//    std::cout << "--------------------nonlinear base effect matrix----------------" << std::endl;
//    std::cout << wbc.getNonlinearBaseEffectsForce() << std::endl;
//    std::cout << "--------------------nonlinear joint effect matrix----------------" << std::endl;
//    std::cout << wbc.getNonlinearJointEffectsForce() << std::endl;

//    // tset jacobian matrix
//    std::cout << "-----------------------jacobian matrix---------------------------" << std::endl;
//    Eigen::MatrixXd jacobian;
//    jacobian = wbc.computeJacobian(base_pos, base_w, joint_pos, limbset_);
//    std::cout << jacobian << std::endl;
    wbc->prepareLegLoading();
    wbc->prepareOptimazation(reference_acc, reference_w);
    wbc->addPhysicalConsistencyConstraints();
    wbc->addStanceAndSwingLegConstraints();
    wbc->addFrictionConeConstraints();
    wbc->addTorqueLimitConstraints();
    wbc->solveOptimazation();
    if(wbc->optimazation_)
    {
        JointTorques joint_torque;
        joint_torque = wbc->getFeedforwardJointTorque();
        std::cout << "-------------------------Feed-Forward Joint Torque--------------------" << std::endl;
        std::cout << joint_torque << std::endl;
    }

    std::cout << "END" << std::endl;

}
