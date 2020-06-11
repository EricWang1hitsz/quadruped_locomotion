#include <balance_controller/ros_controler/whole_body_controller.hpp>
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <iomanip>
using namespace Eigen;

namespace balance_controller {

WholeBodyController::WholeBodyController()
{
    wdyn.reset(new dwl::model::WholeBodyDynamics());

    // actual joint data order LF RF LH RH
    limbs_.push_back(free_gait::LimbEnum::LF_LEG);
    limbs_.push_back(free_gait::LimbEnum::RF_LEG);
    limbs_.push_back(free_gait::LimbEnum::LH_LEG);
    limbs_.push_back(free_gait::LimbEnum::RH_LEG);
    //todo need change if the name of URDF change
    limbset_.push_back("lf_foot");
    limbset_.push_back("rf_foot");
    limbset_.push_back("lh_foot");
    limbset_.push_back("rh_foot");

    // model floating base system and kinematics
    string urdf_file = "/home/hit/catkin_ws_code/src/dwl_ros/src/test/robot.urdf";
    string yarf_file = "/home/hit/catkin_ws_code/src/dwl_ros/src/test/hyq.yarf";
    fbs.resetFromURDFFile(urdf_file, yarf_file);
    wkin.modelFromURDFFile(urdf_file, yarf_file);
    wdyn->modelFromURDFFile(urdf_file, yarf_file);

    // test if URDF is read correctly
//    string model_hierarchy = RigidBodyDynamics::Utils::GetModelHierarchy(fbs.getRBDModel());
//    std::cout << model_hierarchy << std::endl;

    // get id of each body
    dwl::rbd::getListOfBodies(body_id_, fbs.getRBDModel());
    ROS_INFO("test class initialization end");

}

WholeBodyController::WholeBodyController(std::shared_ptr<free_gait::State> robot_state)
    : robot_state_(robot_state)
{
    wdyn.reset(new dwl::model::WholeBodyDynamics());

    // actual joint data order LF RF LH RH
    limbs_.push_back(free_gait::LimbEnum::LF_LEG);
    limbs_.push_back(free_gait::LimbEnum::RF_LEG);
    limbs_.push_back(free_gait::LimbEnum::LH_LEG);
    limbs_.push_back(free_gait::LimbEnum::RH_LEG);
    for(auto leg : limbs_)
    {
        legInfos_[leg] = LegInfo();
    }
    branches_.push_back(free_gait::BranchEnum::BASE);
    branches_.push_back(free_gait::BranchEnum::LF_LEG);
    branches_.push_back(free_gait::BranchEnum::RF_LEG);
    branches_.push_back(free_gait::BranchEnum::LH_LEG);
    branches_.push_back(free_gait::BranchEnum::RH_LEG);
    //TODO why must reset, or cannot traversal limbs.
    robot_state_.reset(new free_gait::State);
//    robot_state_->initialize(limbs_, branches_);

    for(unsigned int i = 0; i < 4; i++)
    {
        robot_state_->setSupportLeg(static_cast<free_gait::LimbEnum>(i), true);
        robot_state_->setSurfaceNormal(static_cast<free_gait::LimbEnum>(i), Vector(0, 0, 1));
    }

    //todo need change if the name of URDF change
    limbset_.push_back("lf_foot_Link");
    limbset_.push_back("rf_foot_Link");
    limbset_.push_back("lh_foot_Link");
    limbset_.push_back("rh_foot_Link");

    minTorque_(12);
    maxTorque_(12);

    robotMass_ = 50;
    // set stance legs number
//    nLegsInForceDistribution_ = 4;
    footDof_ = 3;
//    n_ = footDof_ * nLegsInForceDistribution_;

    // model floating base system and kinematics
    string urdf_file = "/home/hit/catkin_ws_code/src/dwl_ros/src/test/robot.urdf";
    string yarf_file = "/home/hit/catkin_ws_code/src/dwl_ros/src/test/hyq.yarf";
    fbs.resetFromURDFFile(urdf_file, yarf_file);
    wkin.modelFromURDFFile(urdf_file, yarf_file);
    wdyn->modelFromURDFFile(urdf_file, yarf_file);

    // test if URDF is read correctly
//    string model_hierarchy = RigidBodyDynamics::Utils::GetModelHierarchy(fbs.getRBDModel());
//    std::cout << model_hierarchy << std::endl;

    // get id of each body
    dwl::rbd::getListOfBodies(body_id_, fbs.getRBDModel());
    ROS_INFO("test class 2 initialization end");
}

WholeBodyController::~WholeBodyController()
{
    ROS_INFO("whole body controller end");
}

bool WholeBodyController::init(hardware_interface::RobotStateInterface *hardware, ros::NodeHandle &node_handle)
{
    ROS_INFO("Initialize whole-body controller...");
    std::string param_name = "joints";
    if(!node_handle.getParam(param_name, joint_names))
    {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << node_handle.getNamespace() << ").");
        return false;
    }
    n_joints = joint_names.size();
    if(n_joints == 0)
    {
        ROS_ERROR_STREAM("List of joint names is empty.");
        return false;
    }

    robot_state_handle = hardware->getHandle("base_controller");

}

void WholeBodyController::update(const ros::Time &time, const ros::Duration &period)
{
    // actual base pose
    Position base_actual_position = Position(robot_state_handle.getPosition()[0],
            robot_state_handle.getPosition()[1],
            robot_state_handle.getPosition()[2]);
    RotationQuaternion base_actual_orientation = RotationQuaternion(robot_state_handle.getOrientation()[0],
            robot_state_handle.getOrientation()[1],
            robot_state_handle.getOrientation()[2],
            robot_state_handle.getOrientation()[3]);
    // atual base twist
    LinearVelocity base_actual_linear_velocity = LinearVelocity(robot_state_handle.getLinearVelocity()[0],
            robot_state_handle.getLinearVelocity()[1],
            robot_state_handle.getLinearVelocity()[2]);
    LocalAngularVelocity base_actual_angular_velocity = LocalAngularVelocity(robot_state_handle.getAngularVelocity()[0],
            robot_state_handle.getAngularVelocity()[0],
            robot_state_handle.getAngularVelocity()[0]);
    // 1 save base actual state into robot_state_
    robot_state_->setPositionWorldToBaseInWorldFrame(base_actual_position);
    robot_state_->setOrientationBaseToWorld(base_actual_orientation);
    robot_state_->setLinearVelocityBaseInWorldFrame(base_actual_linear_velocity);
    robot_state_->setAngularVelocityBaseInBaseFrame(base_actual_angular_velocity);

    // actual joint position and velocity
    sensor_msgs::JointState joint_command, joint_actual;
    joint_command.effort.resize(12);
    joint_command.position.resize(12);
    joint_command.name.resize(12);
    joint_actual.name.resize(12);
    joint_actual.position.resize(12);
    joint_actual.velocity.resize(12);
    joint_actual.effort.resize(12);

    free_gait::JointPositions all_joint_positions;
    free_gait::JointVelocities all_joint_velocities;
    free_gait::JointEfforts all_joint_efforts;

    for(unsigned int i = 0; i < 12; i++)
    {
        all_joint_positions(i) = robot_state_handle.getJointPositionRead()[i];
        all_joint_velocities(i) = robot_state_handle.getJointVelocityRead()[i];
        all_joint_efforts(i) = robot_state_handle.getJointEffortRead()[i];
        joint_actual.position[i] = all_joint_positions(i);
        joint_actual.velocity[i] = all_joint_velocities(i);
        joint_actual.effort[i] = all_joint_efforts(i);
    }
    // 2 save joint actual state into robot_state_
    robot_state_->setJointPositions(all_joint_positions);
    robot_state_->setAllJointVelocities(all_joint_velocities);

    // Computing reference base acceleration
    Position postionErrorInWorldFrame = base_desired_position - base_actual_position;
    LinearVelocity linearVelocityInWorldFrame = base_desired_linear_velocity - base_actual_linear_velocity;
    base_reference_linear_acceleration = base_desired_linear_acceleration
            + LinearAcceleration(Kr.cwiseProduct(postionErrorInWorldFrame.toImplementation()))
            + LinearAcceleration(Dr.cwiseProduct(linearVelocityInWorldFrame.toImplementation()));
    Eigen::Vector3d orientationErrorInBaseFrame = base_desired_rotation.boxMinus(base_actual_orientation);
    LocalAngularVelocity angularVelocityErrorInBaseFram = base_actual_angular_velocity;
    base_reference_angular_acceleration = base_desired_angular_acceleration
            + AngularAcceleration(Kw.cwiseProduct(orientationErrorInBaseFrame))
            + AngularAcceleration(Dw.cwiseProduct(angularVelocityErrorInBaseFram.toImplementation()));

    // set foot contact state if update
    contactStateMachine();

    // set optimazation problem
    prepareLegLoading();
    prepareOptimazation(base_reference_linear_acceleration, base_reference_angular_acceleration);
    addPhysicalConsistencyConstraints();
    addStanceAndSwingLegConstraints();
    addFrictionConeConstraints();
    addTorqueLimitConstraints();
    solveOptimazation();

    // output optimazation result
    JointTorques joint_torque;
    joint_torque = getFeedforwardJointTorque();




}

void WholeBodyController::desiredCommandCallback(const free_gait_msgs::RobotStateConstPtr &robot_state_msg)
{

    base_desired_position = Position(robot_state_msg->base_pose.pose.pose.position.x,
                                      robot_state_msg->base_pose.pose.pose.position.y,
                                      robot_state_msg->base_pose.pose.pose.position.z);
    base_desired_rotation = RotationQuaternion(robot_state_msg->base_pose.pose.pose.orientation.w,
                                                          robot_state_msg->base_pose.pose.pose.orientation.x,
                                                          robot_state_msg->base_pose.pose.pose.orientation.y,
                                                          robot_state_msg->base_pose.pose.pose.orientation.z);
    base_desired_linear_velocity = LinearVelocity(robot_state_msg->base_pose.twist.twist.linear.x,
                                                                 robot_state_msg->base_pose.twist.twist.linear.y,
                                                                 robot_state_msg->base_pose.twist.twist.linear.z);
    base_desired_angular_velocity = LocalAngularVelocity(robot_state_msg->base_pose.twist.twist.angular.x,
                                                                              robot_state_msg->base_pose.twist.twist.angular.y,
                                                                              robot_state_msg->base_pose.twist.twist.angular.z);
    base_desired_linear_acceleration = LinearAcceleration(robot_state_msg->base_desired_acceleration.linear.x,
                                                          robot_state_msg->base_desired_acceleration.linear.y,
                                                          robot_state_msg->base_desired_acceleration.linear.z);
    base_desired_angular_acceleration = AngularAcceleration(robot_state_msg->base_desired_acceleration.angular.x,
                                                            robot_state_msg->base_desired_acceleration.angular.y,
                                                            robot_state_msg->base_desired_acceleration.angular.z);

    // TODO add swing foot acceleration and desired joint position and velocity
}

void WholeBodyController::footContactsCallback(const sim_assiants::FootContactsConstPtr &foot_contacts)
{
    for(unsigned int i = 0; i < foot_contacts->foot_contacts.size(); i++)
    {
        auto contact = foot_contacts->foot_contacts[i];
        free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
        real_contact_.at(limb) = contact.is_contact;
        real_contact_force_.at(limb).z() = contact.contact_force.wrench.force.z;
    }
}

void WholeBodyController::contactStateMachine()
{
    for(unsigned int i = 0; i < 4; i++)
    {
        free_gait::LimbEnum limb = static_cast<free_gait::LimbEnum>(i);
        if(real_contact_.at(limb))
            robot_state_->setSupportLeg(limb, true);
        else
            robot_state_->setSupportLeg(limb, false);
    }
}

bool WholeBodyController::prepareLegLoading()
{
    nLegsInForceDistribution_ = 0;

    for (auto& legInfo : legInfos_)
    {
      if (robot_state_->isSupportLeg(legInfo.first))
        //if(real_contact_.at(legInfo.first))
      {
          // if is support leg
          legInfo.second.isPartOfForceDistribution_ = true;
          legInfo.second.isLoadConstraintActive_ = false;
          legInfo.second.indexInStanceLegList_ = nLegsInForceDistribution_;//! WSHY: 0,1,2,3
          legInfo.second.startIndexInVectorX_ = legInfo.second.indexInStanceLegList_ * footDof_;//! WSHY: 0,3,6,9
          nLegsInForceDistribution_++;
          legInfo.second.isLoadConstraintActive_ = true;

          legInfo.second.frictionCoefficient_ = 0.8;
      }
      else
      {
          legInfo.second.isPartOfForceDistribution_ = false;
          legInfo.second.isLoadConstraintActive_ = false;
      }
    }

    std::cout << "Loading stance legs number: " << nLegsInForceDistribution_ << std::endl;

    return true;
}

bool WholeBodyController::prepareOptimazation(const LinearAcceleration &linear_a,
                                              const AngularAcceleration &angular_a)
{
    n_ = footDof_ * nLegsInForceDistribution_;// 3 x stance leg number
    // set g matrix, base linear and angular acceleration
    const LinearAcceleration gravitationalAccelerationInWorldFrame = LinearAcceleration(0.0,0.0,-9.8);//torso_->getProperties().getGravity();
    LinearAcceleration gravitationalAccelerationInBaseFrame = robot_state_->getOrientationBaseToWorld().inverseRotate(gravitationalAccelerationInWorldFrame);
    g_.resize(6); // 6 x 1, linear and angular acceleration
    g_.segment(0, linear_a.toImplementation().size()) = robotMass_ * (linear_a.toImplementation() + gravitationalAccelerationInBaseFrame.toImplementation()); //TODO add gravity acceleration and mass
    g_.segment(linear_a.toImplementation().size(), angular_a.toImplementation().size()) = angular_a.toImplementation();
    G_.resize(6, 18 + n_);
    G_.setZero();
    Eigen::MatrixXd G_leftMatrix(6, 6);
    G_leftMatrix.setZero();
    G_leftMatrix.bottomRightCorner(3, 3).setIdentity();

    MatrixXd G_middleMatrix(6, 12);
    G_middleMatrix.setZero();

    MatrixXd G_rightMatrix(6, n_);
    G_rightMatrix.setZero();
    // replicate rows and cols of identity matrix 3x3 as the upper 3 rows of G_rightMatrix.
    G_rightMatrix.middleRows(0, 3) = (Matrix3d::Identity().replicate(1, nLegsInForceDistribution_)).sparseView();

    Eigen::MatrixXd G_pre(6, 18 + n_);
    G_pre.block(0, 0, 6, 6) = G_leftMatrix;
    G_pre.block(0, 6, 6, 12) = G_middleMatrix;
    G_pre.block(0, 18, 6, 3 * nLegsInForceDistribution_) = G_rightMatrix;
    G_.middleRows(0, 6) = G_pre.sparseView();
    std::cout << "G_ Matrix: " << std::endl << setprecision(3) << G_ << std::endl;
    std::cout << "g_ Matrix: " << std::endl << setprecision(3) << g_ << std::endl;
    Eigen::Matrix<double, 6, 1> baseAccelerationWeights_;
    baseAccelerationWeights_ << 0.5, 0.5, 0.5,
                                0.5, 0.5, 0.5;
    S_ = baseAccelerationWeights_.asDiagonal();
//    std::cout << "G_ Matrix: " << std::endl << setprecision(3) << S_ << std::endl;
    double torqueGroudForceWeight_ = 0.5;
    W_.setIdentity(18 + n_);
    W_ = W_ * torqueGroudForceWeight_;
//    std::cout << "G_ Matrix: " << std::endl << setprecision(3) << W_ << std::endl;
    x_.setZero(18 + n_);

    return true;
}

bool WholeBodyController::addPhysicalConsistencyConstraints()
{
    int rowIndex = C_.rows();
    Eigen::SparseMatrix<double, Eigen::RowMajor> C_temp(C_);
    C_.resize(rowIndex + 6, 18 + n_); // Mcom 6 x 6
    C_.middleRows(0, C_temp.rows()) = C_temp;
    c_.conservativeResize(rowIndex + 6);

    // Jacobian matrix
    Eigen::MatrixXd jacobian, floating_jac;
    Position base_position = robot_state_->getPositionWorldToBaseInWorldFrame();
    RotationQuaternion base_orientation = robot_state_->getOrientationBaseToWorld();
    JointPositions joint_position = robot_state_->getJointPositions();
    jacobian = computeJacobian(base_position, base_orientation, joint_position, limbset_); // define foot order when initialize class.
    floating_jac = getFloatingJacobian(); // 12 x 6
    std::cout << "Test floating jacobian: " << std::endl << floating_jac << std::endl;
    //todo floating jacobian for stacne leg

    // Nonlinear effect
    LinearVelocity base_linear_velocity = robot_state_->getLinearVelocityBaseInWorldFrame();
    LocalAngularVelocity base_angular_velocity = robot_state_->getAngularVelocityBaseInBaseFrame();
    JointVelocities joint_velocity = robot_state_->getJointVelocities();
    Eigen::VectorXd nonLinear_effect = computeNonlinearEffectsForce(base_position, base_orientation,
                                                                    base_linear_velocity, base_angular_velocity,
                                                                    joint_position, joint_velocity);
    Eigen::VectorXd floating_effect = getNonlinearBaseEffectsForce();

    // Joint space inertial matrix
    Eigen::MatrixXd inertial_mat = computeJointSpaceInertialMatrix(base_position, base_orientation, joint_position);
    Eigen::MatrixXd floating_mat = getInertialBaseMatrix(); // 6 x 6 pass data by class member variables
    Eigen::MatrixXd middle_matrix(6, 12);
    middle_matrix.setZero();
    Eigen::MatrixXd C_pre(6, 18 + n_);

    C_pre.block(0, 0, 6, 6) = floating_mat;
    C_pre.block(0, 6, 6, 12) = middle_matrix;
    C_pre.block(0, 18, 6, n_) = floating_jac.transpose();
    C_.middleRows(rowIndex, 6) = C_pre.sparseView();
    c_.segment(rowIndex, 6) = -floating_effect;
    std:: cout << "-------------------------------------------------------------" << std::endl;
    std::cout << "C_ Matrix Size After Adding Physical Consistencey: " << std::endl << C_.rows() << " x " << C_.cols() <<std::endl;
//    std::cout << setprecision(3) << C_ << std::endl;
    std::cout << fixed;
//    std::cout << std::setiosflags(ios::left) << setw(4) << std::endl;
    std::cout << C_ << std::endl;
    std::cout << "c_ Matrix Size After Adding Physical Consistencey: " << std::endl << c_.rows() << " x " << c_.cols() << std::endl;
    std::cout << setprecision(3) << c_ << std::endl;
    return true;

}

bool WholeBodyController::addStanceAndSwingLegConstraints()
{
    // add swing and stance condition contraint at the same time
    int rowIndex = C_.rows();
    //std::cout << "C_ first: " << std::endl << C_ << std::endl;
    Eigen::SparseMatrix<double, Eigen::RowMajor> C_temp(C_);
    C_.resize(rowIndex + 12, 18 + 12);
    C_.middleRows(0, C_temp.rows()) = C_temp;
    //std::cout << "C_ second: " << std::endl << C_ << std::endl;
    c_.conservativeResize(rowIndex + 12);
    // TODO judge the leg is support leg or swing leg.
    Position base_position = robot_state_->getPositionWorldToBaseInWorldFrame();
    RotationQuaternion base_orientation = robot_state_->getOrientationBaseToWorld();
    JointPositions joint_position = robot_state_->getJointPositions();

    Eigen::MatrixXd jacobian;
    jacobian = computeJacobian(base_position, base_orientation, joint_position, limbset_);

    Eigen::MatrixXd C_pre(12, 18);
    C_pre.block(0, 0, 12, 18) = jacobian;
    C_.middleRows(rowIndex, 12) = C_pre.sparseView();

    LinearVelocity base_linear_velocity = robot_state_->getLinearVelocityBaseInWorldFrame();
    LocalAngularVelocity base_angular_velocity = robot_state_->getAngularVelocityBaseInBaseFrame();
    JointVelocities joint_velocity = robot_state_->getJointVelocities();
    // Computing the contact Jacd*Qd
    LimbVectorXd contact_jacd_qd;
    computeJdotQdot(contact_jacd_qd, base_position, base_orientation,
                    base_linear_velocity, base_angular_velocity,
                    joint_position, joint_velocity, limbset_);
    Eigen::VectorXd c_row(12);
    int row_index = 0;
    for(LimbVectorXd::iterator it = contact_jacd_qd.begin(); it != contact_jacd_qd.end(); ++it)
    {
        Eigen::VectorXd jacd_qd = it->second;
        c_row.segment(row_index, 3) = jacd_qd;
        row_index = row_index + 3;
    }

    for(auto& legInfo : legInfos_)
    {
        if(legInfo.second.isPartOfForceDistribution_)
        {
            // stance leg, c = - Jstdot*qdot
            std::cout << "stance leg " << std::endl;
            c_.segment(rowIndex + legInfo.second.startIndexInVectorX_, 3) = -c_row.segment(legInfo.second.startIndexInVectorX_, 3);
        }
        else
        {
            // swing leg, c = vdot - Jswdot*qdot
            std::cout << "swing leg " << std::endl;
            Eigen::Vector3d foot_acceleration = Eigen::Vector3d((robot_state_->getTargetFootAccelerationInBaseForLimb(legInfo.first)).toImplementation());
            c_.segment(rowIndex + legInfo.second.startIndexInVectorX_, 3) = foot_acceleration - c_row.segment(legInfo.second.startIndexInVectorX_, 3);
        }
    }
    // check matrix size
    std:: cout << "-------------------------------------------------------------" << std::endl;
    std::cout << "C_ Matrix Size After Adding Stance and Swing Constraint: " << std::endl << C_.rows() << " x " << C_.cols() <<std::endl;
    std::cout << C_ << std::endl;
    std::cout << "c_ Matrix Size After Adding Stance and Swing Constraint: " << std::endl << c_.rows() << " x " << c_.cols() << std::endl;
    std::cout << c_ << std::endl;

    return true;

}

bool WholeBodyController::addFrictionConeConstraints()
{
    ROS_INFO("To add friction cone constraints");
    // IIT Paper: High-slope Terrain Locomotion for Torque-Controlled Quadruped Robots
    int nDirections = 4;
    int nConstraints = nDirections * nLegsInForceDistribution_;
    int rowIndex = D_.rows();
    Eigen::SparseMatrix<double, Eigen::RowMajor> D_temp(D_); // TODO replace with conservativeResize (available in Eigen 3.2)
    D_.resize(rowIndex + nConstraints, 18 + n_);
    D_.middleRows(0, D_temp.rows()) = D_temp;
    d_.conservativeResize(rowIndex + nConstraints);
    f_.conservativeResize(rowIndex + nConstraints);

    const RotationQuaternion& orientationWorldToBase = robot_state_->getOrientationBaseToWorld().inverted();
    const RotationQuaternion orientationControlToBase = robot_state_->getOrientationBaseToWorld().inverted();

    for (auto& legInfo : legInfos_)
    {
        // if is stance leg
      if (legInfo.second.isPartOfForceDistribution_)
      {
        MatrixXd D_rows = MatrixXd::Zero(nDirections, 18 + n_);
//        Position positionWorldToFootInWorldFrame = robot_state_->getPositionWorldToFootInWorldFrame(legInfo.first);
        Vector footContactNormalInWorldFrame;
        footContactNormalInWorldFrame = robot_state_->getSurfaceNormal(legInfo.first);
        Vector footContactNormalInBaseFrame = orientationWorldToBase.rotate(footContactNormalInWorldFrame);

        const Vector3d normalDirection = footContactNormalInBaseFrame.toImplementation();

        // for logging
        legInfo.second.normalDirectionOfFrictionPyramidInWorldFrame_ = Vector(footContactNormalInWorldFrame);
        // The choose the first tangential to lie in the XZ-plane of the base frame.
        // This is the same as the requirement as
        // 1) firstTangential perpendicular to normalDirection,
        // 2) firstTangential perpendicular to normal of XZ-plane of the base frame,
        // 3) firstTangential has unit norm.

        Vector3d vectorY = Vector3d::UnitY();
        Vector3d firstTangentialInBaseFrame = orientationControlToBase.rotate(vectorY);
        Vector3d firstTangential = normalDirection.cross(firstTangentialInBaseFrame).normalized();
        std::cout << "First tangential: " << firstTangential << std::endl;
        // logging
        legInfo.second.firstDirectionOfFrictionPyramidInWorldFrame_ = Vector(orientationWorldToBase.inverseRotate(firstTangential));
        // The second tangential is perpendicular to the normal and the first tangential.
        Vector3d secondTangential = normalDirection.cross(firstTangential).normalized();

        // logging
        legInfo.second.secondDirectionOfFrictionPyramidInWorldFrame_ = Vector(orientationWorldToBase.inverseRotate(secondTangential));

        // First tangential, positive
        D_rows.block(0, legInfo.second.startIndexInVectorX_ + 18, 1, footDof_) = // 0 3 6 9
            legInfo.second.frictionCoefficient_ * normalDirection.transpose() + firstTangential.transpose();
        // First tangential, negative
        D_rows.block(1, legInfo.second.startIndexInVectorX_ + 18, 1, footDof_) =
            legInfo.second.frictionCoefficient_ * normalDirection.transpose() - firstTangential.transpose();
        // Second tangential, positive
        D_rows.block(2, legInfo.second.startIndexInVectorX_ + 18, 1, footDof_) =
            legInfo.second.frictionCoefficient_ * normalDirection.transpose() + secondTangential.transpose();
        // Second tangential, negative
        D_rows.block(3, legInfo.second.startIndexInVectorX_ + 18, 1, footDof_) =
            legInfo.second.frictionCoefficient_ * normalDirection.transpose() - secondTangential.transpose();

        D_.middleRows(rowIndex, nDirections) = D_rows.sparseView();
        d_.segment(rowIndex, nDirections) =  VectorXd::Constant(nDirections, 0.0);
        f_.segment(rowIndex, nDirections) = VectorXd::Constant(nDirections, std::numeric_limits<double>::max());

        rowIndex = rowIndex + nDirections; // 0 4 8 12
      }
    }

    std:: cout << "-------------------------------------------------------------" << std::endl;
    std::cout << "D_ Matrix Size After Adding Friction Cone Constraint: " << std::endl << D_.rows() << " x " << D_.cols() <<std::endl;
    std::cout << D_ << std::endl;
    std::cout << "d_ Matrix Size After Adding Friction Cone Constraint: " << std::endl << d_.rows() << " x " << d_.cols() << std::endl;
    std::cout << d_ << std::endl;
    std::cout << "f_ Matrix Size After Adding Friction Cone Constraint: " << std::endl << f_.rows() << " x " << f_.cols() << std::endl;
    //std::cout << f_ << std::endl;

    return true;
}
bool WholeBodyController::addTorqueLimitConstraints()
{
    // Resize matrxi size to add new constraints.
    int row_Index = D_.rows();
    Eigen::SparseMatrix<double, Eigen::RowMajor> D_temp(D_);
    D_.resize(row_Index + 12, n_ + 18);
    D_.middleRows(0, D_temp.rows()) = D_temp;
    d_.conservativeResize(row_Index + 12);
    f_.conservativeResize(row_Index + 12);

    // joint space inertial matrix
    Position base_position = robot_state_->getPositionWorldToBaseInWorldFrame();
    RotationQuaternion base_orientation = robot_state_->getOrientationBaseToWorld();
    JointPositions joint_position = robot_state_->getJointPositions();
    Eigen::MatrixXd inertial_mat = computeJointSpaceInertialMatrix(base_position, base_orientation, joint_position);
    Eigen::MatrixXd joint_mat = getInertialJointMatrix(); // 12 x 12 pass data by class member variables
    // Jacobian for stance leg.
    Eigen:: MatrixXd jacobian;
    jacobian = computeJacobian(base_position, base_orientation, joint_position, limbset_);
    Eigen::MatrixXd fixed_jac(12, 12); // 12 x 12
    fixed_jac = getFixedJacobian(); // all joint jacobian;
    Eigen::MatrixXd jac_st(12, n_);// 12 x n_;

    for(auto& legInfo : legInfos_)// TODO limb order is ?
    {
        if(legInfo.second.isPartOfForceDistribution_)
        {
            jac_st.block(0, legInfo.second.startIndexInVectorX_, 12, 3);
        }
    }
    Eigen::MatrixXd D_pre(12, 18 + n_);
    D_pre.setZero();
    D_pre.block(0, 6, 12, 12) = joint_mat;
    D_pre.block(0, 18, 12, n_) = jac_st;
    D_.middleRows(row_Index, 12) = D_pre.sparseView();

    LinearVelocity base_linear_velocity = robot_state_->getLinearVelocityBaseInWorldFrame();
    LocalAngularVelocity base_angular_velocity = robot_state_->getAngularVelocityBaseInBaseFrame();
    JointVelocities joint_velocity = robot_state_->getJointVelocities();
    Eigen::VectorXd nonLinear_effect = computeNonlinearEffectsForce(base_position, base_orientation,
                                                                    base_linear_velocity, base_angular_velocity,
                                                                    joint_position, joint_velocity);
    Eigen::VectorXd fixed_effect(12);
    Eigen::VectorXd minTorque(12);
    minTorque << 0, 0, 0,
                 0, 0, 0,
                 0, 0, 0,
                 0, 0, 0;
    Eigen::VectorXd maxTorque(12);
    maxTorque << 50, 50, 50,
                 50, 50, 50,
                 50, 50, 50,
                 50, 50, 50;
    fixed_effect = getNonlinearJointEffectsForce();
    ROS_INFO("test");
    d_.segment(row_Index, 12) = fixed_effect + minTorque;
    f_.segment(row_Index, 12) = fixed_effect + maxTorque;
    ROS_INFO("test");
    std:: cout << "-------------------------------------------------------------" << std::endl;
    std::cout << "D_ Matrix Size After Adding Torque Limit Constraint: " << std::endl << D_.rows() << " x " << D_.cols() <<std::endl;
    std::cout << D_ << std::endl;
    std::cout << "d_ Matrix Size After Adding Torque Limit Constraint: " << std::endl << d_.rows() << " x " << d_.cols() << std::endl;
    std::cout << d_ << std::endl;
    return true;
}

////bool WholeBodyController::addJointKinematicsLimitConstraints()
////{

////}

const Eigen::MatrixXd WholeBodyController::computeJointSpaceInertialMatrix(Position &base_pos,
                                                                           RotationQuaternion &base_orientation,
                                                                           JointPositions &joint_pos)
{
    // base pose conversion
    dwl::rbd::Vector6d base_position;
    Eigen::MatrixXd base_position_(base_pos.toImplementation());
    base_position.block(0, 0, 3, 1) = base_position_;
    // Ration from rotationQuaternion
    const EulerAnglesXyz base_orientation_ =EulerAnglesXyz(base_orientation);
    Eigen::MatrixXd base_rotation(base_orientation_.toImplementation());
    base_position.block(3, 0, 3, 1) = base_rotation;

    // joint position conversion
    Eigen::VectorXd joint_position = Eigen::VectorXd(joint_pos.toImplementation());
    // compute joint space inertial matrix 18 x 18
    inertial_matrix = wdyn->computeJointSpaceInertiaMatrix(base_position, joint_position);
    std::cout << "inertial matrix size: " << inertial_matrix.rows() << " x " << inertial_matrix.cols() << std::endl;
    return inertial_matrix;

}

const Eigen::MatrixXd WholeBodyController::getInertialBaseMatrix()
{
    inertial_base_matrix = inertial_matrix.block(0, 0, 6, 6);
    return inertial_base_matrix;
}

const Eigen::MatrixXd WholeBodyController::getInertialJointMatrix()
{
    inertial_joint_matrix = inertial_matrix.block(6, 6, 12, 12);
    return inertial_joint_matrix;
}

const Eigen::VectorXd WholeBodyController::computeNonlinearEffectsForce(Position &base_pos, RotationQuaternion &base_orientation,
                                                                        LinearVelocity &base_linear_vel, LocalAngularVelocity &base_angular_vel,
                                                                        JointPositions &joint_pos, JointVelocities &joint_vel)
{
    // base pose conversion
    dwl::rbd::Vector6d base_position;
    Eigen::MatrixXd base_position_(base_pos.toImplementation());
    base_position.block(0, 0, 3, 1) = base_position_;
    // Ration from rotationQuaternion
    const EulerAnglesXyz base_orientation_ =EulerAnglesXyz(base_orientation);
    Eigen::MatrixXd base_rotation(base_orientation_.toImplementation());
    base_position.block(3, 0, 3, 1) = base_rotation;
    // base velocity conversion
    dwl::rbd::Vector6d base_velocity;
    Eigen::MatrixXd base_linear_velocity(base_linear_vel.toImplementation());
    Eigen::MatrixXd base_angular_velocity(base_angular_vel.toImplementation());
    base_velocity.block(0, 0, 3, 1) = base_linear_velocity;
    base_velocity.block(3, 0, 3, 1) = base_angular_velocity;
    // joint position conversion
    Eigen::VectorXd joint_position = Eigen::VectorXd(joint_pos.toImplementation());
    // joint velocity conversion
    Eigen::VectorXd joint_velocity = Eigen::VectorXd(joint_vel.toImplementation());

    // compute nonliear effect force, 18 x 1
    nonlinear_effect = wdyn->computeNonlinearEffectsForce(base_position, joint_position, base_velocity, joint_velocity);

    return nonlinear_effect;
}

const Eigen::VectorXd WholeBodyController::getNonlinearBaseEffectsForce()
{
    nonlinear_base_effect = nonlinear_effect.segment(0, 6); // 6 x 1
    return nonlinear_base_effect;
}

const Eigen::VectorXd WholeBodyController::getNonlinearJointEffectsForce()
{
    nonlinear_joint_effect = nonlinear_effect.segment(6, 12); // 12 x 1
    return nonlinear_joint_effect;
}

const Eigen::MatrixXd WholeBodyController::computeLegJacobian(Position &base_pos, RotationQuaternion &base_orientation,
                                                           JointPositions &joint_pos, free_gait::LimbEnum limb)
{
    // base pose conversion
    dwl::rbd::Vector6d base_position;
    Eigen::MatrixXd base_position_(base_pos.toImplementation());
    base_position.block(0, 0, 3, 1) = base_position_;
    // Ration from rotationQuaternion
    const EulerAnglesXyz base_orientation_ =EulerAnglesXyz(base_orientation);
    Eigen::MatrixXd base_rotation(base_orientation_.toImplementation());
    base_position.block(3, 0, 3, 1) = base_rotation;
    // joint position conversion
    Eigen::VectorXd joint_position = Eigen::VectorXd(joint_pos.toImplementation());

    int index = static_cast<int>(limb);
    string foot_name;
    // map foot name in free_gait to the name in RBDL;
    if(index == 0)
        foot_name = "lf_foot";
    if(index == 1)
        foot_name = "rf_foot";
    if(index == 2)
        foot_name = "rh_foot";
    if(index == 3)
        foot_name = "lh_foot";

    // get foot id in kinematic tree
//    if(body_id_.count(foot_name) > 0)
//    {
    std::cout << "foot name: " << foot_name << std::endl;
    int foot_id = body_id_.find(foot_name)->second;
//    }
    std::cout << "foot_id: " << foot_id << std::endl;
    Eigen::VectorXd q = fbs.toGeneralizedJointState(base_position, joint_position);
    Eigen::MatrixXd jac(Eigen::MatrixXd::Zero(3, 18)); // only translation part

    dwl::rbd::computePointJacobian(fbs.getRBDModel(),
                                   q, foot_id,
                                   Eigen::Vector3d::Zero(),
                                   jac, false);
    return jac;

}

const Eigen::MatrixXd WholeBodyController::computeJacobian(Position &base_pos, RotationQuaternion &base_orientation, JointPositions &joint_pos, LimbSelector limb_set)
{
    // base pose conversion
    dwl::rbd::Vector6d base_position;
    Eigen::MatrixXd base_position_(base_pos.toImplementation());
    base_position.block(0, 0, 3, 1) = base_position_;
    // Ration from rotationQuaternion
    const EulerAnglesXyz base_orientation_ =EulerAnglesXyz(base_orientation);
    Eigen::MatrixXd base_rotation(base_orientation_.toImplementation());
    base_position.block(3, 0, 3, 1) = base_rotation;
    // joint position conversion
    Eigen::VectorXd joint_position = Eigen::VectorXd(joint_pos.toImplementation());

    dwl::rbd::BodySelector body_set;
    body_set = limb_set;
    ROS_INFO("start computing jacobian");
    wkin.computeJacobian(jacobian, base_position, joint_position, body_set, dwl::rbd::Linear);

    return jacobian;
}

const Eigen::MatrixXd WholeBodyController::getFloatingJacobian()
{
    Eigen::MatrixXd floating_jac(12, 6);
    floating_jac = jacobian.block(0, 0, 12, 6);
    return floating_jac;
}

const Eigen::MatrixXd WholeBodyController::getFixedJacobian()
{
    Eigen::MatrixXd fixed_jac(12, 12);
    fixed_jac = jacobian.block(0, 6, 12, 12);
    return fixed_jac;
}
// choose each leg's JdotQdot according to its name.
void WholeBodyController::computeJdotQdot(LimbVectorXd &jacd_qd,
                                          Position &base_pos, RotationQuaternion &base_orientation,
                                          LinearVelocity &base_linear_vel, LocalAngularVelocity &base_angular_vel,
                                          JointPositions &joint_pos, JointVelocities &joint_vel, LimbSelector limbset)
{
    // base pose conversion
    dwl::rbd::Vector6d base_position;
    Eigen::MatrixXd base_position_(base_pos.toImplementation());
    base_position.block(0, 0, 3, 1) = base_position_;
    // Ration from rotationQuaternion
    const EulerAnglesXyz base_orientation_ =EulerAnglesXyz(base_orientation);
    Eigen::MatrixXd base_rotation(base_orientation_.toImplementation());
    base_position.block(3, 0, 3, 1) = base_rotation;
    // base velocity conversion
    dwl::rbd::Vector6d base_velocity;
    Eigen::MatrixXd base_linear_velocity(base_linear_vel.toImplementation());
    Eigen::MatrixXd base_angular_velocity(base_angular_vel.toImplementation());
    base_velocity.block(0, 0, 3, 1) = base_linear_velocity;
    base_velocity.block(3, 0, 3, 1) = base_angular_velocity;
    // joint position conversion
    Eigen::VectorXd joint_position = Eigen::VectorXd(joint_pos.toImplementation());
    // joint velocity conversion
    Eigen::VectorXd joint_velocity = Eigen::VectorXd(joint_vel.toImplementation());

    wkin.computeJdotQdot(jacd_qd, base_position, joint_position, base_velocity, joint_velocity, limbset, dwl::rbd::Linear);
}

bool WholeBodyController::solveOptimazation()
{
    ROS_INFO("Starting sloving...");
    if(!ooqpei::QuadraticProblemFormulation::solve(G_, S_, g_, W_, C_, c_, D_, d_, f_, x_))
    {
        ROS_ERROR("solve failed");
        return false;
    }

    ROS_INFO("solve successfully");
    std::cout << x_ << std::endl;

    // save result into variables 6 + 12 + n_
    base_optimal_linear_acceleration = LinearAcceleration(x_.segment(0, 3));
    base_optimal_angular_acceleration = AngularAcceleration(x_.segment(3, 3));
    joint_optimal_acceleration = JointAccelerations(x_.segment(6, 12));


    for(auto& legInfo : legInfos_)
    {
        if(legInfo.second.isPartOfForceDistribution_)
        {
            ground_force_.insert(make_pair(legInfo.first, Force(x_.segment(legInfo.second.startIndexInVectorX_ + 18, 3))));
            std::cout << "force :" << Force(x_.segment(legInfo.second.startIndexInVectorX_ + 18, 3)) << std::endl;
        }
    }

    return true;
}

const JointTorques WholeBodyController::computeFloatingBaseInverseDynamics(Position &base_pos, RotationQuaternion &base_orientation,
                                                                           LinearVelocity &base_linear_vel, LocalAngularVelocity &base_angular_vel,
                                                                           JointPositions &joint_pos, JointVelocities &joint_vel,
                                                                           LinearAcceleration &base_linear_acc, AngularAcceleration &base_angular_acc,
                                                                           JointAccelerations &joint_acc, ExternalForece &ext_force)
{
    // base pose conversion
    dwl::rbd::Vector6d base_position;
    Eigen::MatrixXd base_position_(base_pos.toImplementation());
    base_position.block(0, 0, 3, 1) = base_position_;
    // Ration from rotationQuaternion
    const EulerAnglesXyz base_orientation_ =EulerAnglesXyz(base_orientation);
    Eigen::MatrixXd base_rotation(base_orientation_.toImplementation());
    base_position.block(3, 0, 3, 1) = base_rotation;
    // base velocity conversion
    dwl::rbd::Vector6d base_velocity;
    Eigen::MatrixXd base_linear_velocity(base_linear_vel.toImplementation());
    Eigen::MatrixXd base_angular_velocity(base_angular_vel.toImplementation());
    base_velocity.block(0, 0, 3, 1) = base_linear_velocity;
    base_velocity.block(3, 0, 3, 1) = base_angular_velocity;
    // joint position conversion
    Eigen::VectorXd joint_position = Eigen::VectorXd(joint_pos.toImplementation());
    // joint velocity conversion
    Eigen::VectorXd joint_velocity = Eigen::VectorXd(joint_vel.toImplementation());
    // base acceleration conversion
    dwl::rbd::Vector6d base_acceleration;
    Eigen::MatrixXd base_acceleration_l(base_linear_acc.toImplementation());
    base_acceleration.block(0, 0, 3, 1) = base_acceleration_l;
    Eigen::MatrixXd base_acceleration_w(base_angular_acc.toImplementation());
    base_acceleration.block(3, 0, 3, 1) = base_acceleration_w;
    // joint acceleration conversion
    Eigen::VectorXd joint_acceleration = Eigen::VectorXd(joint_acc.toImplementation());
    // external force conversion
    dwl::rbd::BodyVector6d grf_;
    Eigen::Vector3d grf_1 = Vector3d(ext_force.at(free_gait::LimbEnum::LF_LEG).toImplementation());
    grf_["lf_foot"] << 0, 0, 0, grf_1[0], grf_1[1], grf_1[2];
    Eigen::Vector3d grf_2 = Vector3d(ext_force.at(free_gait::LimbEnum::RF_LEG).toImplementation());
    grf_["rf_foot"] << 0, 0, 0, grf_1[0], grf_1[1], grf_1[2];
    Eigen::Vector3d grf_3 = Vector3d(ext_force.at(free_gait::LimbEnum::LH_LEG).toImplementation());
    grf_["lh_foot"] << 0, 0, 0, grf_1[0], grf_1[1], grf_1[2];
    Eigen::Vector3d grf_4 = Vector3d(ext_force.at(free_gait::LimbEnum::RH_LEG).toImplementation());
    grf_["rh_foot"] << 0, 0, 0, grf_1[0], grf_1[1], grf_1[2];
    // joint torque to be calculated.
    Eigen::VectorXd joint_force;
    wdyn->computeFloatingBaseInverseDynamics(base_acceleration, joint_force,
                                             base_position, joint_position,
                                             base_velocity, joint_velocity,
                                             joint_acceleration, grf_);

    JointTorques joint_torque = JointTorques(joint_force);
    return joint_torque;
}


const JointTorques WholeBodyController::getFeedforwardJointTorque()
{
    // position
    Position base_position = robot_state_->getPositionWorldToBaseInWorldFrame();
    RotationQuaternion base_orientation = robot_state_->getOrientationBaseToWorld();
    JointPositions joint_position = robot_state_->getJointPositions();
    // velocity
    LinearVelocity base_linear_velocity = robot_state_->getLinearVelocityBaseInWorldFrame();
    LocalAngularVelocity base_angular_velocity = robot_state_->getAngularVelocityBaseInBaseFrame();
    JointVelocities joint_velocity = robot_state_->getJointVelocities();
    // acceleration
    LinearAcceleration base_optimal_linear_acceleration_ = base_optimal_linear_acceleration;
    AngularAcceleration base_optimal_angular_acceleration_ = base_optimal_angular_acceleration;
    JointAccelerations joint_optimal_acceleration_ = joint_optimal_acceleration;

    ExternalForece optimal_ground_force_ = ground_force_;

    JointTorques joint_torque_;
    joint_torque_ = computeFloatingBaseInverseDynamics(base_position, base_orientation,
                                                       base_linear_velocity, base_angular_velocity,
                                                       joint_position, joint_velocity,
                                                       base_optimal_linear_acceleration_, base_optimal_angular_acceleration_,
                                                       joint_optimal_acceleration_, optimal_ground_force_);
    return joint_torque_;
}

}

//PLUGINLIB_EXPORT_CLASS(balance_controller::WholeBodyController, controller_interface::ControllerBase)
