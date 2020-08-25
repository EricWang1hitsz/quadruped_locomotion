/********************************************************

@File  quadruped_model.

@Description Kinematic and dynamic model of quadruped robot.

@Author  Eric Wang

@Date:   2020-01-10

@Association: Harbin Institute of Technology, Shenzhen.

*********************************************************/


#pragma once

#include <towr/models/kinematic_model.h>
#include <towr/models/dynamic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>


namespace towr {

class quadrupedKinematicModel : public KinematicModel
{
public:
    /**
     * @brief quadrupedKinematicModel
     */
    quadrupedKinematicModel() : KinematicModel(4)
    {
        const double x_nominal_b = 0.425;
        const double y_nominal_b = 0.305;
        const double z_nominal_b = -0.46;

        // nominal stance in base frame.
        nominal_stance_.at(LF) << x_nominal_b, y_nominal_b, z_nominal_b;
        nominal_stance_.at(RF) << x_nominal_b, -y_nominal_b, z_nominal_b;
        nominal_stance_.at(LH) << -x_nominal_b, y_nominal_b, z_nominal_b;
        nominal_stance_.at(RH) << -x_nominal_b, -y_nominal_b, z_nominal_b;

        // maxmum dev from nominal stance.
        max_dev_from_nominal_ << 0.15, 0.10, 0.10;
    }
};
/**
 * @brief The quadrupedDynamicModel class
 */
class quadrupedDynamicModel : public SingleRigidBodyDynamics
{
public:
    quadrupedDynamicModel()
        :SingleRigidBodyDynamics(27,  // robot mass.
                                 //! TODO(EricWang): How to get the inertia matrix.
                                 0.946438, 1.94478, 2.01835, 0.000938112, -0.00595386, -0.00146328, // the 3x3 Inertia matrix.
                                 4){} // number of endeffectors.
};

}
