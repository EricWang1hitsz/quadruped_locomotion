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

class laikagoKinematicModel : public KinematicModel
{
public:
    /**
     * @brief quadrupedKinematicModel
     */
    laikagoKinematicModel() : KinematicModel(4)
    {
        const double x_nominal_b = 0.21935;
        const double y_nominal_b = 0.1245;
        const double z_nominal_b = -0.3536;
//        const double z_nominal_b = -0.20;

        // nominal stance in base frame.
        nominal_stance_.at(LF) << x_nominal_b, y_nominal_b, z_nominal_b;
        nominal_stance_.at(RF) << x_nominal_b, -y_nominal_b, z_nominal_b;
        nominal_stance_.at(LH) << -x_nominal_b, y_nominal_b, z_nominal_b;
        nominal_stance_.at(RH) << -x_nominal_b, -y_nominal_b, z_nominal_b;

        // maxmum dev from nominal stance.
        max_dev_from_nominal_ << 0.10, 0.10, 0.05;
    }
};
/**
 * @brief The quadrupedDynamicModel class
 */
class laikagoDynamicModel : public SingleRigidBodyDynamics
{
public:
    laikagoDynamicModel()
        :SingleRigidBodyDynamics(13.733,  // robot mass.
                                 //! TODO(EricWang): How to get the inertia matrix.
                                 0.073348887, 0.250684593, 0.254469458, 0.00030338, 0.001918218, -0.000075402, // the 3x3 Inertia matrix.
                                 4){} // number of endeffectors.
};

}
