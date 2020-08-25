/********************************************************

@File  quadruped_model.

@Description Convert constraint to a cost function

@Author  Eric Wang

@Date:   2020-07-14

@Association: Harbin Institute of Technology, Shenzhen.

*********************************************************/

#include <towr/costs/square_cost.h>

namespace towr {

SquaredCost::SquaredCost(const ifopt::ConstraintSet::Ptr& constraint)
  : SquaredCost(constraint, Eigen::VectorXd::Ones(constraint->GetRows()))
{
}

SquaredCost::SquaredCost(const ifopt::ConstraintSet::Ptr& constraint, const Eigen::Ref<const Eigen::VectorXd>& weights)
  : CostTerm(constraint->GetName() + "_squared_cost")
  , constraint_(constraint)
  , n_constraints_(constraint->GetRows())
  , weights_(weights)
{
  // Calculate targets - Average the upper and lower bounds
  targets_.resize(n_constraints_);
  std::vector<ifopt::Bounds> bounds = constraint_->GetBounds();
  for (Eigen::Index ind = 0; ind < n_constraints_; ind++)
  {
    targets_(ind) = (bounds[static_cast<std::size_t>(ind)].upper_ + bounds[static_cast<std::size_t>(ind)].lower_) / 2.;
  }
}

double SquaredCost::GetCost() const
{
  Eigen::VectorXd values = constraint_->GetValues();
  Eigen::VectorXd error = values - targets_;
  double cost = error.transpose() * weights_.asDiagonal() * error;
  return cost;
}

void SquaredCost::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Get a Jacobian block the size necessary for the constraint
  Jacobian cnt_jac_block;
  int var_size = 0;
  for (const auto& vars : GetVariables()->GetComponents())
  {
    if (vars->GetName() == var_set)
      var_size = vars->GetRows();
  }
  assert(var_size > 0);
  cnt_jac_block.resize(constraint_->GetRows(), var_size);

  // Get the Jacobian Block from the constraint
  constraint_->FillJacobianBlock(var_set, cnt_jac_block);

  // Apply the chain rule. See doxygen for this class
  Eigen::VectorXd error = constraint_->GetValues() - targets_;
  jac_block = 2 * error.transpose().sparseView() * weights_.asDiagonal() * cnt_jac_block;
}


}
