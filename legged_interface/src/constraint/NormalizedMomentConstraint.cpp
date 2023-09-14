#include "legged_interface/constraint/NormalizedMomentConstraint.h"

namespace ocs2
{
    namespace legged_robot
    {
        NormalizedMomentConstraint::NormalizedMomentConstraint(scalar_t lambda_l, scalar_t lambda_r, scalar_t h_max)
            : StateConstraint(ConstraintOrder::Linear), lambda_l_(lambda_l), lambda_r_(lambda_r), h_max_(h_max)
        {
        }

        NormalizedMomentConstraint::NormalizedMomentConstraint *clone() const
        {
            return new NormalizedMomentConstraint(*this);
        }

        /** Get the size of the constraint vector at given time */
        size_t NormalizedMomentConstraint::getNumConstraints(scalar_t time) const
        {
            return 1;
        }

        /** Get the constraint vector value */
        vector_t NormalizedMomentConstraint::getValue(scalar_t time, const vector_t &state, const PreComputation &preComp) const
        {
            vector_t value(1);
            value << -1 * lambda_l_ * (state(0) * state(0) + state(1) * state(1)) - lambda_r_ * (state(3) * state(3)) + h_max_;
            return value;
        }

        /** Get the constraint linear approximation */
        VectorFunctionLinearApproximation NormalizedMomentConstraint::getLinearApproximation(scalar_t time, const vector_t &state,
                                                                                             const PreComputation &preComp) const
        {
            VectorFunctionLinearApproximation linearApproximation;
            linearApproximation.f = getValue(time, state, preComp);
            linearApproximation.dfdx = vector_t::Zero(1, state.size());
            linearApproximation.dfdx(0) = -2 * lambda_l_ * state(0);
            linearApproximation.dfdx(1) = -2 * lambda_l_ * state(1);
            linearApproximation.dfdx(3) = -2 * lambda_r_ * state(3);
            return linearApproximation;
        }
        ~NormalizedMomentConstraint()
        {
        }
    } // namespace legged_robot

} // namespace ocs2
