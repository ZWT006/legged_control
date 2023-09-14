
#pragma once
#include "ocs2_core/constraint/StateConstraint.h"

namespace ocs2
{
    namespace legged_robot
    {
        class NormalizedMomentConstraint : public StateConstraint
        {
        private:
            scalar_t lambda_l_;
            scalar_t lambda_r_;
            scalar_t h_max_;
            NormalizedMomentConstraint(const NormalizedMomentConstraint &rhs) = default;

        public:
            NormalizedMomentConstraint(scalar_t lambda_l, scalar_t lambda_r, scalar_t h_max);

            NormalizedMomentConstraint *clone() const override;

            /** Get the size of the constraint vector at given time */
            size_t getNumConstraints(scalar_t time) const override;

            /** Get the constraint vector value */
            vector_t getValue(scalar_t time, const vector_t &state, const PreComputation &preComp) const override;

            /** Get the constraint linear approximation */
            VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t &state,
                                                                     const PreComputation &preComp) const override;
            ~NormalizedMomentConstraint();
        };

    } // namespace legged_robot

} // namespace ocs2
