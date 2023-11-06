#include "legged_interface/constraint/NormalizedMomentConstraint.h"
#include "iostream"

namespace ocs2
{
    namespace legged_robot
    {
        NormalizedMomentConstraint::NormalizedMomentConstraint(scalar_t lambda_l, scalar_t lambda_r, scalar_t h_max)
            : StateConstraint(ConstraintOrder::Linear), lambda_l_(lambda_l), lambda_r_(lambda_r), h_max_(h_max)
        {
        }
        NormalizedMomentConstraint::NormalizedMomentConstraint(const NormalizedMomentConstraint &rhs)
            : StateConstraint(rhs) , lambda_l_(rhs.lambda_l_), lambda_r_(rhs.lambda_r_), h_max_(rhs.h_max_)
        {
        }

        /*************************************************************************************************************/
        // NormalizedMomentConstraint *clone() const override { return new NormalizedMomentConstraint(*this); }

        /** Get the size of the constraint vector at given time */
        // size_t getNumConstraints(scalar_t time) const override { return 1; };

        // NormalizedMomentConstraint::~NormalizedMomentConstraint() = default;
        /*************************************************************************************************************/

        /** Get the constraint vector value */
        vector_t NormalizedMomentConstraint::getValue(scalar_t time, const vector_t &state, const PreComputation &preComp) const
        {
            vector_t value(1);
            value << -1 * lambda_l_ * (state(0) * state(0) + state(1) * state(1)) - lambda_r_ * (state(3) * state(3)) + h_max_;
            // std::cout << "value: " << value.transpose() << std::endl;
            return value;
        }

        /** Get the constraint linear approximation */
        VectorFunctionLinearApproximation NormalizedMomentConstraint::getLinearApproximation(scalar_t time, const vector_t &state,
                                                                                             const PreComputation &preComp) const
        {
            VectorFunctionLinearApproximation linearApproximation;
            linearApproximation.f = getValue(time, state, preComp);
            //NOTE: 这里的 dfdx 是一个 1*n 的矩阵 注意对向量的求导应该是矩阵，即使是1*n维的
            linearApproximation.dfdx = matrix_t::Zero(1, state.size());
            linearApproximation.dfdx(0) = -2 * lambda_l_ * state(0);
            linearApproximation.dfdx(1) = -2 * lambda_l_ * state(1);
            linearApproximation.dfdx(3) = -2 * lambda_r_ * state(3);
            // std::cout << "linearApproximation.f: " << linearApproximation.f.transpose() << std::endl;
            // std::cout << "linearApproximation.dfdx: " << linearApproximation.dfdx << std::endl;
            return linearApproximation;
        }

    } // namespace legged_robot

} // namespace ocs2
