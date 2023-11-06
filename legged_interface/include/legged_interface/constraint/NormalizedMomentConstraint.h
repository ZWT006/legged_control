
#pragma once
#include "ocs2_core/constraint/StateConstraint.h"

namespace ocs2
{
    namespace legged_robot
    {
        //NOTE: final 关键字表示该类不能被继承，是最终类，终极类，不可被继承
        class NormalizedMomentConstraint final: public StateConstraint
        {
        private:
            scalar_t lambda_l_;
            scalar_t lambda_r_;
            scalar_t h_max_;
            NormalizedMomentConstraint(const NormalizedMomentConstraint &rhs);

        public:
            NormalizedMomentConstraint(scalar_t lambda_l, scalar_t lambda_r, scalar_t h_max);
            //NOTE: override 用于显式地指示该函数是一个覆盖（重写）基类中的虚函数
            NormalizedMomentConstraint *clone() const override { return new NormalizedMomentConstraint(*this); }

            /** Get the size of the constraint vector at given time */
            size_t getNumConstraints(scalar_t time) const override { return 1; };

            /** Get the constraint vector value */
            vector_t getValue(scalar_t time, const vector_t &state, const PreComputation &preComp) const override;

            /** Get the constraint linear approximation */
            VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t &state,
                                                                     const PreComputation &preComp) const override;
            ~NormalizedMomentConstraint() override = default;
        };

    } // namespace legged_robot

} // namespace ocs2
