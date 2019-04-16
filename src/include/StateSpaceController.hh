#pragma once

#include "SpaceOut.hh"

template <uint32_t States, uint32_t Inputs, uint32_t Ouputs>
struct StateSpaceControllerCoeffs { 
    StateSpaceControllerCoeffs(
        const Eigen::Matrix<double, Inputs, States>& K,
        const Eigen::Matrix<double, Inputs, States>& Kff,
        const Eigen::Matrix<double, Inputs, 1>& Umin,
        const Eigen::Matrix<double, Inputs, 1>& Umax
    ) : K(K), Kff(Kff), Umin(Umin), Umax(Umax) {}

    const Eigen::Matrix<double, Inputs, States>& K;
    const Eigen::Matrix<double, Inputs, States>& Kff;
    const Eigen::Matrix<double, Inputs, 1>& Umin;
    const Eigen::Matrix<double, Inputs, 1>& Umax;
};

template <uint32_t States, uint32_t Inputs, uint32_t Ouputs>
class StateSpaceController {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * Creates a State Space Controller based on the given plant and coeffs
         * 
         * @param controllerCoeffs Controller Coeffecients
         * @param plant
         */

        StateSpaceController(const StateSpaceControllerCoeffs<States, Inputs, Outputs>& controllerCoeffs, 
            StateSpacePlant<States, Inputs, Outputs>& plant);
        
        /**
         * Enables controller.
         */

        void enable();

        /**
         * Disables controller, resets the output to U
         */ 

        void disable();

        /**
         * Returns an element of the controller (gain) matrix K
         *
         * @param row Row of K.
         * @param column Column of K.
         */
        
        double K(uint32_t row, uint32_t column) const;

};