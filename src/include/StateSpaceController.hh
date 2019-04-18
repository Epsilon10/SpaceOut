#pragma once

#include "SpaceOut.hh"
#include "StateSpacePlant.hh"

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
         */]
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

        /**
         * Returns the feedforward matrix Kff
         */
        const Eigen::Matrix<double, Inputs, States>& Kff() const;

        /**
         * Returns an element from the feedforward matrix Kff 
         * 
         * @param row Row of Kff
         * @param column Column of Kff
         */ 

        const Eigen::Matrix<double, Inputs, States>& Kff(uint32_t row, uint32_t column) const;

        /**
         * Returns reference vector r
         */ 
        const Eigen::Matrix<double, States, 1>& R() const;

        /**
         * Returns an element of the reference vector r.
         *
         * @param row Row of r.
         */
        double R(uint32_t row) const;

        /**
         * Resets the controller
         */
        void reset();

        /**
         * Update the controller without setting a new reference
         * 
         * @param x the current state x
         */
        void update(const Eigen::Matrix<double, States, 1>& x);

        /**
         * Update the controller and set a new reference
         * 
         * @param x the current state x
         * @param nextR the next reference vector
         */
        void update(const Eigen::Matrix<double, States, 1>& x, const Eigen::Matrix<double, States, 1>& nextR);



    private:
        StateSpacePlant<States, Inputs, Outputs>* plant;

        bool _enabled = false;

        // Current reference
        Eigen::Matrix<double, States, 1> _r;

        // Capped computed controller output
        Eigen::Matrix<double, Inputs, 1> _u;

        std::vector<StateSpaceControllerCoeffs<States, Inputs, Outputs>> _ceoffecients;
        uint32_t index = 0;
        
        void cap_u();

};