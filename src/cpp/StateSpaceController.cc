#include "StateSpaceController.hh"

template <uint32_t States, uint32_t Inputs, uint32_t Outputs>
StateSpaceController<States, Inputs, Outputs>::StateSpaceController(
    const StateSpaceControllerCoeffs<States, Inputs, Outputs>& controllerCoeffs,
    StateSpacePlant<States, Inputs, Outputs>& plant)
    : plant(&plant) {
  AddCoefficients(controllerCoeffs);
}

template <uint32_t States, uint32_t Inputs, uint32_t Outputs>
void StateSpaceController<States, Inputs, Outputs>::enable() {
    _enabled = true;
}

template <uint32_t States, uint32_t Inputs, uint32_t Outputs>
void StateSpaceController<States, Inputs, Outputs>::enable() {
    _enabled = false;
}

template <uint32_t States, uint32_t Inputs, uint32_t Outputs>
const Eigen::Matrix<double, Inputs, States>& 
StateSpaceController<States, Inputs, Outputs>::Kff(uint32_t row, uint32_t column) const {
    
}


