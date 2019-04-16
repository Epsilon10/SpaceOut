#include "StateSpacePlant.hh"

template <uint32_t States, uint32_t Inputs, uint32_t Outputs>
StateSpacePlant<States, Inputs, Outputs>::StateSpacePlant(const StateSpacePlantCoeffs<States, Inputs, Outputs>& plantCoeffs) {
    add_coeffecients(plantCoeffs);
    reset();
}

template <uint32_t States, uint32_t Inputs, uint32_t Outputs>
const Eigen::Matrix<double, States, States>& 
StateSpacePlant<States, Inputs, Outputs>::A() const {
    return get_coeffecients().A;
}

template<uint32_t States, uint32_t Inputs, uint32_t Outputs>
double StateSpacePlant<States, Inputs, Outputs>::A(uint32_t row, uint32_t column) const {
    return A()(row, column);
} 

template <uint32_t States, uint32_t Inputs, uint32_t Outputs>
const Eigen::Matrix<double, States, Inputs>& 
StateSpacePlant<States, Inputs, Outputs>::B() const {
    return get_coeffecients().B;
}

template <uint32_t States, uint32_t Inputs, uint32_t Outputs>
double StateSpacePlant<States, Inputs, Outputs>::B(uint32_t row, uint32_t column) const {
    return get_coeffecients().B;
}

