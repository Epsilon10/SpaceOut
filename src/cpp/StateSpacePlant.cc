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
    return B()(row, column);
}

template <uint32_t States, uint32_t Inputs, uint32_t Outputs>
void StateSpacePlant<States, Inputs, Outputs>::update(const Eigen::Matrix<double, Inputs, 1>& u) {
    //check u
    _X = update_X(X(), u);
    _Y = update_Y(u);
}

template <uint32_t States, uint32_t Inputs, uint32_t Outputs>
Eigen::Matrix<double, States, 1> StateSpacePlant<States, Inputs, Outputs>::update_X(const Eigen::Matrix<double, States, 1> x,
    const Eigen::Matrix<double, Inputs, 1>& u) {
        return A() * x + B() * u;
}

template <uint32_t States, uint32_t Inputs, uint32_t Outputs>
Eigen::Matrix<double, Outputs, 1> StateSpacePlant<States, Inputs, Outputs>::update_Y(const Eigen::Matrix<double, Inputs, 1>& u) const {
    return C() * x + D() * u;
}

template <uint32_t States, uint32_t Inputs, uint32_t Outputs>
void StateSpacePlant<States, Inputs, Outputs>::add_coeffecients(const StateSpacePlantCoeffs<States, Inputs, Outputs>& plantCoeffs) {
    gains.push_back(plantCoeffs);
}

