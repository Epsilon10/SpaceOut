#pragma once
#include "SpaceOut.hh"


template <uint32_t States, uint32_t Inputs, uint32_t Outputs>
struct StateSpacePlantCoeffs {
    public:
        explicit StateSpaceCoeffs(
                const Eigen::Matrix<double, States, States>& A, 
                const Eigen::Matrix<double, States, Inputs>& B,
                const Eigen::Matrix<double, Outputs, States>& C,
                const Eigen::Matrix<double, Outputs, Inputs>& D
            ) : A(A), B(B), C(C), D(D) {}

            const Eigen::Matrix<double, States, States> A;
            const Eigen::Matrix<double, States, Inputs> B;
            const Eigen::Matrix<double, Outputs, States> C;
            const Eigen::Matrix<double, Outputs, Inputs> D;
};

template <uint32_t States, uint32_t Inputs, uint32_t Outputs>
class StateSpacePlant {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * Constructs a plant with the given coeffecients
         * 
         * @param plantCoeffs Plant coeffecients
         */ 

        StateSpacePlant(const StateSpacePlantCoeffs<States, Inputs, Outputs>& plantCoeffs);

        /**
         * Returns the system matrix A
         */

        const Eigen::Matrix<double, States, States>& A() const;

        /**
         * Returns an element of the system matrix A
         * @param row Row of A
         * @param column Column of A
         */

        double A(uint32_t row, uint32_t column) const;

        /**
         * Returns the input matrix B
         */ 
    
        const Eigen::Matrix<double, States, Inputs>& B() const;        -

        /**
         * Returns an element of the system matrix B
         * @param row Row of B
         * @param column Column of B
         */

        double B(uint32_t row, uint32_t column) const;

        /**
         * Returns the system matrix C
         */ 
        
        const Eigen::Matrix<double, Outputs, States>& C() const;

        /**
         * Returns an element of the system matrix C
         * @param row Row of C
         * @param column Column of C
         */    

        double C(uint32_t row, uint32_t column) const;

        // TODO: Add D getters

        /**
         * Returns the system matrix C
         */ 

        const Eigen::Matrix<double, States, 1>& X() const;

        /**
         * Returns an element of the current state X
         * 
         * @param row Row of X
         */ 

        double X(uint32_t row) const;

        /**
         * Returns the current measurement vector Y
         */ 

        const Eigen::Matrix<double, Outputs, 1>& Y() const;

        /**
         * Returns an element of the current measurement vector y.
         *
         * @param row Row of y.
         */

        double Y(uint32_t row) const;

        /**
         * Set the initial state x.
         *
         * @param x The initial state.
         */

        void set_X(const Eigen::Matrix<double, States, 1>& x);

        /**
         * Set the current measurement y.
         *
         * @param y The current measurement.
         */

        void set_Y(const Eigen::Matrix<double, Outputs, 1>& y);

        /**
         * Set an element of the current measurement y.
         *
         * @param row     Row of y.
         * @param value Value of element of y.
         */

        void set_Y(uint32_t row, double value);

        /**
         * Add a set of gains to the gain queue
         * 
         * @param plantCoeffs
         */

        void add_coeffecients(const StateSpacePlantCoeffs<States, Inputs, Outputs>& plantCoeffs);

        /**
         * Get coeffs at an index
         * 
         * @param index Gain queue index
         */

        const StateSpacePlantCoeffs<States, Inputs, Outputs>& get_coeffecients(uint32_t index) const;

        /**
         * Get current gains
         */
        void get_coeffecients() const; 

        /**
         * Returns the current gain index
         */
        void get_gain_index() const; 

        /** 
         * Set gain queue index
         * 
         * @param index The gain index
         */

        void set_gain_index(uint32_t new_index);

        /**
         * Resets the plant
         * 
         */
        void reset();

        /**
         * Computes the new x and y given a control input
         * 
         * @param input Input vector
         */

        void update(const  Eigen::Matrix<double, Inputs, 1>& u);

        /**
         * Computes the new x given the old x and the input vector
         *
         * This is used by state observers directly to run updates based on state
         * estimate.
         *
         * @param x current state
         * @param u control input
         */

        Eigen::Matrix<double, States, 1> update_X(const Eigen::Matrix<double, States, 1>& x, const Eigen::Matrix<double, Inputs, 1>& u) const;

        /**
         * Computes the new y given the control input.
         *
         * This should be used when setting x manually to update y separately.
         *
         * @param u The control input.
   */
        Eigen::Matrix<double, Outputs, 1> update_Y(const Eigen::Matrix<double, Inputs, 1>& u) const;


    private:
        
        Eigen::Matrix<double, States, 1> _X;
        Eigen::Matrix<double, Outputs, 1> _Y;
        uint32_t gain_index = 0;

        std::vector<StateSpacePlantCoeffs<States, Inputs, Outputs>> gains;

};