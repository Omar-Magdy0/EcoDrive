#ifndef SIL_H
#define SIL_H

#include <math.h>
#include <stdio.h>



/** 
 * Equations aid @   https://colab.research.google.com/drive/1znA4MlEywY9Z5h4IoQfeT-dujAUs9yPJ?usp=sharing
*/
//#define EIGEN_SOLVER
#if defined(EIGEN_SOLVER)
#include <Eigen/Dense>
#endif

struct DummyLoad{
    double Tc;
    double B = 2.5e-4;
    double K = 1e-7;
    double J = 0.025;
};


struct Sil
{
    struct State{
        double time;
        double ip[3]; /** Phase currents ia, ib, ic */
        double vp[3]; /** Motor terminal voltages va, vb, vc */
        double vn;
        double vinv[3]; /** inverter equivelant average voltage source */
        double rinv[3]; /** inverter equivelant average impedance */
        double Lself[3]; /** Laa, Lbb, Lcc*/
        double Lmut[3]; /** Lab, Lbc, Lca*/
        double dPsim_dTheta[3]; /** Rate of change of field flux with respect to angle*/
        double dLself_dTheta[3]; /** Rate of change of self inductances with respect to angle*/
        double dLmut_dTheta[3]; /** Rate of change of mutual inductances with respect to angle*/
        double dPsim_dt[3];
        double theta; /** Rotor mechanical angle radians */
        double omega; /** Rotor mechanical angular speed radians per second */
        double alignment_torque; /** Motor alignment torque */
        double reluctance_torque; /** Motor reluctance torque */
        double torque; /** Motor total developed torque */
    } state;

    struct Param{
        double motor_Rs; /** stator resistance */
        double motor_Ls; /** stator inductance */
        double motor_Lm; /** magnetizing inductance */
        double motor_Ms; /** mutual inductance */
        double motor_fluxLinkage; /** flux linkage */
        double motor_pp; /** pole pairs */
        double motor_rotorOffset; /** rotor offset rads*/
        double motor_J; /** motor inertia */
        double motor_B; /** motor damping coofficient */
        double inv_Ron; /** inverter on resistance */
        double inv_Roff;
        double inv_alpha = 0; /** inverter impedance transition constant in terms of samples*/
        double load_J;
        double load_B;
        void (*dPsim_dTheta)(double theta_e, double dPsi_dTheta[3]);
    } param;

    struct Input {
        double dt;
        double duty[3]; /** inverter duty cycles for a,b,c */
        double drive[3]; /** inverter drive for a,b,c  1 for driven, 0 for float/Hi-Z */
        double vcc; /** inverter supply voltage */
        double load_torque;
    };

    class {
        FILE* file;
        public:
            bool start(const char* filename)
            {
                file = fopen(filename, "w");
                if(file){
                    fprintf(file, "Time,theta,omega,torque,alignment_torque,reluctance_torque,ip_a,ip_b,ip_c,vp_a,vp_b,vp_c,vn,vinv_a,vinv_b,vinv_c,rinv_a,rinv_b,rinv_c\n");
                    return true;
                }else{
                    return false;
                }
            }
            bool log(const State& s) {
                if(file) {
                    fprintf(file, "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
                                  "%.9f,%.9f,%.9f,"
                                  "%.9f,%.9f,%.9f,%.9f,"
                                  "%.9f,%.9f,%.9f,"
                                  "%.9f,%.9f,%.9f\n",
                        s.time, s.theta, s.omega, s.torque, s.alignment_torque, s.reluctance_torque,
                        s.ip[0], s.ip[1], s.ip[2],
                        s.vp[0], s.vp[1], s.vp[2], s.vn,
                        s.vinv[0], s.vinv[1], s.vinv[2],
                        s.rinv[0], s.rinv[1], s.rinv[2]);
                    return true;
                }
                return false;
            }
            bool stop(){
                if(file){
                    fclose(file);
                    file = nullptr;
                    return true;
                }else{
                    return false;
                }
            }
    } logger;

    void step(const Input& in)
    {
        state.time += in.dt;
        //Inverter equivelance 
        for(int i = 0; i < 3; i++) {
            state.vinv[i] = ( in.vcc * in.duty[i] ) * in.drive[i];
            double rt = in.drive[i] ? param.inv_Ron : param.inv_Roff; 
            state.rinv[i] = state.rinv[i]*param.inv_alpha + rt*(1 - param.inv_alpha);
        }

        //Motor equivelant circuit update
        constexpr double m_2pi3 = 2 * M_PI / 3;
        constexpr double m_pi6 = M_PI / 6;
        double theta_e = param.motor_pp * state.theta + param.motor_rotorOffset;
        double omega_e = param.motor_pp * state.omega;
        double phase_angles[3] = { theta_e, theta_e - m_2pi3, theta_e + m_2pi3 };

        param.dPsim_dTheta(theta_e, state.dPsim_dTheta);
        for(int i = 0; i < 3; i++) {
            state.Lself[i] = param.motor_Ls + param.motor_Lm*cos(2*phase_angles[i]);
            state.dLself_dTheta[i] = - 2 * param.motor_Lm*sin(2*phase_angles[i]);
            state.Lmut[i] = - param.motor_Ms - param.motor_Lm*cos(2*(phase_angles[i] + m_pi6));
            state.dLmut_dTheta[i] = 2 * param.motor_Lm*sin(2*(phase_angles[i] + m_pi6));
            state.dPsim_dt[i] = state.dPsim_dTheta[i] * omega_e;
        }

        // Circuit solver
        // Solving for the following Set of Equations for motor phase voltages vp_a, vp_b, vp_c
        // Vpn = R*I + dPsi/dt
        // dPsi/dt = dPsim/dTheta * dTheta/dt + dL/dt * I + L * dI/dt
        // L is inductance matrix with Lself on diagonal and Lmut on off diagonal
        // treating dL/dt as a resistance term
        // A = B*X + C*dX/dt
        // using backward euler for dX/dt = (X - Xprev)/dt
        // A - C/dt*Xprev = (B + C/dt)*X
        // E = G*X
        //

        #ifndef EIGEN_SOLVER
        //Circuit solver
        double Rself[3], Rmut[3];
        for(int i = 0; i < 3; i++) {
            Rself[i] = state.rinv[i] + param.motor_Rs + state.dLself_dTheta[i] * omega_e;
            Rmut[i] = state.dLmut_dTheta[i] * omega_e;
        }

        //Backward euler equivelant solving
        // Set of equations 
        // 
        double L_dt[2][2];
        L_dt[0][0] = (state.Lself[0] + state.Lself[2] - 2*state.Lmut[2]) / in.dt;
        L_dt[0][1] = (state.Lmut[0] - state.Lmut[1] - state.Lmut[2] + state.Lself[2]) / in.dt;
        L_dt[1][0] = (state.Lmut[0] - state.Lmut[1] - state.Lmut[2] + state.Lself[2]) / in.dt;
        L_dt[1][1] = (state.Lself[1] + state.Lself[2] - 2*state.Lmut[1]) / in.dt;

        double E[2];
        E[0] = (state.vinv[0] - state.dPsim_dt[0]) - (state.vinv[2] - state.dPsim_dt[2]) + L_dt[0][0] * state.ip[0] + L_dt[0][1] * state.ip[1];
        E[1] = (state.vinv[1] - state.dPsim_dt[1]) - (state.vinv[2] - state.dPsim_dt[2]) + L_dt[1][0] * state.ip[0] + L_dt[1][1] * state.ip[1];
        
        double R[2][2];
        R[0][0] = Rself[0] + Rself[2] - 2*Rmut[2] + L_dt[0][0];
        R[0][1] = Rmut[0] - Rmut[1] - Rmut[2] + Rself[2] + L_dt[0][1];
        R[1][0] = Rmut[0] - Rmut[1] - Rmut[2] + Rself[2] + L_dt[1][0];
        R[1][1] = Rself[1] - 2*Rmut[1] + Rself[2] + L_dt[1][1];
        double iprev[3] = {state.ip[0], state.ip[1], state.ip[2]};
        //Solve 2x2 system through inverting R
        double det = R[0][0]*R[1][1] - R[1][0]*R[0][1];
        //lets solve for ip[0] and ip[1]
        state.ip[0] = ( E[0]*R[1][1] - E[1]*R[0][1] ) / det;
        state.ip[1] = ( E[1]*R[0][0] - E[0]*R[1][0] ) / det;
        state.ip[2] = - state.ip[0] - state.ip[1];
        //Now 3 phase abc currents ready....  get Phase voltages on motor terminals
        for(int i = 0; i < 3; i++) state.vp[i] = state.vinv[i] - state.ip[i] * state.rinv[i];
        state.vn = ( (state.vinv[2] - state.dPsim_dt[2]) + (state.Lmut[2]/in.dt) * iprev[0] + (state.Lmut[1]/in.dt) * iprev[1] + (state.Lself[2]/in.dt) * iprev[2]); 
        state.vn -= ((Rmut[2]+state.Lmut[2]/in.dt) * state.ip[0] + (Rmut[1]+state.Lmut[1]/in.dt) * state.ip[1] + (Rself[2]+state.Lself[2]/in.dt) * state.ip[2]);

        // Electromechanical conversion
        state.alignment_torque = param.motor_pp * ( state.ip[0]*state.dPsim_dTheta[0] + state.ip[1]*state.dPsim_dTheta[1] + state.ip[2]*state.dPsim_dTheta[2]); 
        state.reluctance_torque =  state.ip[0] * (state.dLself_dTheta[0]*state.ip[0] + state.dLmut_dTheta[0] *state.ip[1] + state.dLmut_dTheta[2] *state.ip[2]);
        state.reluctance_torque += state.ip[1] * (state.dLmut_dTheta[0]* state.ip[0] + state.dLself_dTheta[1]*state.ip[1] + state.dLmut_dTheta[1] *state.ip[2]);
        state.reluctance_torque += state.ip[2] * (state.dLmut_dTheta[2]* state.ip[0] + state.dLmut_dTheta[1] *state.ip[1] + state.dLself_dTheta[2]*state.ip[2]);
        state.reluctance_torque = 0.5 * param.motor_pp * state.reluctance_torque;
        state.torque = state.alignment_torque + state.reluctance_torque; 
        
        //Eigen Based implementation
        #else
 
        Eigen::Matrix<double, 4, 4> B;
        for(int i = 0; i < 3; i++)
        {
            B(i,i) = state.rinv[i] + param.motor_Rs + state.dLself_dTheta[i] * omega_e;
            B(i,3) = 1;
            B(3,i) = 1;
        }
        B(0,1) = state.dLmut_dTheta[0] * omega_e; B(1,0) = state.dLmut_dTheta[0] * omega_e; 
        B(0,2) = state.dLmut_dTheta[2] * omega_e; B(2,0) = state.dLmut_dTheta[2] * omega_e;
        B(1,2) = state.dLmut_dTheta[1] * omega_e; B(2,1) = state.dLmut_dTheta[1] * omega_e;
        B(3,3) = 0;

        Eigen::Matrix<double, 4, 4> C_dt;
        for(int i = 0; i < 3; i++)
        {
            C_dt(i,i) = state.Lself[i];
            C_dt(i,3) = 0;
            C_dt(3,i) = 0;
        }
        C_dt(0,1) = state.Lmut[0]; C_dt(1,0) = state.Lmut[0]; 
        C_dt(0,2) = state.Lmut[2]; C_dt(2,0) = state.Lmut[2];
        C_dt(1,2) = state.Lmut[1]; C_dt(2,1) = state.Lmut[1];
        C_dt(3,3) = 0;
        C_dt = C_dt / in.dt;
        
        Eigen::Matrix<double, 4, 1> A;
        for(int i = 0; i < 3; i++) A(i,0) = state.vinv[i] - dPsim_dt[i];
        A(3,0) = 0;
        Eigen::Matrix<double, 4, 1> X;
        Eigen::Matrix<double, 4, 1> X_prev;
        X_prev << state.ip[0], state.ip[1], state.ip[2], state.vn;
        Eigen::Matrix<double, 4, 1> lhs = A - C_dt * X_prev;
        Eigen::Matrix<double, 4, 4> rhs = B + C_dt;
        // LU Decomposition
        X = rhs.inverse() * lhs;

        // Extract results
        state.ip[0] = X(0);
        state.ip[1] = X(1);
        state.ip[2] = X(2);
        state.vn = X(3);
        for(int i = 0; i < 3; i++) state.vp[i] = state.vinv[i] - state.ip[i] * state.rinv[i];

        state.alignment_torque = param.motor_pp * ( state.ip[0]*state.dPsim_dTheta[0] + state.ip[1]*state.dPsim_dTheta[1] + state.ip[2]*state.dPsim_dTheta[2]);
        state.reluctance_torque = param.motor_pp * 0.5 * (  state.ip[0] * (state.dLself_dTheta[0]*state.ip[0] + state.dLmut_dTheta[0] *state.ip[1] + state.dLmut_dTheta[2] *state.ip[2]) +
                                                            state.ip[1] * (state.dLmut_dTheta[0]* state.ip[0] + state.dLself_dTheta[1]*state.ip[1] + state.dLmut_dTheta[1] *state.ip[2]) +
                                                            state.ip[2] * (state.dLmut_dTheta[2]* state.ip[0] + state.dLmut_dTheta[1] *state.ip[1] + state.dLself_dTheta[2]*state.ip[2]) );
        state.torque = state.alignment_torque + state.reluctance_torque;
        #endif

        // Mechanical solve backward euler
        double J_dt = (param.motor_J + param.load_J)/in.dt;
        double Bt = param.motor_B + param.load_B;
        double T = state.torque - in.load_torque;

        state.omega = (T + J_dt*state.omega)/(J_dt + Bt);
        state.theta = state.omega*in.dt + state.theta;
    }

    void abc_to_dq(float a, float b, float c, float *d, float *q) 
    {
        // Clarke transform (abc -> αβ)
        float alpha = (2.0f/3.0f) * (a - 0.5f * b - 0.5f * c);
        float beta  = (2.0f/3.0f) * (0.86602540378f * b - 0.86602540378f * c);  // sqrt(3)/2
        
        // Park transform (αβ -> dq)
        float theta_e = param.motor_pp * state.theta + param.motor_rotorOffset;
        // Park transform (αβ -> dq)
        *d = alpha * cosf(theta_e) + beta * sinf(theta_e);
        *q = -alpha * sinf(theta_e) + beta * cosf(theta_e);
    }

    void abc_to_alpha_beta(float a, float b, float c, float *alpha, float *beta) 
    {
        // Clarke transform (abc -> αβ)
        *alpha = (2.0f/3.0f) * (a - 0.5f * b - 0.5f * c);
        *beta  = (2.0f/3.0f) * (0.86602540378f * b - 0.86602540378f * c);  // sqrt(3)/2
    }
};
        

#include <vector>
#include <array>
#include <hdf5.h>
#include <H5Cpp.h>
class SilLogger
{
    volatile bool log_on;
    unsigned int current_state_rows = 0;
    unsigned int current_param_rows = 0;
    enum class stateDataOrder
    {
        time,
        ip0,ip1,ip2,
        vp0,vp1,vp2,vn,
        vinv0,vinv1,vinv2,
        rinv0,rinv1,rinv2,
        Lself0,Lself1,Lself2,
        Lmut0,Lmut1,Lmut2,
        dPsim_dTheta0,dPsim_dTheta1,dPsim_dTheta2,
        theta, omega, alignment_torque, reluctance_torque,
        load_torque,
        COUNT
    };
    enum class paramDataOrder
    {
        time,
        vcc,
        Rs, Ls, Lm, Ms,
        FluxLinkage, PP,
        rotorOffset, Motor_J, Motor_B,
        inv_Ron, inv_Roff, inv_alpha,
        load_J, load_B,
        COUNT
    };

    H5::H5File* file = nullptr;
    H5::DataSet* state_dataset = nullptr;
    H5::DataSet* param_dataset = nullptr;
    std::vector<std::array<float, (int)stateDataOrder::COUNT>> state_buffer;
    std::vector<std::array<float, (int)paramDataOrder::COUNT>> param_buffer;

public:
    unsigned int state_reserve_cap = 10000;
    unsigned int param_reserve_cap = 100;
    void logStart(const char* filename)
    {
        state_buffer.clear();
        state_buffer.reserve(state_reserve_cap);
        param_buffer.clear();
        param_buffer.reserve(param_reserve_cap);
        current_state_rows = 0;
        current_param_rows = 0;
        //Other stuff, for HDF5 file initialization , metadata and constants write
        file = new H5::H5File(filename, H5F_ACC_TRUNC);

        //Dataspace for states 
        {
            hsize_t dims[2] = {0, (int)stateDataOrder::COUNT};           // Current size: 0 rows, 30 cols
            hsize_t max_dims[2] = {H5S_UNLIMITED, (int)stateDataOrder::COUNT};  // Rows can grow, cols fixed
            H5::DataSpace dataspace(2, dims, max_dims);

            //Set chunking properties
            H5::DSetCreatPropList prop;
            hsize_t chunk_dims[2] = {10000, (int)stateDataOrder::COUNT};
            prop.setChunk(2, chunk_dims);
            prop.setDeflate(3);  // Compression level 1-9

            state_dataset = new H5::DataSet(
                    file->createDataSet("sil_state", H5::PredType::NATIVE_FLOAT, dataspace, prop)
            );

            // Add column names as metadata
            std::vector<std::string> column_names = {
                "time",
                "ip0", "ip1", "ip2",
                "vp0", "vp1", "vp2", "vn",
                "vinv0", "vinv1", "vinv2",
                "rinv0", "rinv1", "rinv2",
                "Lself0", "Lself1", "Lself2",
                "Lmut0", "Lmut1", "Lmut2",
                "dPsim_dTheta0", "dPsim_dTheta1", "dPsim_dTheta2",
                "theta", "omega", "alignment_torque", "reluctance_torque",
                "load_torque"
            };

            // Flatten to comma-separated string (HDF5 attribute can't store vector directly)
            std::string names_str;
            for(size_t i = 0; i < column_names.size(); i++) {
                if(i > 0) names_str += ",";
                names_str += column_names[i];
            }

                // Write as attribute
            H5::DataSpace attr_space(H5S_SCALAR);
            H5::StrType str_type(H5::PredType::C_S1, names_str.size() + 1);
            H5::Attribute attr = state_dataset->createAttribute("column_names", str_type, attr_space);
            attr.write(str_type, names_str.c_str());
        }

        //Dataset for parameters
        {
            hsize_t dims[2] = {0, (int)paramDataOrder::COUNT};           // Current size: 0 rows, 30 cols
            hsize_t max_dims[2] = {H5S_UNLIMITED, (int)paramDataOrder::COUNT};  // Rows can grow, cols fixed
            H5::DataSpace dataspace(2, dims, max_dims);

            //Set chunking properties
            H5::DSetCreatPropList prop;
            hsize_t chunk_dims[2] = {100, (int)paramDataOrder::COUNT};
            prop.setChunk(2, chunk_dims);
            prop.setDeflate(8);  // Compression level 1-9

            param_dataset = new H5::DataSet(
                    file->createDataSet("sil_param", H5::PredType::NATIVE_FLOAT, dataspace, prop)
            );

            // Add column names as metadata
            std::vector<std::string> column_names = {
                "time",
                "vcc",
                "Rs", "Ls", "Lm", "Ms",
                "FluxLinkage", "PP",
                "rotorOffset", "Motor_J", "Motor_B",
                "inv_Ron", "inv_Roff", "inv_alpha",
                "load_J", "load_B",
            };

            // Flatten to comma-separated string (HDF5 attribute can't store vector directly)
            std::string names_str;
            for(size_t i = 0; i < column_names.size(); i++) {
                if(i > 0) names_str += ",";
                names_str += column_names[i];
            }

                // Write as attribute
            H5::DataSpace attr_space(H5S_SCALAR);
            H5::StrType str_type(H5::PredType::C_S1, names_str.size() + 1);
            H5::Attribute attr = param_dataset->createAttribute("column_names", str_type, attr_space);
            attr.write(str_type, names_str.c_str());
        }
        log_on = true;
    }
    void flushStateBuffer()
    {
        // Step 1: Check if anything to flush
        if(state_buffer.empty()) return;

        hsize_t num_new_rows = state_buffer.size();           // e.g., 10000
        hsize_t num_cols = (int)stateDataOrder::COUNT;       // e.g., 30
        hsize_t new_total_rows = current_state_rows + num_new_rows;

        hsize_t new_dims[2] = {new_total_rows, num_cols};
        state_dataset->extend(new_dims);  // Dataset now has room for new rows
        
        H5::DataSpace file_space = state_dataset->getSpace();
        hsize_t start[2] = {current_state_rows, 0};  // Start at first new row
        hsize_t count[2] = {num_new_rows, num_cols};  // Write all new rows
        file_space.selectHyperslab(H5S_SELECT_SET, count, start);
        
        H5::DataSpace mem_space(2, count);  // Your buffer is also 2D: rows × cols
        state_dataset->write(&state_buffer[0][0], H5::PredType::NATIVE_FLOAT, 
                             mem_space, file_space);
        
 
        current_state_rows = new_total_rows;
        state_buffer.clear();
    }
    void flushParamBuffer()
    {
        if(param_buffer.empty()) return;

        // Calculate dimensions
        hsize_t num_new_rows = param_buffer.size();
        hsize_t num_cols = (int)paramDataOrder::COUNT;
        hsize_t new_total_rows = current_param_rows + num_new_rows;

        // Extend dataset to make room
        hsize_t new_dims[2] = {new_total_rows, num_cols};
        param_dataset->extend(new_dims);

        // Select WHERE to write (the new empty space)
        H5::DataSpace file_space = param_dataset->getSpace();
        hsize_t start[2] = {current_param_rows, 0};
        hsize_t count[2] = {num_new_rows, num_cols};
        file_space.selectHyperslab(H5S_SELECT_SET, count, start);

        H5::DataSpace mem_space(2, count);
        param_dataset->write(&param_buffer[0][0], H5::PredType::NATIVE_FLOAT, mem_space,
                             file_space);

        // Update tracking and clear buffer
        current_param_rows = new_total_rows;
        param_buffer.clear();
    }
    void log(const Sil &sil, const Sil::Input &sil_input, bool log_param = false)
    {
        if(!log_on)return;
        std::array<float, (int)stateDataOrder::COUNT> state_sample;
        //Writing here 
        state_sample[(int)stateDataOrder::time] = sil.state.time; 
        state_sample[(int)stateDataOrder::vn] = sil.state.vn;
        state_sample[(int)stateDataOrder::theta] = sil.state.theta;
        state_sample[(int)stateDataOrder::omega] = sil.state.omega;
        state_sample[(int)stateDataOrder::alignment_torque] = sil.state.alignment_torque;
        state_sample[(int)stateDataOrder::reluctance_torque] = sil.state.reluctance_torque;
        state_sample[(int)stateDataOrder::load_torque] = sil_input.load_torque;
        for(int i = 0; i < 3; i++) 
        {
            state_sample[(int)stateDataOrder::vp0 + i] = sil.state.vp[i]; 
            state_sample[(int)stateDataOrder::ip0 + i] = sil.state.ip[i];
            state_sample[(int)stateDataOrder::vinv0 + i] = sil.state.vinv[i];
            state_sample[(int)stateDataOrder::rinv0 + i] = sil.state.rinv[i];
            state_sample[(int)stateDataOrder::Lself0 + i] = sil.state.Lself[i];
            state_sample[(int)stateDataOrder::Lmut0 + i] = sil.state.Lmut[i];
            state_sample[(int)stateDataOrder::dPsim_dTheta0 + i] = sil.state.dPsim_dTheta[i];
        }
        state_buffer.push_back(state_sample);
        if(state_buffer.size() >= state_reserve_cap)
        {
            flushStateBuffer();
        }

        if(log_param)
        {
            std::array<float, (int)paramDataOrder::COUNT> param_sample;
            param_sample[(int)paramDataOrder::time] = sil.state.time;
            param_sample[(int)paramDataOrder::vcc] = sil_input.vcc;
            param_sample[(int)paramDataOrder::Rs] = sil.param.motor_Rs;
            param_sample[(int)paramDataOrder::Ls] = sil.param.motor_Ls;
            param_sample[(int)paramDataOrder::Lm] = sil.param.motor_Lm;
            param_sample[(int)paramDataOrder::Ms] = sil.param.motor_Ms;
            param_sample[(int)paramDataOrder::FluxLinkage] = sil.param.motor_fluxLinkage;
            param_sample[(int)paramDataOrder::PP] = sil.param.motor_pp;
            param_sample[(int)paramDataOrder::rotorOffset] = sil.param.motor_rotorOffset;
            param_sample[(int)paramDataOrder::Motor_J] = sil.param.motor_J;
            param_sample[(int)paramDataOrder::Motor_B] = sil.param.motor_B;
            param_sample[(int)paramDataOrder::inv_Ron] = sil.param.inv_Ron;
            param_sample[(int)paramDataOrder::inv_Roff] = sil.param.inv_Roff;
            param_sample[(int)paramDataOrder::inv_alpha] = sil.param.inv_alpha;
            param_sample[(int)paramDataOrder::load_J] = sil.param.load_J;
            param_sample[(int)paramDataOrder::load_B] = sil.param.load_B;
            param_buffer.push_back(param_sample);
            if(param_buffer.size() >= param_reserve_cap)
            {
                flushParamBuffer();
            }
        }
    }
    void logStop()
    {
        log_on = false;
        // Other stuff for HDF5 file clearing
        // Flush Buffers and Close HDF5 handles
        flushStateBuffer();
        flushParamBuffer();
        delete state_dataset;
        delete param_dataset;
        delete file;
        state_dataset = nullptr;
        param_dataset = nullptr;
        file = nullptr;
    }
    bool isLogOn()
    {
        return log_on;
    }
};

inline Sil sil;
inline Sil::Input sil_input;
inline SilLogger silLogger;
inline DummyLoad dummy_load;

#endif // SIL_H
