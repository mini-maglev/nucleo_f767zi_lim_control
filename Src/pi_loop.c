#include <math.h>

// PI Controller Gains
#define KPq 1.0   // Proportional gain for i_q
#define KIq 0.1   // Integral gain for i_q
#define KPd 1.0   // Proportional gain for i_d
#define KId 0.1   // Integral gain for i_d

#define SPEED_REF 1000.0 // Desired speed setpoint (in rpm)
#define id_ref 0 // Set id_ref to 0 (we only regulate torque, not flux)


// Integral error accumulators
static double integral_error_q = 0.0;
static double integral_error_d = 0.0;

// PI Controller function for speed regulation
void motor_PI_control(double i_a, double i_b, double i_c, double theta_rotor, double speed_actual, double *i_alpha, double *i_beta) {
    // Clarke Transform: Convert 3-phase currents to stationary 2D frame
    double i_alpha_temp = (2.0 / 3.0) * (i_a - 0.5 * i_b - 0.5 * i_c);
    double i_beta_temp = (2.0 / 3.0) * ((sqrt(3) / 2.0) * (i_b - i_c));

    // Park Transform: Convert stationary frame to DQ frame
    double i_d = i_alpha_temp * cos(theta_rotor) + i_beta_temp * sin(theta_rotor);
    double i_q = -i_alpha_temp * sin(theta_rotor) + i_beta_temp * cos(theta_rotor);

    // Speed PI Controller: Compute error and apply PI control
    double speed_error = SPEED_REF - speed_actual;
    integral_error_q += speed_error;

    double id_error = id_ref - i_d;
    integral_error_d +=  id_error;

    double i_q_new = KPq * speed_error + KIq * integral_error_q;
    double i_d_new = KPd * id_error + KId * integral_error_d;

    // Inverse Park Transform: Convert DQ currents back to stationary frame
    *i_alpha = i_d_new * cos(theta_rotor) - i_q_new * sin(theta_rotor);
    *i_beta = i_d_new * sin(theta_rotor) + i_q_new * cos(theta_rotor);
}
