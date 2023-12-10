#include <stdio.h>
#include <math.h>

// Constants
#define max_size 10

// Function to compute the sign of a number
int sign(double x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

// Motor torque function
double motor_torque(double omega) {
    double tm = 190.0;
    double omega_m = 4200.0;
    double beta = 0.4;
    return fmax(0, tm * (1 - beta * pow((omega / omega_m - 1), 2)));
}


// Vehicle update function
double vehicle_update(double t, double x, double u[], double params[]) {
    // Extract parameters
    double m = params[0];
    double g = params[1];
    double cr = params[2];
    double cd = params[3];
    double rho = params[4];
    double a = params[5];
    double alpha[] = {40, 25, 16, 12, 10};  // gear ratio / wheel radius

    // Extract inputs
    double v = x;
    double throttle = fmax(0, fmin(1, u[0]));  // clip throttle to [0, 1]
    int gear = (int)u[1];
    double theta = u[2];

    // Compute engine-related parameters
 
    double omega = alpha[gear - 1] * v;
    double f = alpha[gear - 1] * motor_torque(omega) * throttle;

    // Compute disturbance forces
    double fg = m * g * sin(theta);
    double fr = m * g * cr * sign(v);
    double fa = 0.5 * rho * cd * a * fabs(v) * v;

    // Final acceleration on the car
    double fd = fg + fr + fa;
    double dv = (f - fd) / m;

    return dv;
}



int main() {
    // Simulation parameters
    double t[max_size];
    double vref[max_size];
    double gear[max_size];
    double theta[max_size];
    double speed_readings[max_size];

    // Initialize parameters
    double params[] = {1600.0, 9.8, 0.01, 0.32, 1.3, 2.4};  // m, g, cr, cd, rho, a
    double u[] = {1, 1, 1};  // gear, throttle, road slope
    // Initialize speed readings with constant values
    for (int i = 0; i < max_size; i++) {
        t[i] = i;
        vref[i] = 20.0;
        gear[i] = 4.0;
        theta[i] = 0.0;
    }

    // Initial conditions
    double current_speed = 0.0;

    
    FILE *outputFile = fopen("output.txt", "w");

    
    if (outputFile == NULL) {
        fprintf(stderr, "Error opening the file.\n");
        return 1;
    }
   
    for (int i = 0; i < max_size; i++) {
    
        double control_signal = 0.1 * (vref[i] - current_speed);
        current_speed += vehicle_update(t[i], current_speed, u, params);

        
        speed_readings[i] = current_speed;

        
        fprintf(outputFile, "%.2f\n", current_speed);
    }

    fclose(outputFile);

    return 0;
}

