package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;


public class PIDBasic {
    private boolean hasRun = false;

    private ElapsedTime timer = new ElapsedTime();

    private double previousError = 0;

    private double integralSum = 0;

    private double derivative = 0;

    double Kp = 0;
    double Ki = 0;
    double Kd = 0;


    public double calculate(double reference, double state) {
        double ct = getDT();
        double error = calculateError(reference, state);
        double derivative = calculateDerivative(error,ct);
        integrate(error,ct);
        previousError = error;
        return error * Kp + integralSum * Ki + derivative * Kd;
    }

    public double getDT() {
        if (!hasRun) {
            hasRun = true;
            timer.reset();
        }
        double ct = timer.seconds();
        timer.reset();
        return ct;
    }

    protected double calculateError(double reference, double state) {
        return reference - state;
    }

    protected void integrate(double error, double ct) {
        integralSum += ((error + previousError) / 2) * ct;
    }

    protected double calculateDerivative(double error, double ct) {
        derivative = (error - previousError) / ct;
        return derivative;
    }
}
