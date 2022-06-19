package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MotorControl {

    public static double commandMotor(MotionProfile profile, DcMotorEx motor, ElapsedTime time){
        time.reset();
        MotionState state = profile.get(time.seconds());
        double feedback = calculateFullFeedback(state.getX(), state.getV(), motor.getCurrentPosition(), motor.getVelocity());
        double feedforward = calculateFeedForward(state.getV(), state.getA(), state.getJ());
        return feedback + feedforward;
    }

    public static double PIDVAJControl(MotionProfile profile, DcMotorEx motor, ElapsedTime time){
        time.reset();
        MotionState state = profile.get(time.seconds());
        double feedback = PIDControl(state.getX(), motor.getCurrentPosition(), time.seconds());
        double feedforward = calculateFeedForward(state.getV(), state.getA(), state.getJ());
        return feedback + feedforward;
    }

    public static double calculateFullFeedback(double posReference, double velocityReference, double position, double velocity){
        double velocityError = calculateFeedback(velocityReference, velocity, Constants.K2);
        double posError = calculateFeedback(posReference, position, Constants.K1);
        return velocityError + posError;
    }

    public static double calculateFeedback(double reference , double state, double gain) {
        double error = reference - state;
        return gain * error;
    }

    public static double calculateFeedForward(double targetVelocity, double targetAcceleration, double targetJerk){
        return (targetVelocity * Constants.Kv) + (targetAcceleration * Constants.Ka) + (targetJerk * Constants.Kj);
    }

    public static double PIDControl(double reference, double position, double time) {
        double integralSum = 0;
        double lastError = 0;

        double error = reference - position;

        // rate of change of the error
        double derivative = (error - lastError) / time;

        // sum of all error over time
        integralSum = integralSum + (error * time);
        lastError = error;

        return (Constants.Kp * error) + (Constants.Ki * integralSum) + (Constants.Kd * derivative);

    }

    public double angleWrap(double degrees) {
        double radians = Math.toRadians(degrees);
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return Math.toDegrees(radians);
    }
}
