package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.MotorControl;

@TeleOp(name = "TeleOP")
public class TeleOP extends OpMode {


    ElapsedTime motorTime = new ElapsedTime();

    private DcMotorEx motor = null;

    MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(0, 0, 0),
            new MotionState(60, 0, 0),
            25,
            40,
            100
    );

    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "leftMotor");

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad1.a) motor.setPower(MotorControl.commandMotor(profile, motor, motorTime));
    }
}
