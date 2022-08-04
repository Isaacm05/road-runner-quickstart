package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.vision.CenteringPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Camera Center", group = "Test")
public class CameraTest extends LinearOpMode {

    private final CenteringPipeline pipeline = new CenteringPipeline(telemetry);

    private final PIDBasic pid = new PIDBasic(1,0,0);

    OpenCvWebcam webcam;

    double power = 0;

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized"); //Indicate that the program has initialized (convention more than anything)
        telemetry.update(); //Update the telemetry (clear the screen and write new data)

        SampleMecanumDriveCancelable chassis = new SampleMecanumDriveCancelable(hardwareMap);

        chassis.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(-90)));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.setPipeline(pipeline); //Sets the camera's pipeline once it is opened (which may take a few seconds)
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); //Sets the dimensions and orientation of the camera's stream (which streams to the Driver Station's viewport)
                telemetry.addData("OpenCV", "OpenCV Connected"); //Tells the driver that OpenCV has connected (been opened successfully)
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("OpenCV", "OpenCV failed to load Error Code: " + errorCode); //Tells the driver that OpenCV has failed to load, as well as why
                telemetry.update();
            }
        });

        while(!opModeIsActive()) {
            telemetry.addData("Something","Display Something");
            telemetry.update();
        }

        if(!opModeIsActive() || isStopRequested()) return;

        if (isStopRequested()) {
            webcam.stopStreaming();
            telemetry.addData("status", "stopped");
            telemetry.update();
            return;
        }

        waitForStart(); /* Wait for the driver to press start (necessary for the program not to crash, even if it is technically redundant) */

        //THE FOLLOWING CODE RUNS AFTER THE DRIVER PRESSES START

        while (opModeIsActive() && !isStopRequested()) {
            power = pid.calculate(160, pipeline.getPosition())/2;
            chassis.setMotorPowers(power, power, -power, -power);

            chassis.update();

            Pose2d poseEstimate = chassis.getPoseEstimate();

            telemetry.addData("Power", power*2);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }


    }
}
