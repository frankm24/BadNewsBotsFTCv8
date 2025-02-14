package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import badnewsbots.hardware.GamepadEx;
import badnewsbots.robots.MecanumBotDriveOnly;

@TeleOp
public class MecanumDriveTeleOp extends LinearOpMode {
    private MecanumBotDriveOnly robot;

    // Settings
    private final float slowSpeed = 0.25f;
    private final float mediumSpeed = 0.5f;
    private final float fastSpeed = 1.0f;
    private float speedMultiplier = mediumSpeed; // scale movement speed
    private final boolean reverseDriveControlLogicalFront = true;
    private float reverseMultiplier;

    private GamepadEx smartGamepad;
    private FtcDashboard ftcDashboard;

    private SampleMecanumDrive drive;

    private Pose2d poseEstimate;

    public void mainLoop() {
        double prevTime = 0;
        double front_leftPower = 0;
        double back_leftPower = 0;
        double front_rightPower = 0;
        double back_rightPower = 0;

        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double deltaTime = currentTime - prevTime;

            if (smartGamepad.startPressed()) {
                changeSpeedScale();
            }

            float leftStickY = -smartGamepad.leftStickY() * speedMultiplier * reverseMultiplier;
            float leftStickX = smartGamepad.leftStickX() * speedMultiplier * reverseMultiplier;
            float rightStickX = smartGamepad.rightStickX() * speedMultiplier;

            double denominator = Math.max(Math.abs(leftStickY) + Math.abs(leftStickX) + Math.abs(rightStickX), 1);
            front_leftPower = (leftStickY + leftStickX + rightStickX) / denominator;
            back_leftPower = (leftStickY - leftStickX + rightStickX) / denominator;
            front_rightPower = (leftStickY - leftStickX - rightStickX) / denominator;
            back_rightPower = (leftStickY + leftStickX - rightStickX) / denominator;

            drive.setMotorPowers(front_leftPower, back_leftPower, back_rightPower, front_rightPower);

            poseEstimate = drive.getPoseEstimate();

            drive.updatePoseEstimate();
            telemetry.addData("Δt (ms)", deltaTime * 1E3); // Δt at top so driver can see that the loop is updating fine
            telemetry.addData("front_left power", front_leftPower);
            telemetry.addData("back_left power", back_leftPower);
            telemetry.addData("front_right power", front_rightPower);
            telemetry.addData("back_right power", back_rightPower); //String.format("%.2f",back_rightPower*100)
            telemetry.addData("x (in)", poseEstimate.getX());
            telemetry.addData("y (in)", poseEstimate.getY());
            telemetry.addData("θ (rad)", poseEstimate.getHeading());
            telemetry.update();
            prevTime = currentTime;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumBotDriveOnly(this);
        drive = robot.getDrive();
        smartGamepad = new GamepadEx(gamepad1);
        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, ftcDashboard.getTelemetry());

        if (reverseDriveControlLogicalFront) {
            reverseMultiplier = -1;
        } else {
            reverseMultiplier = 1;
        } // reverse drive if necessary;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(new Pose2d());

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status: ", "Initialized");
            telemetry.update();
            idle();
        }
        mainLoop();
    }
    private void changeSpeedScale() {
        if (speedMultiplier == fastSpeed) speedMultiplier = mediumSpeed;
        else if (speedMultiplier == mediumSpeed) speedMultiplier = fastSpeed;
        else speedMultiplier = mediumSpeed;
    }
}
