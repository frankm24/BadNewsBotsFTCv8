package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.PowerPlayCompBotMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import badnewsbots.InterOpStorage;
import badnewsbots.Junction;
import badnewsbots.OpenCvRecorder;
import badnewsbots.hardware.GamepadEx;
import badnewsbots.hardware.RotatingClaw;
import badnewsbots.robots.PowerPlayCompBot;

// This class handles driver-controlled (Tele-Op) operation of the robot used in Power Play games.
@TeleOp
public final class PowerPlayTeleOp extends LinearOpMode {
    // Robot object
    private PowerPlayCompBot robot;

    private enum DriveMode {
        ROBOT_CENTRIC,
        AIM_ASSIST
    }

    private DriveMode currentDriveMode = DriveMode.ROBOT_CENTRIC;

    // Settings
    private final float slowSpeed = 0.25f;
    private final float mediumSpeed = 0.5f;
    private final float fastSpeed = 1.0f;
    private float speedMultiplier = mediumSpeed; // scale movement speed
    private final boolean reverseDriveControlLogicalFront = true;
    private float reverseMultiplier;

    private GamepadEx smartGamepad;
    private FtcDashboard ftcDashboard;
    private OpenCvRecorder recorder;

    private OpenCvWebcam camera;
    private PowerPlayCompBotMecanumDrive drive;
    private RotatingClaw claw;
    private RevBlinkinLedDriver ledDriver;

    private PIDFController headingController = new PIDFController(PowerPlayCompBotMecanumDrive.HEADING_PID);
    private List<Junction> junctions = InterOpStorage.junctions;
    private Vector2d targetPosition = junctions.get(0).getPositionVec2d();
    private Pose2d poseEstimate;
    private Canvas fieldOverlay;

    public void mainLoop() {
        double prevTime = 0;
        double front_leftPower = 0;
        double back_leftPower = 0;
        double front_rightPower = 0;
        double back_rightPower = 0;

        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double deltaTime = currentTime - prevTime;

            poseEstimate = drive.getPoseEstimate();
            // Declare telemetry packet for dashboard field drawing
            TelemetryPacket packet = new TelemetryPacket();
            fieldOverlay = packet.fieldOverlay();
            Pose2d driveDirection = new Pose2d();
            smartGamepad.update();
            claw.update();

            //if (smartGamepad.back_pressed) changeDriveMode();
            if (smartGamepad.startPressed()) changeSpeedScale();

            if (smartGamepad.dpadUp()) claw.moveSlidesToPresetHeight(RotatingClaw.SlideHeight.HIGH_GOAL);
            if (smartGamepad.dpadLeft()) claw.moveSlidesToPresetHeight(RotatingClaw.SlideHeight.MID_GOAL);
            if (smartGamepad.dpadDown()) claw.moveSlidesToPresetHeight(RotatingClaw.SlideHeight.LOW_GOAL);

            //if (smartGamepad.right_trigger_bool) claw.incrementSlidePositions();
            //if (smartGamepad.left_trigger_bool) claw.decrementSlidePositions();
            if (smartGamepad.yPressed()) claw.rotateToOtherSide();
            if (smartGamepad.aPressed()) claw.toggleGrip();
            if (smartGamepad.bPressed()) claw.moveSlidesToPresetHeight(RotatingClaw.SlideHeight.BOTTOM);
            if (smartGamepad.leftBumperPressed()) claw.letGo();
            if (smartGamepad.rightBumperPressed()) claw.gripRaiseAndRotate();

            if (currentDriveMode == DriveMode.ROBOT_CENTRIC) {
                // Only change target if not in target mode, once using aim assist keep target position fixed.
                Collections.sort(junctions, new JunctionComparator());
                targetPosition = junctions.get(0).getPositionVec2d();

                float leftStickY = -smartGamepad.leftStickY() * speedMultiplier * reverseMultiplier;
                float leftStickX = smartGamepad.leftStickX() * speedMultiplier * reverseMultiplier;
                //float rightStickY = smartGamepad.right_stick_y * speedMultiplier;
                float rightStickX = smartGamepad.rightStickX() * speedMultiplier;

                double denominator = Math.max(Math.abs(leftStickY) + Math.abs(leftStickX) + Math.abs(rightStickX), 1);
                front_leftPower = (leftStickY + leftStickX + rightStickX) / denominator;
                back_leftPower = (leftStickY - leftStickX + rightStickX) / denominator;
                front_rightPower = (leftStickY - leftStickX - rightStickX) / denominator;
                back_rightPower = (leftStickY + leftStickX - rightStickX) / denominator;

                drive.setMotorPowers(front_leftPower, back_leftPower, back_rightPower, front_rightPower);
            } else {
                // Create a vector from the gamepad x/y inputs which is the field relative movement
                // Then, rotate that vector by the inverse of that heading for field centric control
                Vector2d fieldFrameInput = new Vector2d(-smartGamepad.leftStickY(), -smartGamepad.leftStickX());
                Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                // Difference between the target vector and the bot's position
                Vector2d difference = targetPosition.minus(poseEstimate.vec());
                // Obtain the target angle for feedback and derivative for feedforward
                double theta = difference.angle();
                // Not technically omega because its power. This is the derivative of atan2
                double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                // Set the target heading for the heading controller to our desired angle
                headingController.setTargetPosition(theta);

                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                double headingInput = (headingController.update(poseEstimate.getHeading() + Math.toRadians(180))
                        * DriveConstants.kV + thetaFF)
                        * DriveConstants.TRACK_WIDTH;
                // Combine the field centric x/y velocity with our derived angular velocity
                driveDirection = new Pose2d(
                        robotFrameInput,
                        headingInput
                );
                driveDirection = driveDirection.plus(new Pose2d(0, 0, Math.toRadians(180)));
                drive.setWeightedDrivePower(driveDirection);
                // Update the heading controller with our current heading
                headingController.update(poseEstimate.getHeading() + Math.toRadians(180));
            }
            drive.updatePoseEstimate();
            drawDashboard();
            ftcDashboard.sendTelemetryPacket(packet);
            telemetry.addData("Δt (ms)", deltaTime * 1E3); // Δt at top so driver can see that the loop is updating fine
            telemetry.addData("Wrist side", claw.getCurrentSide());
            telemetry.addData("Grabber state", claw.getCurrentGripperState());
            telemetry.addData("Slide preset height", claw.getTargetSlideHeight());
            telemetry.addData("Slide position", claw.getSlide2PosTicks());
            telemetry.addData("Slide target position", claw.getSlide2TargetPosTicks());
            telemetry.addData("Drive mode", currentDriveMode);
            telemetry.addData("front_left power", front_leftPower);
            telemetry.addData("back_left power", back_leftPower);
            telemetry.addData("front_right power", front_rightPower);
            telemetry.addData("back_right power", back_rightPower);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("θ", poseEstimate.getHeading());
            telemetry.update();
            prevTime = currentTime;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new PowerPlayCompBot(this);
        drive = robot.getDrive();
        //camera = robot.getCamera();
        claw = robot.getRotatingClaw();
        smartGamepad = new GamepadEx(gamepad1);
        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, ftcDashboard.getTelemetry());
        // ledDriver = robot.getLedDriver();
        //setLedAllianceColor();

        if (reverseDriveControlLogicalFront) {
            reverseMultiplier = -1;
        } else {
            reverseMultiplier = 1;
        } // reverse drive if necessary;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);
        drive.setPoseEstimate(InterOpStorage.currentPose); // remember pose from autonomous
        /*
        // OpenCV begins here
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        recorder = new OpenCvRecorder(camera, telemetry, false);
        recorder.openCameraAsync();
        */

        claw.rotateToSide(claw.getCurrentSide());
        claw.release();

        // init loop
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status: ", "Initialized");
            //if (recorder.isRecording()) {
            //    telemetry.addData("Recording FPS: ", camera.getFps());
            //   telemetry.addData("Theoretical Max FPS: ", camera.getCurrentPipelineMaxFps());
            //}
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

    private void changeDriveMode() {
        if (currentDriveMode == DriveMode.ROBOT_CENTRIC) {
            currentDriveMode = DriveMode.AIM_ASSIST;
        } else {
            currentDriveMode = DriveMode.ROBOT_CENTRIC;
        }
    }

    private void setLedAllianceColor() {
        // Sets LED to show alliance color based on the last autonomous that was run
        if (InterOpStorage.alliance == InterOpStorage.Alliance.RED) {
            ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);
        } else if (InterOpStorage.alliance == InterOpStorage.Alliance.BLUE) {
            ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
        }
    }

    private void drawDashboard() {
        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, poseEstimate);

        // Draw the target on the field
        fieldOverlay.setStroke("#dd2c00");
        fieldOverlay.strokeCircle(targetPosition.getX(), targetPosition.getY(), 2);

        // Draw lines to target
        fieldOverlay.setStroke("#b89eff");
        fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), poseEstimate.getX(), poseEstimate.getY());
        fieldOverlay.setStroke("#ffce7a");
        fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), targetPosition.getX(), poseEstimate.getY());
        fieldOverlay.strokeLine(targetPosition.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, poseEstimate);
    }

    private void drawUltrasonicSensors() {
        Bitmap bitmap = Bitmap.createBitmap(1280, 720, Bitmap.Config.ARGB_8888);
        bitmap.setPixel(0, 0, Color.argb(255, 255, 0, 0));
        for (int i = 0; i < 233; i++) {
        }
        ftcDashboard.sendImage(bitmap);
    }

    private class JunctionComparator implements Comparator<Junction> {
        // override the compare() method
        public int compare(Junction j1, Junction j2) {
            double j1Distance = poseEstimate.vec().distTo(j1.getPositionVec2d());
            double j2Distance = poseEstimate.vec().distTo(j2.getPositionVec2d());
            double difference = j2Distance - j1Distance;
            if (difference > 0) return -1;
            else if (difference == 0) return 0;
            else return 1;
        }
    }
}