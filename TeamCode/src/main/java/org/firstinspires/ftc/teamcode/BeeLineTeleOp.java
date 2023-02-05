package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import badnewsbots.hardware.GamepadEx;
import badnewsbots.robots.BeeLineChassis;

@Disabled
@TeleOp
public final class BeeLineTeleOp extends LinearOpMode {
    // Robot object
    private BeeLineChassis robot;

    private float SpeedMultiplier = 1.0f; // scale movement speed
    private int flywheelTargetSpeed = 1000;
    private boolean flywheelOn = false;

    private enum PusherState {
        IN,
        MOVING_OUT,
        OUT,
        MOVING_IN
    }
    private PusherState pusherState = PusherState.IN;
    private double pusherTime = 0;

    private FtcDashboard ftcDashboard;

    private GamepadEx smartGamepad;

    public void mainLoop() {
        double prevTime = 0;
        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double deltaTime = currentTime - prevTime;
            smartGamepad.update();
            float LeftStickY = -1 * smartGamepad.left_stick_y * SpeedMultiplier;
            float LeftStickX = smartGamepad.left_stick_x * SpeedMultiplier;
            float RightStickY = -1 * smartGamepad.right_stick_y * SpeedMultiplier;
            float RightStickX = smartGamepad.right_stick_x * SpeedMultiplier;

            if (smartGamepad.start_pressed) {
                if (SpeedMultiplier == 0.5f) {
                    SpeedMultiplier = 1.0f;
                } else {
                    SpeedMultiplier = 0.5f;
                }
            }
            // In case the flywheel defaults to the wrong direction :)
            if (smartGamepad.y_pressed) {flywheelTargetSpeed *= -1;}
            if (smartGamepad.dpad_up_pressed) {flywheelTargetSpeed += 500;}
            if (smartGamepad.dpad_down_pressed) {flywheelTargetSpeed -= 500;}
            if (smartGamepad.a_pressed) {flywheelOn = !flywheelOn;}
            if (smartGamepad.right_trigger_pressed) {
                telemetry.addLine("right trigger pressed");
                if (pusherState == PusherState.IN) {
                    pusherState = PusherState.MOVING_OUT;
                    pusherTime = currentTime;
                    robot.pusher.setPosition(0);
                }
            }
            if (pusherState == PusherState.MOVING_OUT) {
                if (currentTime - pusherTime >= 0.460) {
                    robot.pusher.setPosition(0.48);
                    pusherTime = currentTime;
                    pusherState = PusherState.MOVING_IN;
                }
            }
            if (pusherState == PusherState.MOVING_IN) {
                if (currentTime - pusherTime >= 0.460) {
                    pusherState = PusherState.IN;
                }
            }
            if (smartGamepad.start_pressed) {
                if (SpeedMultiplier == 0.5f) {
                    SpeedMultiplier = 1.0f;
                } else {
                    SpeedMultiplier = 0.5f;
                }
            }

            if (flywheelOn) {
                robot.flywheel.setVelocity(flywheelTargetSpeed);
            } else {
                robot.flywheel.setVelocity(0);
            }

            double front_leftPower = LeftStickY - RightStickX;
            double back_leftPower = LeftStickY - RightStickX;
            double front_rightPower = LeftStickY + RightStickX;
            double back_rightPower = LeftStickY + RightStickX;

            robot.front_left.setPower(front_leftPower);
            robot.back_left.setPower(back_leftPower);
            robot.front_right.setPower(front_rightPower);
            robot.back_right.setPower(back_rightPower);

            telemetry.addData("Front left power (%) ", front_leftPower);
            telemetry.addData("Back left power (%) ", back_leftPower);
            telemetry.addData("Front right power (%) ", front_rightPower);
            telemetry.addData("Back right power (%) ", back_rightPower);
            telemetry.addData("Control step time: ", deltaTime);
            telemetry.addData("IMU Data", robot.imu.getAngularOrientation());
            telemetry.update();
            prevTime = currentTime;
        }
    }

    @Override
    public void runOpMode() {
        smartGamepad = new GamepadEx(gamepad1);
        robot = new BeeLineChassis(this);
        ftcDashboard = FtcDashboard.getInstance();
        //telemetry = new MultipleTelemetry(telemetry, ftcDashboard.getTelemetry());

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status: ", "Initialized");
            telemetry.update();
            idle();
        }
        mainLoop();
    }
}



