package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;
import java.util.List;

import badnewsbots.hardware.GamepadEx;

// This OpMode acts as a tool for testing the control of DC motors using the REV Control Hub as part of a mechanical subsystem of a robot.
@TeleOp(group = "Testing")
public final class MechanismMotorTest extends LinearOpMode {
    private DcMotorEx motor;
    private DcMotorEx motor1;
    private GamepadEx smartGamepad;
    private double motorPower;
    private int motorTargetPosition;
    private final List<DcMotorEx> motorsToTest = new ArrayList<>();

    enum TestMode {
        POSITION,
        POWER
    }
    private TestMode currentTestMode;

    private final int minTicks = -3251;
    private final int maxTicks = 0;

    @Override
    public void runOpMode() {
        smartGamepad = new GamepadEx(gamepad1);
        motor = hardwareMap.get(DcMotorEx.class, "linearSlide1");
        motor1 = hardwareMap.get(DcMotorEx.class, "linearSlide2");
        //motorsToTest.add(motor);
        motorsToTest.add(motor);

        motorPower = 0;
        motorTargetPosition = 0;
        currentTestMode = TestMode.POWER;

        telemetry.addLine("Press Y to toggle test mode between POWER (pure PWM control) and POSITION (encoders + motor driver built-in PIDF)");
        telemetry.addLine("POWER mode: Press DPAD UP/DOWN to inc/dec power, press A to set power to 0, and press B to set power to -0.1");
        telemetry.addLine("POSITION mode: Press DPAD UP/DOWN to inc/dec target pos., press A to set target pos. to 0");
        for (int i = 0; i < motorsToTest.size(); i++) {
            telemetry.addData("Motor " + i + " PIDF Coeffs: ", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        }
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            smartGamepad.update();

            if (smartGamepad.yPressed()) {
                if (currentTestMode == TestMode.POWER) {
                    currentTestMode = TestMode.POSITION;
                    for (DcMotorEx motor : motorsToTest) {
                        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        motor.setPower(0.5);
                        motorTargetPosition = minTicks;
                    }
                } else {
                    currentTestMode = TestMode.POWER;
                    for (DcMotorEx motor : motorsToTest) {
                        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        motorPower = 0;
                    }
                }
            }
            if (currentTestMode == TestMode.POWER) {
                if (smartGamepad.dpadUp()) {
                    motorPower += 0.01;
                }
                if (smartGamepad.dpadDown()) {
                    motorPower -= 0.01;
                }
                if (smartGamepad.aPressed()) {
                    motorPower = 0;
                }
                if (smartGamepad.bPressed()) {
                    motorPower = -0.1;
                }
                for (DcMotorEx motor : motorsToTest) {
                    motor.setPower(motorPower);
                }
            } else {
                if (smartGamepad.dpadUp()) {
                    motorTargetPosition = Math.min(motorTargetPosition + 2, maxTicks);
                }
                if (smartGamepad.dpadDown()) {
                    motorTargetPosition = Math.max(motorTargetPosition - 2, minTicks);
                }
                if (smartGamepad.aPressed()) {
                    motorTargetPosition = 0;
                }
                for (DcMotorEx motor : motorsToTest) {
                    motor.setTargetPosition(motorTargetPosition);
                }
            }
            //-3251
            telemetry.addData("Test mode: ", currentTestMode);
            for (int i = 0; i < motorsToTest.size(); i++) {
                telemetry.addData("Motor " + i + " position: ", motor.getCurrentPosition());
            }
            telemetry.addData("Motor power: ", motorPower);
            telemetry.addData("Motor target position: ", motorTargetPosition);
            telemetry.update();
        }
    }
}
