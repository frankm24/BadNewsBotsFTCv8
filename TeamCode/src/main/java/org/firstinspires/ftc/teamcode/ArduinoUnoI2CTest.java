package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import badnewsbots.hardware.ArduinoUNO_R2;

@TeleOp
public class ArduinoUnoI2CTest extends LinearOpMode {
    private ArduinoUNO_R2 arduinoUno;
    public void runOpMode() {
        arduinoUno = hardwareMap.get(ArduinoUNO_R2.class, "Arduino");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while (opModeIsActive()) {
            String currentMessage = arduinoUno.readMessage();
            telemetry.addData("Current message", currentMessage);
            telemetry.update();
        }
    }
}
