package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.PowerPlayCompBotMecanumDrive;

@Config
@Autonomous(group = "drive")
public class LeftAndRight extends LinearOpMode {

    public static double DISTANCE = 50;

    @Override
    public void runOpMode() throws InterruptedException {
        PowerPlayCompBotMecanumDrive drive = new PowerPlayCompBotMecanumDrive(hardwareMap);

        Trajectory trajectoryLeft = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(DISTANCE)
                .build();

        Trajectory trajectoryRight = drive.trajectoryBuilder(trajectoryLeft.end())
                .strafeRight(DISTANCE)
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(trajectoryLeft);
            drive.followTrajectory(trajectoryRight);
        }
    }
}
