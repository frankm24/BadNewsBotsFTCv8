package org.firstinspires.ftc.teamcode.ppauto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.PowerPlayCompBotMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

import badnewsbots.InterOpStorage;
import badnewsbots.hardware.GamepadEx;
import badnewsbots.hardware.RotatingClaw;
import badnewsbots.pipelines.SignalSleevePipeline;
import badnewsbots.robots.PowerPlayCompBot;

@Autonomous
public final class AutoBlue2OneCone extends LinearOpMode {

    private PowerPlayCompBot robot;
    private GamepadEx smartGamepad;
    private FtcDashboard ftcDashboard;
    private SignalSleevePipeline.ConeOrientation coneOrientation;
    private double[] colorFilterAverages;
    private PowerPlayCompBotMecanumDrive drive;
    private RotatingClaw claw;

    private final double tileSize = 23.5;
    private OpenCvCamera camera;

    private TrajectorySequence blueAutoOneCone2_1;
    private TrajectorySequence blueAutoOneCone2_2;
    private TrajectorySequence blueAutoOneCone2_3;
    private Pose2d blueStartPose2;

    private static double xoff = 0;
    private static double yoff = -2;
    private static double hoff = 0;

    @Override
    public void runOpMode() {
        robot = new PowerPlayCompBot(this);
        drive = robot.getDrive();
        claw = robot.getRotatingClaw();

        initializeAutonomousTrajectories();

        camera = robot.getLeftCamera();
        smartGamepad = new GamepadEx(gamepad1);
        ftcDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, ftcDashboard.getTelemetry());

        SignalSleevePipeline pipeline = new SignalSleevePipeline(SignalSleevePipeline.CameraOrientation.LEFT);
        initOpenCV(pipeline);

        InterOpStorage.alliance = InterOpStorage.Alliance.BLUE;
        claw.rotateToSide(RotatingClaw.Side.BACK);
        claw.grip();

        while (!isStarted() && !isStopRequested()) {
            coneOrientation = pipeline.getConeOrientation();
            colorFilterAverages = pipeline.getFilterAverages();
            telemetry.addData("Status: ", "Initialized");
            telemetry.addData("Cone filter averages: (G, M, O)", Arrays.toString(colorFilterAverages));
            telemetry.addData("Cone orientation: ", coneOrientation);
            telemetry.addData("FPS: ", camera.getFps());
            telemetry.update();
            idle();
        }

        drive.setPoseEstimate(blueStartPose2);
        claw.moveSlidesToPresetHeight(RotatingClaw.SlideHeight.HIGH_GOAL);

        if (coneOrientation == SignalSleevePipeline.ConeOrientation.ONE) {
            drive.followTrajectorySequence(blueAutoOneCone2_1);
        }
        if (coneOrientation == SignalSleevePipeline.ConeOrientation.TWO) {
            drive.followTrajectorySequence(blueAutoOneCone2_2);
        }
        if (coneOrientation == SignalSleevePipeline.ConeOrientation.THREE) {
            drive.followTrajectorySequence(blueAutoOneCone2_3);
        }
        claw.moveSlidesToPresetHeight(RotatingClaw.SlideHeight.BOTTOM);
        InterOpStorage.currentPose = drive.getPoseEstimate();
    }

    private void initializeAutonomousTrajectories() {
        float coneDiameter = 4.0f;
        blueStartPose2 = new Pose2d(-(1.5 * tileSize), 3 * tileSize - robot.width/2, Math.toRadians(180));
        Pose2d blue2InitialCenter = new Pose2d(blueStartPose2.getX(),
                blueStartPose2.getY() -(2.5*tileSize - robot.width/2) - 6, Math.toRadians(180));
        Pose2d blue2BetweenGoals = blue2InitialCenter.minus(new Pose2d(12, 0, 0));
        Pose2d blue2AtCone = blue2InitialCenter.plus(new Pose2d(-(coneDiameter + tileSize), 0, 0));
        Pose2d blue2AtHighGoal = new Pose2d(-tileSize, 0, Math.toRadians(-42.5));

        Pose2d blue2ConeLeftPos = new Pose2d(-12, tileSize/2, Math.toRadians(180));
        Pose2d blue2ConeMidPos = new Pose2d(-35, tileSize/2, Math.toRadians(180));
        Pose2d blue2ConeRightPos = new Pose2d(-2.5*tileSize, tileSize/2, Math.toRadians(180));

        TrajectorySequence blueAutoOneCone2_1 = robot.getDrive().trajectorySequenceBuilder(blueStartPose2)
                .lineTo(new Vector2d(blue2InitialCenter.getX(), blue2InitialCenter.getY()))
                .addTemporalMarker(() -> {

                })
                .lineToLinearHeading(new Pose2d(blue2AtHighGoal.getX(), blue2AtHighGoal.getY(), blue2AtHighGoal.getHeading() + Math.toRadians(180)))
                .addTemporalMarker(() -> {

                })
                .waitSeconds(1)
                .splineTo(new Vector2d(blue2BetweenGoals.getX(), blue2BetweenGoals.getY()), blue2BetweenGoals.getHeading())
                .setReversed(true)
                .lineTo(new Vector2d(blue2ConeLeftPos.getX(), blue2ConeLeftPos.getY()))
                .build();

        TrajectorySequence blueAutoOneCone2_2 = robot.getDrive().trajectorySequenceBuilder(blueStartPose2)
                .lineTo(new Vector2d(blue2InitialCenter.getX(), blue2InitialCenter.getY()))
                .addTemporalMarker(() -> {

                })
                .lineToLinearHeading(new Pose2d(blue2AtHighGoal.getX(), blue2AtHighGoal.getY(), blue2AtHighGoal.getHeading() + Math.toRadians(180)))
                .addTemporalMarker(() -> {

                })
                .waitSeconds(1)
                .splineTo(new Vector2d(blue2BetweenGoals.getX(), blue2BetweenGoals.getY()), blue2BetweenGoals.getHeading())
                .setReversed(true)
                .lineTo(new Vector2d(blue2ConeMidPos.getX(), blue2ConeMidPos.getY()))
                .build();

        TrajectorySequence blueAutoOneCone2_3 = robot.getDrive().trajectorySequenceBuilder(blueStartPose2)
                .lineTo(new Vector2d(blue2InitialCenter.getX(), blue2InitialCenter.getY()))
                .addTemporalMarker(() -> {

                })
                .lineToLinearHeading(new Pose2d(blue2AtHighGoal.getX(), blue2AtHighGoal.getY(), blue2AtHighGoal.getHeading() + Math.toRadians(180)))
                .addTemporalMarker(() -> {

                })
                .waitSeconds(1)
                .splineTo(new Vector2d(blue2BetweenGoals.getX(), blue2BetweenGoals.getY()), blue2BetweenGoals.getHeading())
                .setReversed(true)
                .lineTo(new Vector2d(blue2ConeRightPos.getX(), blue2ConeRightPos.getY()))
                .build();
    }

    private void initOpenCV(OpenCvPipeline pipeline) {
        camera.setPipeline(pipeline);
        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        //camera.showFpsMeterOnViewport(true);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //ftcDashboard.getTelemetry().addLine("Camera stream initialized");
                telemetry.update();
                camera.startStreaming(640, 480);
                //ftcDashboard.startCameraStream(camera, 30);
            }
            @Override
            public void onError(int errorCode) {
                throw new OpenCvCameraException("Could not open camera device. Error code: " + errorCode) ;
            }
        });
    }
}


