package badnewsbots.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.PowerPlayCompBotMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

import badnewsbots.hardware.RotatingClaw;
import badnewsbots.pipelines.SignalSleevePipeline;
import badnewsbots.robots.PowerPlayCompBot;

// This abstract class defines the logic behind an autonomous program which detects the orientation of the signal sleeve
// (1, 2, or 3), and follows the corresponding RoadRunnerTrajectorySequence (which must be provided when this class is extended),
// executing the correct autonomous plan.
public abstract class SignalSleeveAutonomous extends LinearOpMode {
    protected PowerPlayCompBot robot;
    private SignalSleevePipeline.ConeOrientation coneOrientation;
    protected PowerPlayCompBotMecanumDrive drive;
    protected RotatingClaw claw;

    protected final double tileSize = 23.5;
    private OpenCvCamera camera;

    protected TrajectorySequence trajectory1;
    protected TrajectorySequence trajectory2;
    protected TrajectorySequence trajectory3;
    protected SignalSleevePipeline.CameraOrientation cameraOrientation;

    @Override
    public void runOpMode() {
        robot = new PowerPlayCompBot(this);
        drive = robot.getDrive();
        claw = robot.getRotatingClaw();

        if (cameraOrientation == SignalSleevePipeline.CameraOrientation.RIGHT)
            camera = robot.getRightCamera();
        else if (cameraOrientation == SignalSleevePipeline.CameraOrientation.LEFT)
            camera = robot.getLeftCamera();

        initializeAutonomousTrajectories();

        SignalSleevePipeline pipeline = new SignalSleevePipeline(cameraOrientation);
        initOpenCV(pipeline);

        while (!isStarted() && !isStopRequested()) {
            coneOrientation = pipeline.getConeOrientation();
            double[] colorFilterAverages = pipeline.getFilterAverages();
            telemetry.addData("Status: ", "Initialized");
            telemetry.addData("Cone filter averages: (G, M, O)", Arrays.toString(colorFilterAverages));
            telemetry.addData("Cone orientation: ", coneOrientation);
            telemetry.addData("FPS: ", camera.getFps());
            telemetry.update();
            idle();
        }

        preTrajectory();

        if (coneOrientation == SignalSleevePipeline.ConeOrientation.ONE) {
            drive.followTrajectorySequence(trajectory1);
        }
        if (coneOrientation == SignalSleevePipeline.ConeOrientation.TWO) {
            drive.followTrajectorySequence(trajectory2);
        }
        if (coneOrientation == SignalSleevePipeline.ConeOrientation.THREE) {
            drive.followTrajectorySequence(trajectory3);
        }

        postTrajectory();
    }

    protected abstract void initializeAutonomousTrajectories();
    protected abstract void preTrajectory();
    protected abstract void postTrajectory();

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
