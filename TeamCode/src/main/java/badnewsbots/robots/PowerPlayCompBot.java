package badnewsbots.robots;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.PowerPlayCompBotMecanumDrive;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import badnewsbots.hardware.RotatingClaw;

public final class PowerPlayCompBot {
    // OOP
    private OpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public final double length = 15.0;
    public final double width = 16.25;
    private final PowerPlayCompBotMecanumDrive drive;
    private  OpenCvWebcam rightCamera;
    private  OpenCvWebcam leftCamera;
    private final VoltageSensor voltageSensor;

    private boolean leftCameraPluggedIn = false;
    private boolean rightCameraPluggedIn = false;

    // Mechanisms
    private final RotatingClaw rotatingClaw;
    private final RevBlinkinLedDriver ledDriver;

    public PowerPlayCompBot(OpMode opMode) {
        this.opMode = opMode;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        // Enables automatic "bulk reads" from robot hardware, so multiple encoder .get()'s are combined into for the lynx module
        // Should improve performance significantly, since hardwareMap read calls take 2ms each
        for (LynxModule module : hardwareMap.getAll( LynxModule.class ) )
            module.setBulkCachingMode( LynxModule.BulkCachingMode.AUTO );

        drive = new PowerPlayCompBotMecanumDrive(hardwareMap);
        rotatingClaw = new RotatingClaw(hardwareMap, telemetry);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        // Cameras
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        try {
            WebcamName webcamName1 = hardwareMap.get(WebcamName.class, "Webcam 1");
            leftCamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName1);
            leftCameraPluggedIn = true;
        } catch (Exception e) {
            leftCameraPluggedIn = false;
        }
        try {
            WebcamName webcamName2 = hardwareMap.get(WebcamName.class, "Webcam 2");
            rightCamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName2);
            rightCameraPluggedIn = true;
        } catch (Exception e) {
            rightCameraPluggedIn = false;
        }

        ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "led_driver");
        //telemetry.addLine("Robot object created");
        //telemetry.update();
    }

    public boolean isLeftCameraPluggedIn() {
        return leftCameraPluggedIn;
    }
    public boolean isRightCameraPluggedIn() {
        return rightCameraPluggedIn;
    }
    public PowerPlayCompBotMecanumDrive getDrive() {return drive;}
    public OpenCvWebcam getLeftCamera() {return leftCamera;}
    public OpenCvWebcam getRightCamera() {return rightCamera;}
    public RotatingClaw getRotatingClaw() {return rotatingClaw;}
    public RevBlinkinLedDriver getLedDriver() {return ledDriver;}
}
