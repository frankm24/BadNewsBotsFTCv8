package badnewsbots.pipelines;

import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class SignalSleevePipeline extends OpenCvPipeline {
    private final boolean returnResult = true;
    private final boolean saveImage = false;

    public enum ConeOrientation {
        ONE,
        TWO,
        THREE,
        NONE
    }

    public enum CameraOrientation {
        LEFT,
        RIGHT
    }

    private ConeOrientation coneOrientation = ConeOrientation.NONE;
    private CameraOrientation cameraOrientation;

    private final Scalar greenMin = new Scalar(45, 50, 50);
    private final Scalar greenMax = new Scalar(70, 255, 255);

    private final Scalar magentaMin = new Scalar(150, 50, 50);
    private final Scalar magentaMax = new Scalar(170, 200, 200);

    private final Scalar orangeMin = new Scalar(13, 50, 50);
    private final Scalar orangeMax = new Scalar(20, 255, 255);

    private final Scalar roiOutlineColor = new Scalar(255, 0, 0);
    private final Scalar green = new Scalar(0, 255, 0);
    private final Scalar magenta = new Scalar(255, 0, 255);
    private final Scalar orange = new Scalar(255, 127, 0);

    private Point point1Right = new Point(300, 440);
    private Point point2Right = new Point(430, 350);
    private final Rect roiRight = new Rect(point1Right, point2Right);

    private Point point1Left = new Point(365, 350);
    private Point point2Left = new Point(480, 240);
    private final Rect roiLeft = new Rect(point1Left, point2Left);
    private final Rect roiToUse;

    private RevBlinkinLedDriver ledDriver;
    private boolean useLedStatusIndicator;

    private int greenCount;
    private int magentaCount;
    private int orangeCount;

    private Size pipelineSize;

    private Mat hsvImage;
    private Mat greenFiltered;
    private Mat orangeFiltered;
    private Mat magentaFiltered;

    public SignalSleevePipeline(CameraOrientation cameraOrientation) {
        this.cameraOrientation = cameraOrientation;
        if (cameraOrientation == CameraOrientation.RIGHT) roiToUse = roiRight;
        else roiToUse = roiLeft;
        useLedStatusIndicator = false;
    }

    public SignalSleevePipeline(CameraOrientation cameraOrientation, RevBlinkinLedDriver ledDriver) {
        this(cameraOrientation);
        this.useLedStatusIndicator = true;
        this.ledDriver = ledDriver;
    }

    @Override
    public void init(Mat input) {
        pipelineSize = input.size();
        hsvImage = new Mat();

        greenFiltered = new Mat();
        magentaFiltered = new Mat();
        orangeFiltered = new Mat();
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvImage, greenMin, greenMax, greenFiltered);
        Core.inRange(hsvImage, magentaMin, magentaMax, magentaFiltered);
        Core.inRange(hsvImage, orangeMin, orangeMax, orangeFiltered);

        Mat greenROI = greenFiltered.submat(roiToUse);
        Mat magentaROI = magentaFiltered.submat(roiToUse);
        Mat orangeROI = orangeFiltered.submat(roiToUse);

        greenCount = Core.countNonZero(greenROI);
        magentaCount = Core.countNonZero(magentaROI);
        orangeCount = Core.countNonZero(orangeROI);

        double maxMean = Math.max(magentaCount, Math.max(greenCount, orangeCount));

        RevBlinkinLedDriver.BlinkinPattern currentPattern;
        if (maxMean == greenCount) {
            coneOrientation = ConeOrientation.ONE;
            currentPattern = RevBlinkinLedDriver.BlinkinPattern.LIME;
        } else if (maxMean == magentaCount) {
            coneOrientation = ConeOrientation.TWO;
            currentPattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
        } else if (maxMean == orangeCount) {
            coneOrientation = ConeOrientation.THREE;
            currentPattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
        } else {
            coneOrientation = ConeOrientation.NONE;
            currentPattern = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;
        }

        // Code to visualize results (NOT vital)
        input.setTo(green, greenFiltered);
        input.setTo(magenta, magentaFiltered);
        input.setTo(orange, orangeFiltered);
        Imgproc.rectangle(input, roiToUse, roiOutlineColor, 3);

        if (useLedStatusIndicator) {
            ledDriver.setPattern(currentPattern);
        }
        if (saveImage) Imgcodecs.imwrite(Environment.getExternalStorageDirectory() + "/signal.png", hsvImage);

        if (returnResult) {return input;} else {return input;}
    }

    public ConeOrientation getConeOrientation() {return coneOrientation;}

    public double[] getFilterAverages() {
        return new double[] {greenCount, magentaCount, orangeCount};
    }
}
