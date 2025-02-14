package badnewsbots.pipelines;

import android.os.Environment;

import com.acmerobotics.roadrunner.drive.MecanumDrive;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.PipelineRecordingParameters;

import badnewsbots.robots.MecanumBotDriveOnly;

public class RecordingPipeline extends OpenCvPipeline {
    public final boolean record;
    private final OpenCvCamera camera;

    public RecordingPipeline(OpenCvCamera camera, boolean record) {
        this.record = record;
        this.camera = camera;
    }

    @Override
    public Mat processFrame(Mat input) {
        return input;
    }

    @Override
    public void init(Mat input) {
        if (record) {
            camera.startRecordingPipeline(
                    new PipelineRecordingParameters.Builder()
                            .setBitrate(4, PipelineRecordingParameters.BitrateUnits.Mbps)
                            .setEncoder(PipelineRecordingParameters.Encoder.H264)
                            .setOutputFormat(PipelineRecordingParameters.OutputFormat.MPEG_4)
                            .setFrameRate(30)
                            .setPath(Environment.getExternalStorageDirectory() + "/pipeline_rec_" + ".mp4")
                            .build());
        }
    }
}
