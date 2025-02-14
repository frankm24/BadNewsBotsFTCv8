package org.firstinspires.ftc.teamcode.ppauto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import badnewsbots.InterOpStorage;
import badnewsbots.hardware.RotatingClaw;
import badnewsbots.opmode.SignalSleeveAutonomous;
import badnewsbots.pipelines.SignalSleevePipeline;

@Autonomous
public final class AutoBlue2HighGoal extends SignalSleeveAutonomous {

    private Pose2d blueStartPose2;

    private static double xoff = 0;
    private static double yoff = -2;
    private static double hoff = 0;

    @Override
    protected void initializeAutonomousTrajectories() {
        cameraOrientation = SignalSleevePipeline.CameraOrientation.RIGHT;

        float coneDiameter = 4.0f;
        blueStartPose2 = new Pose2d(-(1.5 * tileSize), 3 * tileSize - robot.width/2, Math.toRadians(180));
        Pose2d initialCenter = new Pose2d(blueStartPose2.getX() + xoff,
                blueStartPose2.getY() -(2.5*tileSize - robot.width/2) + yoff , Math.toRadians(180) + hoff);
        Pose2d betweenGoals = initialCenter.minus(new Pose2d(12 + xoff, 0 + yoff, 0 + hoff));
        Pose2d atCone = initialCenter.plus(new Pose2d(-(coneDiameter + tileSize),  0, 0));
        Pose2d atHighGoal = new Pose2d(-tileSize, 0, Math.toRadians(-42.5));
        int coneGrabbingWaitTime = 1;
        int highGoalPlacementWaitTime = 1;

        trajectory1 = robot.getDrive().trajectorySequenceBuilder(blueStartPose2)
                .lineTo(new Vector2d(initialCenter.getX(), initialCenter.getY()))
                /*
                .addTemporalMarker(() -> {
                    while(opModeIsActive()) {
                        telemetry.addData("Pose", drive.getPoseEstimate());
                        telemetry.update();
                    }
                })
                .waitSeconds(99999)
                 */
                .addTemporalMarker(() -> {
                    claw.rotateToSide(RotatingClaw.Side.FRONT);
                })
                .lineToLinearHeading(new Pose2d(atHighGoal.getX(), atHighGoal.getY(), atHighGoal.getHeading() + Math.toRadians(180)))
                .addTemporalMarker(() -> {
                    claw.release();
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    claw.rotateToSide(RotatingClaw.Side.BACK);
                    claw.moveSlidesToPresetHeight(RotatingClaw.SlideHeight.TOP_TWO_CONES);
                })
                .splineTo(new Vector2d(betweenGoals.getX(), betweenGoals.getY()), betweenGoals.getHeading())
                .waitSeconds(.5)
                // START BLOCK to LOOP
                .lineTo(new Vector2d(atCone.getX(), atCone.getY()))
                .addTemporalMarker(() -> {
                    claw.grip();
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    claw.gripRaiseAndRotate();
                    claw.moveSlidesToPresetHeight(RotatingClaw.SlideHeight.HIGH_GOAL);
                })
                .waitSeconds(coneGrabbingWaitTime)
                .setReversed(true)
                .lineToLinearHeading(betweenGoals)
                .splineTo(new Vector2d(atHighGoal.getX(), atHighGoal.getY()), atHighGoal.getHeading())
                .addTemporalMarker(() -> {
                    claw.release();
                    claw.rotateToSide(RotatingClaw.Side.BACK);
                    claw.moveSlidesToPresetHeight(RotatingClaw.SlideHeight.TOP_TWO_CONES);
                })
                .waitSeconds(highGoalPlacementWaitTime)
                .setReversed(false)
                .splineTo(new Vector2d(betweenGoals.getX(), betweenGoals.getY()), betweenGoals.getHeading())
                // END BLOCK to LOOP
                .build();
    }
    @Override
    protected void preTrajectory() {
        drive.setPoseEstimate(blueStartPose2);
        claw.moveSlidesToPresetHeight(RotatingClaw.SlideHeight.HIGH_GOAL);
    }
    @Override
    protected void postTrajectory() {
        claw.moveSlidesToPresetHeight(RotatingClaw.SlideHeight.BOTTOM);
        InterOpStorage.currentPose = drive.getPoseEstimate();
    }
}


