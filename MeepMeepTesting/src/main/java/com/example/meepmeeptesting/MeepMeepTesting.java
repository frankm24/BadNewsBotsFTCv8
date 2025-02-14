package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.awt.Robot;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        /*
        robot length: 15 in.
        robot width: 14.6 in.
         */
        float robotLength = 15.0f;
        float robotWidth = 16.25f;
        float tileSize = 23.5f;
        float coneDiameter = 4.0f;

        Pose2d redStartPoseParking1 = new Pose2d(-1.5 * tileSize, -3 * tileSize + robotWidth/2, Math.toRadians(180));
        Pose2d redStartPoseParking2 = new Pose2d(1.5 * tileSize, -3 * tileSize + robotWidth/2, Math.toRadians(180));

        Pose2d blueStartPoseParking1 = new Pose2d((1.5 * tileSize), 3 * tileSize - robotWidth/2, Math.toRadians(0));
        Pose2d blueStartPoseParking2 = new Pose2d(-(1.5 * tileSize), 3 * tileSize - robotWidth/2, Math.toRadians(0));

        Pose2d blueStartPose1 = new Pose2d((1.5 * tileSize), 3 * tileSize - robotWidth/2, Math.toRadians(180));
        Pose2d blueStartPose2 = new Pose2d(-(1.5 * tileSize), 3 * tileSize - robotWidth/2, Math.toRadians(180));

        Constraints constraints = new Constraints(52.48291908330528, 52.48291908330528,
                Math.toRadians(261.482587826087), Math.toRadians(261.482587826087), 11.38);

        RoadRunnerBotEntity robot = new RoadRunnerBotEntity(meepMeep, constraints, robotWidth, robotLength,
                new Pose2d(), new ColorSchemeBlueLight(), 1, DriveTrainType.MECANUM, true);

        Pose2d blue2InitialCenter = new Pose2d(blueStartPose2.getX(),
                blueStartPose2.getY() -(2.5*tileSize - robotWidth/2) - 6, Math.toRadians(180));
        Pose2d blue2BetweenGoals = blue2InitialCenter.minus(new Pose2d(12, 0, 0));
        Pose2d blue2AtCone = blue2InitialCenter.plus(new Pose2d(-(coneDiameter + tileSize), 0, 0));
        Pose2d blue2AtHighGoal = new Pose2d(-tileSize, 0, Math.toRadians(-42.5));

        Pose2d blue2ConeLeftPos = new Pose2d(-12, tileSize/2, Math.toRadians(180));
        Pose2d blue2ConeMidPos = new Pose2d(-35, tileSize/2, Math.toRadians(180));
        Pose2d blue2ConeRightPos = new Pose2d(-2.5*tileSize, tileSize/2, Math.toRadians(180));

        Pose2d blue1InitialCenter = new Pose2d(blueStartPose1.getX(),
                blueStartPose1.getY() - (2.5*tileSize - robotWidth/2) - 6, Math.toRadians(180));
        Pose2d blue1BetweenGoals = blue1InitialCenter.plus(new Pose2d(12, 0, Math.toRadians(-180)));
        Pose2d blue1AtCone = blue1InitialCenter.plus(new Pose2d((coneDiameter + tileSize), 0, 0));
        Pose2d blue1AtHighGoal = new Pose2d(tileSize, 0, Math.toRadians(-42.5));

        Pose2d blue1ConeLeftPos = new Pose2d(2.5*tileSize, tileSize/2, Math.toRadians(180));
        Pose2d blue1ConeMidPos = new Pose2d(35, tileSize/2, Math.toRadians(180));
        Pose2d blue1ConeRightPos = new Pose2d(12, tileSize/2, Math.toRadians(180));

        int coneGrabbingWaitTime = 1;
        int highGoalPlacementWaitTime = 1;

        TrajectorySequence blueAutoOneCone1_1 = robot.getDrive().trajectorySequenceBuilder(blueStartPose1)
                .lineTo(new Vector2d(blue1InitialCenter.getX(), blue1InitialCenter.getY()))
                .addTemporalMarker(() -> {

                })
                .lineToLinearHeading(new Pose2d(blue1AtHighGoal.getX(), blue1AtHighGoal.getY(), blue1AtHighGoal.getHeading() + Math.toRadians(270)))
                .addTemporalMarker(() -> {

                })
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(blue1BetweenGoals.getX(), blue1BetweenGoals.getY()), blue1BetweenGoals.getHeading())
                .setReversed(true)
                .lineTo(new Vector2d(blue1ConeLeftPos.getX(), blue1ConeLeftPos.getY()))
                .build();
        TrajectorySequence blueAutoOneCone1_2 = robot.getDrive().trajectorySequenceBuilder(blueStartPose1)
                .lineTo(new Vector2d(blue1InitialCenter.getX(), blue1InitialCenter.getY()))
                .addTemporalMarker(() -> {

                })
                .lineToLinearHeading(new Pose2d(blue1AtHighGoal.getX(), blue1AtHighGoal.getY(), blue1AtHighGoal.getHeading() + Math.toRadians(270)))
                .addTemporalMarker(() -> {

                })
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(blue1BetweenGoals.getX(), blue1BetweenGoals.getY()), blue1BetweenGoals.getHeading())
                .setReversed(true)
                .lineTo(new Vector2d(blue1ConeMidPos.getX(), blue1ConeMidPos.getY()))
                .build();

        TrajectorySequence blueAutoOneCone1_3 = robot.getDrive().trajectorySequenceBuilder(blueStartPose1)
                .lineTo(new Vector2d(blue1InitialCenter.getX(), blue1InitialCenter.getY()))
                .addTemporalMarker(() -> {

                })
                .lineToLinearHeading(new Pose2d(blue1AtHighGoal.getX(), blue1AtHighGoal.getY(), blue1AtHighGoal.getHeading() + Math.toRadians(270)))
                .addTemporalMarker(() -> {

                })
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(blue1BetweenGoals.getX(), blue1BetweenGoals.getY()), blue1BetweenGoals.getHeading())
                .setReversed(true)
                .lineTo(new Vector2d(blue1ConeRightPos.getX(), blue1ConeRightPos.getY()))
                .build();

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

        robot.followTrajectorySequence(blueAutoOneCone1_3);

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(0.95f)
                .addEntity(robot)
                .start();
    }
}
//LAtest blue auto 2 1
/*
TrajectorySequence blueAuto2_1 = robot.getDrive().trajectorySequenceBuilder(blueStartPose2)
                .lineTo(new Vector2d(initialCenter.getX(), initialCenter.getY()))
                .lineToLinearHeading(new Pose2d(atHighGoal.getX(), atHighGoal.getY(), atHighGoal.getHeading() + Math.toRadians(180)))
                .splineTo(new Vector2d(betweenGoals.getX(), betweenGoals.getY()), betweenGoals.getHeading())
                .addTemporalMarker(() -> {

                })
                .waitSeconds(.5)
                // START BLOCK to LOOP
                .lineTo(new Vector2d(atCone.getX(), atCone.getY()))
                .addTemporalMarker(() -> {

                })
                .waitSeconds(coneGrabbingWaitTime)
                .setReversed(true)
                .lineToLinearHeading(betweenGoals)
                .splineTo(new Vector2d(atHighGoal.getX(), atHighGoal.getY()), atHighGoal.getHeading())
                .addTemporalMarker(() -> {

                })
                .waitSeconds(highGoalPlacementWaitTime)
                .setReversed(false)
                .splineTo(new Vector2d(betweenGoals.getX(), betweenGoals.getY()), betweenGoals.getHeading())
                // END BLOCK to LOOP
                .build();
 */


/*
TrajectorySequence blueAuto2_1 = robot.getDrive().trajectorySequenceBuilder(blueStartPoseParking2)
                .strafeRight(2*tileSize)
                .lineToLinearHeading(center)
                .lineToLinearHeading(atCone)
                .setReversed(true)
                .lineToLinearHeading(betweenGoals)
                .splineTo(new Vector2d(atHighGoal.getX(), atHighGoal.getY()), atHighGoal.getHeading())
                .build();
 */

/*
                .back(tileSize)
                .addTemporalMarker(() -> {
                    //
                })
                .forward(2.5)
                .lineTo(new Vector2d( -2.5*tileSize + 2.5, tileSize/2))
                .lineTo(new Vector2d(-3*tileSize + robotLength/2, tileSize/2))
                .addTemporalMarker(() -> {
                    // pick up next cone
                    // tell linear mechanism to begin moving up, rotating as necessary
                })
                .waitSeconds(0.5)
                .setReversed(false)
                .lineTo(new Vector2d(-2*tileSize, tileSize/2))
                .splineTo(new Vector2d(-tileSize - 6.9, 6.4), Math.toRadians(-42.5))
                .setReversed(true)
                .addTemporalMarker(() -> {

                })
                .waitSeconds(.5)
                .splineTo(new Vector2d(-2*tileSize, tileSize/2), Math.toRadians(180))
                .lineTo(new Vector2d( -2.5*tileSize + 2.5, tileSize/2))
                .addTemporalMarker(() -> {

                })
                .waitSeconds(.5)
                .setReversed(false)
                // end of cycle
                */

/*
                .splineTo(new Vector2d(-1.5*tileSize + 7, robotWidth/2 + 0.75), Math.toRadians(-54))
                .setReversed(true)
                .addTemporalMarker(() -> {

                })
                .waitSeconds(.5)
                .splineTo(new Vector2d( -2.5*tileSize -1, tileSize/2), Math.toRadians(180))
                .addTemporalMarker(() -> {

                })
                .waitSeconds(.5)
                .setReversed(false)
                .splineTo(new Vector2d(-1.5*tileSize + 7, robotWidth/2 + 0.75), Math.toRadians(-54))
                .setReversed(true)
                .addTemporalMarker(() -> {

                })
                .waitSeconds(.5)
                .splineTo(new Vector2d( -2.5*tileSize -1, tileSize/2), Math.toRadians(180))
                .addTemporalMarker(() -> {

                })
                .waitSeconds(.5)
                .setReversed(false)
                .splineTo(new Vector2d(-1.5*tileSize + 7, robotWidth/2 + 0.75), Math.toRadians(-54))
                .setReversed(true)
                .addTemporalMarker(() -> {

                })
                .waitSeconds(.5)
                .splineTo(new Vector2d( -2.5*tileSize -1, tileSize/2), Math.toRadians(180))
                .addTemporalMarker(() -> {

                })
                .waitSeconds(.5)
                .setReversed(false)
                .splineTo(new Vector2d(-1.5*tileSize + 7, robotWidth/2 + 0.75), Math.toRadians(-54))
                .setReversed(true)
                .addTemporalMarker(() -> {

                })
                .waitSeconds(.5)
                .splineTo(new Vector2d( -2.5*tileSize -1, tileSize/2), Math.toRadians(180))
                .waitSeconds(0.5)
                .strafeLeft(tileSize)
                //.splineTo
                 */