package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class CloseAutonTesting {

  public static void main(String[] args) {
    MeepMeep meepMeep = new MeepMeep(800);

    RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
        // CENTER CLOSE
        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
        .followTrajectorySequence(drive ->
                drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(-90)))
                    .setTangent(0)
                    .splineToLinearHeading(new Pose2d(48, 36, Math.toRadians(180)), Math.toRadians(-90))
//                .lineToLinearHeading(
//                    new Pose2d(48, 36, Math.toRadians(180))) // REPLACE w/ STRAFETOLINHEADING
                    .waitSeconds(3)
                    .lineTo(new Vector2d(18, 24))
                    .waitSeconds(1)
                    .splineTo(new Vector2d(60, 12), Math.toRadians(0))
                    .build()
        );

    meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start();
  }
}