package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class CloseAutonTestingCycleBlueFAR {

  public static void main(String[] args) {
    MeepMeep meepMeep = new MeepMeep(800);

    RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
        // CENTER CLOSE
        .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
        .followTrajectorySequence(drive ->
            drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(-40, 60))
                .splineToLinearHeading(new Pose2d(-50, 23, Math.toRadians(90)), Math.toRadians(-90))
                .lineToConstantHeading(new Vector2d(-48, 12))
                .turn(Math.toRadians(90))
                .lineToConstantHeading(new Vector2d(-63, 12))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(50, 27), Math.toRadians(45))
                .build()
        );

    meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start();
  }
}