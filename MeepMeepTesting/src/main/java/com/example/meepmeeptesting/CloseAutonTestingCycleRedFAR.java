package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class CloseAutonTestingCycleRedFAR {

  public static void main(String[] args) {
    MeepMeep meepMeep = new MeepMeep(800);

    RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
        // CENTER CLOSE
        .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
        .followTrajectorySequence(drive ->
                drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(-90)))
                        .lineTo(new Vector2d(-36,-34))
                        .setTangent(-90)
                        .splineToLinearHeading(new Pose2d(-55, -36, Math.toRadians(180)), Math.toRadians(-180))
                        .setTangent(-0)
                        .splineToLinearHeading(new Pose2d(-36, -61, Math.toRadians(180)), Math.toRadians(-0))
                        .lineTo(new Vector2d(36,-60))
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(50,-36, Math.toRadians(180)), Math.toRadians(-0))
                        .setTangent(90)
                        .splineToSplineHeading(new Pose2d(60,-12, Math.toRadians(0)), Math.toRadians(-0))

                    .build()
        );

    meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start();
  }
}