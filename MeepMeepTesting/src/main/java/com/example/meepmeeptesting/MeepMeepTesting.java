package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.MeepMeep.Background;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

  public static void main(String[] args) {
    MeepMeep meepMeep = new MeepMeep(800);

    RoadRunnerBotEntity right = new DefaultBotBuilder(meepMeep)
        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
        .setColorScheme(new ColorSchemeRedDark())
        .build();

    right.runAction(right.getDrive().actionBuilder(new Pose2d(-38, 62, Math.toRadians(270)))
        .splineToSplineHeading(new Pose2d(-46, 24, Math.toRadians(90)), Math.toRadians(270))
        .waitSeconds(2)
        .setTangent(Math.toRadians(180))
        .splineToSplineHeading(new Pose2d(-56, 12, Math.toRadians(180)), Math.toRadians(180))
        .waitSeconds(2)
        .strafeTo(new Vector2d(12, 12))
        .setTangent(0)
        .splineToSplineHeading(new Pose2d(48, 29, Math.toRadians(180)), Math.toRadians(90))
        .build());

    RoadRunnerBotEntity center = new DefaultBotBuilder(meepMeep)
        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
        .setColorScheme(new ColorSchemeBlueLight())
        .build();

    center.runAction(center.getDrive().actionBuilder(new Pose2d(-38, 62, Math.toRadians(270)))
        .splineToSplineHeading(new Pose2d(-36, 12, Math.toRadians(90)), Math.toRadians(270))
        .waitSeconds(2)
        .setTangent(Math.toRadians(180))
        .splineToSplineHeading(new Pose2d(-56, 12, Math.toRadians(180)), Math.toRadians(180))
        .waitSeconds(2)
        .strafeTo(new Vector2d(12, 12))
        .setTangent(0)
        .splineToSplineHeading(new Pose2d(48, 36, Math.toRadians(180)), Math.toRadians(90))
        .build());

    RoadRunnerBotEntity left = new DefaultBotBuilder(meepMeep)
        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
        .setColorScheme(new ColorSchemeBlueDark())
        .build();

    left.runAction(left.getDrive().actionBuilder(new Pose2d(-50, 7, Math.toRadians(180)))
        .strafeTo(new Vector2d(12, -12))
        .setTangent(0)
        //.waitSeconds(0) // TODO: update per match for partner
        .splineToLinearHeading(new Pose2d(50, -56, Math.toRadians(180)),
            Math.toRadians(-90))
        .strafeTo(new Vector2d(52.5, -37.5))
        .build());

    meepMeep.setBackground(Background.FIELD_CENTERSTAGE_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(right)
        .addEntity(left)
        .addEntity(center)

        .start();
  }
}