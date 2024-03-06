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
    MeepMeep meepMeep = new MeepMeep(600);

    RoadRunnerBotEntity right = new DefaultBotBuilder(meepMeep)
        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
        .setColorScheme(new ColorSchemeRedDark())
        .build();

    right.runAction(right.getDrive().actionBuilder(new Pose2d(10.5, -62, Math.toRadians(270)))
            //Board
        .strafeToSplineHeading(new Vector2d(48, -42), Math.toRadians(180))
        .waitSeconds(0.2)
            //Spike
        .strafeTo(new Vector2d(12, -29))
        .waitSeconds(0.2)

            //By truss
        .strafeTo(new Vector2d(10.5, -60))
        .waitSeconds(0.2)

            //through truss to corner
        .strafeTo(new Vector2d(-58, -60))
        .waitSeconds(.2)

            //stack
        .strafeTo(new Vector2d(-58, -34))
        .waitSeconds(.2)

            //back
        .strafeTo(new Vector2d(-58, -60))
        .waitSeconds(.2)

            //line up with board
        .strafeTo(new Vector2d(10.5, -60))
        .waitSeconds(.2)

            //board
        .strafeTo(new Vector2d(48, -42))
        .waitSeconds(.2)
            .build());

    RoadRunnerBotEntity center = new DefaultBotBuilder(meepMeep)
        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
        .setColorScheme(new ColorSchemeBlueLight())
        .build();

    center.runAction(center.getDrive().actionBuilder(new Pose2d(10.5, -62, Math.toRadians(270)))
            //Board
            .strafeToSplineHeading(new Vector2d(48, -36), Math.toRadians(180))
            .waitSeconds(0.2)
            //Spike
            .strafeTo(new Vector2d(18, -24))
            .waitSeconds(0.2)

            //By truss
            .strafeTo(new Vector2d(10.5, -60))
            .waitSeconds(0.2)

            //through truss to corner
            .strafeTo(new Vector2d(-58, -60))
            .waitSeconds(.2)

            //stack
            .strafeTo(new Vector2d(-58, -34))
            .waitSeconds(.2)

            //back
            .strafeTo(new Vector2d(-58, -60))
            .waitSeconds(.2)

            //line up with board
            .strafeTo(new Vector2d(10.5, -60))
            .waitSeconds(.2)

            //board
            .strafeTo(new Vector2d(48, -36))
            .waitSeconds(.2)
            .build());

    RoadRunnerBotEntity left = new DefaultBotBuilder(meepMeep)
        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
        .setColorScheme(new ColorSchemeBlueDark())
        .build();

    left.runAction(left.getDrive().actionBuilder(new Pose2d(10.5, -62, Math.toRadians(180)))
            //Board
            .strafeToSplineHeading(new Vector2d(48, -29), Math.toRadians(180))
            .waitSeconds(0.2)
            //Spike
            .strafeTo(new Vector2d(33, -29))
            .waitSeconds(0.2)

            //By truss
            .strafeTo(new Vector2d(10.5, -60))
            .waitSeconds(0.2)

            //through truss to corner
            .strafeTo(new Vector2d(-58, -60))
            .waitSeconds(.2)

            //stack
            .strafeTo(new Vector2d(-58, -34))
            .waitSeconds(.2)

            //back
            .strafeTo(new Vector2d(-58, -60))
            .waitSeconds(.2)

            //line up with board
            .strafeTo(new Vector2d(10.5, -60))
            .waitSeconds(.2)

            //board
            .strafeTo(new Vector2d(48, -42))
            .waitSeconds(.2)
            .build());

    meepMeep.setBackground(Background.FIELD_CENTERSTAGE_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        //.addEntity(right)
        .addEntity(left)
        //.addEntity(center)

        .start();
  }
}