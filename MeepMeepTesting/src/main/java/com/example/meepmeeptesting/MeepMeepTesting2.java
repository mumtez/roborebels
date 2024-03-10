package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.MeepMeepBackdropRed.BOARD_RIGHT;
import static com.example.meepmeeptesting.MeepMeepBackdropRed.SPIKE_RIGHT;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.MeepMeep.Background;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting2 {

  public static void main(String[] args) {
    MeepMeep meepMeep = new MeepMeep(600);

    RoadRunnerBotEntity right = new DefaultBotBuilder(meepMeep)
        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
        .setColorScheme(new ColorSchemeRedDark())
        .build();

    right.runAction(right.getDrive().actionBuilder(new Pose2d(13, -61, Math.toRadians(90)))
        .splineTo(SPIKE_RIGHT, Math.toRadians(90))
        // Board
        .setReversed(true)
        .splineTo(BOARD_RIGHT,
            Math.toRadians(180))
        .build());

    meepMeep.setBackground(Background.FIELD_CENTERSTAGE_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        //.addEntity(right)
        .addEntity(right)
        //.addEntity(center)

        .start();
  }
}