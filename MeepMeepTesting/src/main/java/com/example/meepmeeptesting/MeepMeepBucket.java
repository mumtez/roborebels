package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.MeepMeep.Background;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepBucket {

  public static void main(String[] args) {
    MeepMeep meepMeep = new MeepMeep(600);

    Pose2d bucket = new Pose2d(new Vector2d(-54, -54), Math.toRadians(225));
    Vector2d leftBlock = new Vector2d(-48, -23.5);
    Pose2d middleBlock = new Pose2d(new Vector2d(-56, -48), Math.toRadians(270));
    Pose2d rightBlock = new Pose2d(new Vector2d(-48, -48), Math.toRadians(270));

    Pose2d park1 = new Pose2d(new Vector2d(-48, -36), Math.toRadians(180));
    Vector2d park2 = new Vector2d(35, -36);
    Vector2d park3 = new Vector2d(48, -62);

    meepMeep.setBackground(Background.FIELD_INTO_THE_DEEP_OFFICIAL);

    RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
        .setColorScheme(new ColorSchemeRedDark())
        .build();

    bot.runAction(bot.getDrive().actionBuilder(new Pose2d(-10.5, -62, Math.toRadians(90)))
        //Board

        //.strafeToSplineHeading(new Vector2d(-48, -62), Math.toRadians(180))

        .splineToSplineHeading(bucket, Math.toRadians(260))
        .waitSeconds(.2)

        .splineToLinearHeading(rightBlock, Math.toRadians(100))
        .waitSeconds(.2)

        .splineToSplineHeading(bucket, Math.toRadians(200))
        .waitSeconds(.2)

        .splineToLinearHeading(middleBlock, Math.toRadians(160))
        .waitSeconds(.2)

        .splineToSplineHeading(bucket, Math.toRadians(280))
        .waitSeconds(.2)

        .strafeToLinearHeading(leftBlock, Math.toRadians(180))
        .waitSeconds(.2)

        .splineToSplineHeading(bucket, Math.toRadians(260))
        .waitSeconds(.2)

        .splineToSplineHeading(park1, Math.toRadians(80))
        .waitSeconds(.2)

        .strafeToLinearHeading(park2, Math.toRadians(180))
        .waitSeconds(.2)

        .strafeToLinearHeading(park3, Math.toRadians(180))
        .waitSeconds(.2)

        //.waitSeconds(.2)

        .build());

    meepMeep.setBackground(Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(bot)

        .start();
  }
}