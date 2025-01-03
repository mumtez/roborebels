package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.MeepMeep.Background;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepSpecimen {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        Pose2d bar = new Pose2d(new Vector2d(0, -35), Math.toRadians(270));
        Pose2d leftBlock = new Pose2d(new Vector2d(62, -5), Math.toRadians(270));
        Pose2d middleBlock = new Pose2d(new Vector2d(54, -5), Math.toRadians(270));
        Pose2d rightBlock = new Pose2d(new Vector2d(46, -5), Math.toRadians(270));

        Pose2d wallPickup = new Pose2d( new Vector2d(30, 60), Math.toRadians(270));


        Pose2d park1 = new Pose2d(new Vector2d(-48, -36), Math.toRadians(180));
        Vector2d park2 = new Vector2d(35, -36);
        Vector2d park3 = new Vector2d(48, -62);

        meepMeep.setBackground(Background.FIELD_INTO_THE_DEEP_OFFICIAL);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedDark())
                .build();

        bot.runAction(bot.getDrive().actionBuilder(new Pose2d(10.5, -62, Math.toRadians(270)))
                //Board

                //.strafeToSplineHeading(new Vector2d(-48, -62), Math.toRadians(180))

                        .waitSeconds(1.5)



                .splineToLinearHeading(bar, Math.toRadians(180))
                .waitSeconds(.2)

                .splineToSplineHeading(rightBlock, Math.toRadians(45))
                .waitSeconds(.2)

                .splineToLinearHeading(new Pose2d( new Vector2d(42, -55), Math.toRadians(270)),90 )

                .splineToLinearHeading(new Pose2d( new Vector2d(42, -5), Math.toRadians(270)),90)
                //.waitSeconds(.2)

                .splineToLinearHeading(middleBlock, Math.toRadians(160))
                .waitSeconds(.2)

                .splineToLinearHeading(new Pose2d( new Vector2d(54, -55), Math.toRadians(270)),90 )

                .splineToLinearHeading(new Pose2d( new Vector2d(54, -5), Math.toRadians(270)),90)
                .waitSeconds(.2)

                .splineToLinearHeading(leftBlock, Math.toRadians(180))
                .waitSeconds(.2)

                .strafeTo(new Vector2d(62, -55))
                .waitSeconds(.2)

                .splineToLinearHeading(wallPickup, 90)

                //.waitSeconds(.2)

                .build());

        meepMeep.setBackground(Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)

                .start();
    }
}