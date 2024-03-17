package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.MeepMeep.Background;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting2 {

  public static void main(String[] args) {
    MeepMeep meepMeep = new MeepMeep(600);

    Vector2d start = new Vector2d(-37, -61);

    Vector2d leftSpike = new Vector2d(-46, -38);
    double leftSpikeHeading = Math.toRadians(180);
    Vector2d rightSpike = new Vector2d(-30, -35);
    double rightSpikeHeading = Math.toRadians(0);
    Vector2d centerSpike = new Vector2d(-38, -33);
    double centerSpikeHeading = Math.toRadians(90);

    Vector2d stack = new Vector2d(-60, -12);

    Vector2d audienceGate = new Vector2d(-12, -6);
    Vector2d backdropGate = new Vector2d(12, -6);

    Vector2d leftBackdrop = new Vector2d(49, -30);
    Vector2d centerBackdrop = new Vector2d(49, -37);
    Vector2d rightBackdrop = new Vector2d(49, -44);

    Spot spot = Spot.RIGHT;

    RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
        .setColorScheme(new ColorSchemeRedDark())
        .build();

    switch (spot) {
      case RIGHT:
        bot.runAction(bot.getDrive().actionBuilder(new Pose2d(start.x, start.y, Math.toRadians(90)))
            .splineTo(rightSpike, rightSpikeHeading)
            .strafeToConstantHeading(stack)
            .turnTo(Math.toRadians(180))

            .setReversed(true)
            .splineToConstantHeading(audienceGate, Math.toRadians(0))
            .splineToConstantHeading(backdropGate, Math.toRadians(0))
            .splineToConstantHeading(rightBackdrop, Math.toRadians(270))
            .build());
        break;

      case LEFT:
        bot.runAction(bot.getDrive().actionBuilder(new Pose2d(start.x, start.y, Math.toRadians(90)))
            .splineTo(leftSpike, leftSpikeHeading)
            .setTangent(0)
            .splineToConstantHeading(new Vector2d(leftSpike.x + 10, stack.y), Math.toRadians(180))
            .strafeToConstantHeading(stack)

            .setReversed(true)
            .splineToConstantHeading(audienceGate, Math.toRadians(0))
            .splineToConstantHeading(backdropGate, Math.toRadians(0))
            .splineToConstantHeading(leftBackdrop, Math.toRadians(270))
            .build());
        break;

      case CENTER:
        bot.runAction(bot.getDrive().actionBuilder(new Pose2d(start.x, start.y, Math.toRadians(90)))
            .splineToConstantHeading(centerSpike, centerSpikeHeading)
            .strafeToConstantHeading(new Vector2d(start.x, start.y + 5))
            .strafeToLinearHeading(stack, Math.toRadians(180))

            .setReversed(true)
            .splineToConstantHeading(audienceGate, Math.toRadians(0))
            .splineToConstantHeading(backdropGate, Math.toRadians(0))
            .splineToConstantHeading(centerBackdrop, Math.toRadians(270))

            .build());
        break;
    }

    meepMeep.setBackground(Background.FIELD_CENTERSTAGE_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(bot)
        .start();
  }

  enum Spot {
    LEFT, CENTER, RIGHT
  }
}