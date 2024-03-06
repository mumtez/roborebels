package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.MeepMeep.Background;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBackdropRed {

  // POSITIONS

  public static Pose2d BACKDROP_START = new Pose2d(10.5, -61, Math.toRadians(90));

  public static double BOARD_X = 48;
  public static Vector2d BOARD_LEFT = new Vector2d(BOARD_X, -29);
  public static Vector2d BOARD_CENTER = new Vector2d(BOARD_X, -36);
  public static Vector2d BOARD_RIGHT = new Vector2d(BOARD_X, -43);

  public static Vector2d SPIKE_LEFT = new Vector2d(10, -35);
  public static Vector2d SPIKE_CENTER = new Vector2d(18, -24);
  public static Vector2d SPIKE_RIGHT = new Vector2d(32, -30);

  public static Vector2d PARK_CORNER = new Vector2d(60, -60);
  public static Vector2d PARK_CENTER = new Vector2d(60, -10);

  public static Vector2d AUDIENCE_TRUSS = new Vector2d(-30, -60);
  public static Vector2d BACKDROP_TRUSS = new Vector2d(10.5, -60);
  public static Vector2d STACK = new Vector2d(-60, -35);

  public static VelConstraint BASE_VEL_CONSTRAINTS = (pose2dDual, posePath, v) -> {
    if (pose2dDual.position.x.value() > 45 || pose2dDual.position.x.value() < -48) {
      return 40.0;
    } else {
      return 120.0;
    }
  }; // TODO: doesnt work in meep meep but will work in real bot

  // CONFIGURATION

  public enum PARK_POSITION {
    CENTER, CORNER
  }

  public static PARK_POSITION parkPosition = PARK_POSITION.CENTER; // TODO: change to test all possibilities

  public enum PROP_POSITION {
    LEFT, CENTER, RIGHT, NONE;
  }

  public static PROP_POSITION propPosition = PROP_POSITION.RIGHT; // TODO: change to test all 3 possibilities

  public static double PLACE_TIME = 1.0;
  public static double PICKUP_TIME = 2.0;

  public static void main(String[] args) {
    MeepMeep meepMeep = new MeepMeep(800);

    RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        .setConstraints(100, 100, Math.PI * 3.0 / 2.0, Math.PI * 3.0 / 2.0,
            15) // TODO: estimated time is not realistic as we need to slowdown for some movements
        .setColorScheme(new ColorSchemeRedDark())
        .build();

    Vector2d yellowPlacement;
    Vector2d spikePlacement;
    Vector2d whitePlacement;
    switch (propPosition) {
      case LEFT:
        yellowPlacement = BOARD_LEFT;
        spikePlacement = SPIKE_LEFT;
        whitePlacement = BOARD_RIGHT;
        break;
      case CENTER:
        yellowPlacement = BOARD_CENTER;
        spikePlacement = SPIKE_CENTER;
        whitePlacement = BOARD_RIGHT;
        break;
      default: // RIGHT and no detection
        yellowPlacement = BOARD_RIGHT;
        spikePlacement = SPIKE_RIGHT;
        whitePlacement = BOARD_CENTER;
        break;
    }

    Vector2d parkVec;
    switch (parkPosition) {
      case CENTER:
        parkVec = PARK_CENTER;
        break;
      default: // CORNER
        parkVec = PARK_CORNER;
        break;
    }

    Action trajectory = bot.getDrive().actionBuilder(BACKDROP_START)
        // Board
        .strafeToLinearHeading(yellowPlacement, Math.toRadians(180))
        .waitSeconds(PLACE_TIME)

        // Spike
        .strafeToConstantHeading(spikePlacement)
        .waitSeconds(0.2)

        // By truss
        .setReversed(true)
        .splineToConstantHeading(BACKDROP_TRUSS, Math.toRadians(180))
        .setReversed(false)

        //through truss to corner
        .strafeToConstantHeading(AUDIENCE_TRUSS)

        //stack
        .splineToConstantHeading(STACK, Math.toRadians(90))
        .waitSeconds(PICKUP_TIME)

        //back
        .setReversed(true)
        .setTangent(Math.toRadians(270))
        .splineToConstantHeading(AUDIENCE_TRUSS, Math.toRadians(0))

        // Cross truss
        .strafeToConstantHeading(BACKDROP_TRUSS)

        //board
        .setReversed(true)
        .splineToConstantHeading(whitePlacement, Math.toRadians(0))
        .waitSeconds(PLACE_TIME)
        .setReversed(false)

        // park
        .splineToConstantHeading(parkVec, Math.toRadians(0))
        .build();

    bot.runAction(trajectory);
    meepMeep.setBackground(Background.FIELD_CENTERSTAGE_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(bot)
        .start();
  }
}