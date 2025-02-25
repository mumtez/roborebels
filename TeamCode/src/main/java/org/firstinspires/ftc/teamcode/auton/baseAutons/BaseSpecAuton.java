/*

package org.firstinspires.ftc.teamcode.auton.baseAutons;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NewRobot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides;

// TODO tune
@Config
public class BaseSpecAuton {

  public static double OUTTAKE_TIME_SEC = 0.7;

  public static int HSLIDE_1 = Intake.SLIDE_OUT - 0;
  public static int HSLIDE_2 = Intake.SLIDE_OUT - 0;
  public static int HSLIDE_3 = Intake.SLIDE_OUT - 0;

  public static double PLACE_DELAY = 0.05;
  public static double PLACE_DELAY2 = 0.3;
  public static double GRAB_DELAY = 0.1;


  // MAIN POINTS
  public static double[] START = {9, 58, Math.toRadians(180)};
  public static double[] INTAKE_ONE = {28, 24, Math.toRadians(320)};
  public static double[] DROP_ONE = {24, 24, Math.toRadians(220)};
  public static double[] INTAKE_TWO = {23, 21, Math.toRadians(330)};
  public static double[] DROP_TWO = {23, 22, Math.toRadians(180)};
  public static double[] INTAKE_THREE = {31, 15, Math.toRadians(290)};
  public static double[] DROP_THREE = {24, 14, Math.toRadians(180)};

  public static double[] GRAB_WALL_PRE = {20, 34, Math.toRadians(180)};
  public static double[] GRAB_WALL = {11.5, 34, Math.toRadians(180)};
  public static double[] PRE_BAR = {31, 70, Math.toRadians(180)};
  public static double[] PLACE_BAR = {36, 70, Math.toRadians(180)};
  public static double[] POST_BAR = {30, 70, Math.toRadians(180)};
  public static double[] END = {12, 30, Math.toRadians(270)};

  // CONTROL POINTS
  public static double[] POST_BAR_CONTROL = {33, 67};

  // PATHS
  private Pose startPose,
      grabWallPrePose, grabWallPose,
      preBarPose, placeBarPose, postBarPose,
      intakeOnePose, dropOnePose,
      intakeTwoPose, dropTwoPose,
      intakeThreePose, dropThreePose,
      endPose;

  private Point postBarControl;

  PathChain pickupOne;
  PathChain dropOne;
  PathChain pickupTwo;
  PathChain dropTwo;
  PathChain pickupThree;
  PathChain dropThree;
  PathChain grabOne;
  PathChain grab;
  PathChain placeBar1;
  PathChain placeBar2;
  PathChain placeBar3;
  PathChain placeBar4;
  PathChain placeBar5;
  PathChain preloadPrePlaceBar;
  PathChain park;

  private int pathState = 0;
  private Timer pathTimer;

  final NewRobot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;


  public BaseSpecAuton(LinearOpMode opMode, NewRobot robot) {
    this.opMode = opMode;
    this.telemetry = opMode.telemetry;
    this.robot = robot;
  }

  public Pose poseFromArr(double[] arr) {
    return new Pose(arr[0], arr[1], arr[2]);
  }

  // TODO: remove anywhere that uses postBarPose and replace with bezier curve with control points for speedup
  public void buildPaths() {
    // POSE SETUP
    startPose = poseFromArr(START);
    grabWallPrePose = poseFromArr(GRAB_WALL_PRE);
    grabWallPose = poseFromArr(GRAB_WALL);
    preBarPose = poseFromArr(PRE_BAR);
    placeBarPose = poseFromArr(PLACE_BAR);
    postBarPose = poseFromArr(POST_BAR);
    intakeOnePose = poseFromArr(INTAKE_ONE);
    dropOnePose = poseFromArr(DROP_ONE);
    intakeTwoPose = poseFromArr(INTAKE_TWO);
    dropTwoPose = poseFromArr(DROP_TWO);
    intakeThreePose = poseFromArr(INTAKE_THREE);
    dropThreePose = poseFromArr(DROP_THREE);
    endPose = poseFromArr(END);

    // CONTROL POINT SETUP
    postBarControl = new Point(POST_BAR_CONTROL[0], POST_BAR_CONTROL[1]);

    // PATH CHAIN SETUP

    pickupOne = robot.follower.pathBuilder()
        .addBezierLine(
            new Point(placeBarPose),
            new Point(postBarPose)
        )
        .setConstantHeadingInterpolation(placeBarPose.getHeading())
        .addBezierCurve(
            new Point(postBarPose),
            new Point(18, 50),
            new Point(intakeOnePose)
        )
        .setLinearHeadingInterpolation(postBarPose.getHeading(), intakeOnePose.getHeading())
        .build();

    dropOne = robot.follower.pathBuilder()
        .addBezierLine(
            new Point(intakeOnePose),
            new Point(dropOnePose)
        )
        .setLinearHeadingInterpolation(intakeOnePose.getHeading(), dropOnePose.getHeading())
        .build();

    pickupTwo = robot.follower.pathBuilder()
        .addBezierLine(
            new Point(dropOnePose),
            new Point(intakeTwoPose)
        )
        .setLinearHeadingInterpolation(dropOnePose.getHeading(), intakeTwoPose.getHeading())
        .build();

    dropTwo = robot.follower.pathBuilder()
        .addBezierLine(
            new Point(intakeTwoPose),
            new Point(dropTwoPose)
        )
        .setLinearHeadingInterpolation(intakeTwoPose.getHeading(), dropTwoPose.getHeading())
        .build();

    pickupThree = robot.follower.pathBuilder()
        .addBezierLine(
            new Point(dropTwoPose),
            new Point(intakeThreePose)
        )
        .setLinearHeadingInterpolation(dropTwoPose.getHeading(), intakeThreePose.getHeading())
        .build();

    dropThree = robot.follower.pathBuilder()
        .addBezierLine(
            new Point(intakeThreePose),
            new Point(dropThreePose)
        )
        .setLinearHeadingInterpolation(intakeThreePose.getHeading(), dropThreePose.getHeading())
        .build();

    grabOne = robot.follower.pathBuilder()
        .addBezierLine(
            new Point(dropThreePose),
            new Point(grabWallPrePose)
        )
        .setConstantHeadingInterpolation(dropThreePose.getHeading())
        .addBezierLine(
            new Point(grabWallPrePose),
            new Point(grabWallPose)
        )
        .setConstantHeadingInterpolation(grabWallPose.getHeading())
        .build();

    grab = robot.follower.pathBuilder()
        .addBezierCurve(
            new Point(placeBarPose),
            new Point(24, placeBarPose.getY()),
            new Point(35, grabWallPose.getY()),
            new Point(grabWallPose)
        )
        .setConstantHeadingInterpolation(grabWallPose.getHeading())
        .build();

    preloadPrePlaceBar = robot.follower.pathBuilder()
        .addBezierLine(
            new Point(startPose),
            new Point(preBarPose)
        )
        .setConstantHeadingInterpolation(startPose.getHeading())
        .build();

    placeBar1 = robot.follower.pathBuilder()
        .addBezierCurve(
            new Point(startPose),
            new Point(10, 70),
            new Point(placeBarPose)
        )
        .setConstantHeadingInterpolation(preBarPose.getHeading())
        .build();

    placeBar2 = robot.follower.pathBuilder()
        .addBezierCurve(
            new Point(grabWallPose),
            new Point(15, grabWallPose.getY()),
            new Point(15, placeBarPose.getY() + 1.5),
            new Point(placeBarPose.getX(), placeBarPose.getY() + 1.5)
        )
        .setConstantHeadingInterpolation(placeBarPose.getHeading())
        .build();

    placeBar3 = robot.follower.pathBuilder()
        .addBezierCurve(
            new Point(grabWallPose),
            new Point(15, grabWallPose.getY()),
            new Point(15, placeBarPose.getY() + 3),
            new Point(placeBarPose.getX(), placeBarPose.getY() + 3)
        )
        .setConstantHeadingInterpolation(placeBarPose.getHeading())
        .build();

    placeBar4 = robot.follower.pathBuilder()
        .addBezierCurve(
            new Point(grabWallPose),
            new Point(15, grabWallPose.getY()),
            new Point(15, placeBarPose.getY() + 4.5),
            new Point(placeBarPose.getX(), placeBarPose.getY() + 4.5)
        )
        .setConstantHeadingInterpolation(placeBarPose.getHeading())
        .build();

    placeBar5 = robot.follower.pathBuilder()
        .addBezierCurve(
            new Point(grabWallPose),
            new Point(24, grabWallPose.getY()),
            new Point(24, placeBarPose.getY() + 6),
            new Point(placeBarPose.getX(), placeBarPose.getY() + 6)
        )
        .setConstantHeadingInterpolation(placeBarPose.getHeading())
        .build();

    park = robot.follower.pathBuilder()
        .addBezierCurve(
            new Point(placeBarPose),
            postBarControl,
            new Point(postBarPose)
        )
        .setConstantHeadingInterpolation(placeBarPose.getHeading())
        .addBezierCurve(
            new Point(postBarPose),
            new Point(24, 36),
            new Point(endPose)
        )
        .setLinearHeadingInterpolation(postBarPose.getHeading(), endPose.getHeading())
        .build();

  }

  public void setPathState(int pState) {
    pathState = pState;
    pathTimer.resetTimer();
  }

  public void autonomousPathUpdate() {
    switch (pathState) {
      case 0:
        robot.slides.setTarget(VerticalSlides.SPECIMEN);
        robot.slides.setMode(RunMode.RUN_WITHOUT_ENCODER);
        robot.claw.setUnder();
        robot.claw.clawClose();
        robot.follower.followPath(placeBar1, true);
        setPathState(102);
        break;

      case 102:
        if (!robot.follower.isBusy()) {
          robot.claw.setPlace();
          if (pathTimer.getElapsedTimeSeconds() > PLACE_DELAY) {
            robot.follower.followPath(pickupOne, true);
            setPathState(103);
          }
        }
        break;

      case 103:
        if (pathTimer.getElapsedTimeSeconds() > PLACE_DELAY2) {
          robot.claw.clawOpenWall();
          setPathState(2);
        }
        break;

      // MOVE TO INTAKE 1
      case 2:
        if (!robot.follower.isBusy()) {
          robot.intake.update(1, false, HSLIDE_1, robot.getAllianceColor());
        }

        if (!robot.follower.isBusy() && robot.intake.validSampleIn(robot.getAllianceColor())) {
          robot.intake.update(0, true, Intake.SLIDE_TRANSFER, robot.getAllianceColor());
          setPathState(3);
        }
        break;

      case 3:
        if (pathTimer.getElapsedTimeSeconds() > 0.4) {
          robot.follower.followPath(dropOne);
          setPathState(4);
        }
        break;

      // PLACE 1
      case 4:
        if (!robot.follower.isBusy()) {
          robot.intake.update(-1, true, HSLIDE_1, robot.getAllianceColor());
          setPathState(41);
        }
        break;

      case 41:
        if (pathTimer.getElapsedTimeSeconds() > OUTTAKE_TIME_SEC) {
          robot.intake.update(0, true, Intake.SLIDE_TRANSFER, robot.getAllianceColor());
          setPathState(42);
        }
        break;

      case 42:
        if (pathTimer.getElapsedTimeSeconds() > 0.4) {
          robot.follower.followPath(pickupTwo, true);
          setPathState(5);
        }
        break;

      // INTAKE 2
      case 5:
        if (!robot.follower.isBusy()) {
          robot.intake.update(1, false, HSLIDE_2, robot.getAllianceColor());
        }
        if (!robot.follower.isBusy() && robot.intake.validSampleIn(robot.getAllianceColor())) {
          robot.intake.update(0, true, Intake.SLIDE_TRANSFER, robot.getAllianceColor());
          robot.follower.followPath(dropTwo, true);
          setPathState(7);
        }
        break;

      // PLACE 2
      case 7:
        if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.75) {
          robot.intake.update(-1, true, HSLIDE_2, robot.getAllianceColor());
          setPathState(71);
        }
        break;

      case 71:
        if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds() > OUTTAKE_TIME_SEC) {
          robot.intake.update(0, true, Intake.SLIDE_TRANSFER, robot.getAllianceColor());
          setPathState(72);
        }
        break;

      case 72:
        if (pathTimer.getElapsedTimeSeconds() > 0.4) {
          robot.follower.followPath(pickupThree);
          setPathState(8);
        }
        break;

      // INTAKE 3
      case 8:
        if (!robot.follower.isBusy()) {
          robot.intake.update(1, false, HSLIDE_3, robot.getAllianceColor());
        }
        if (!robot.follower.isBusy() && robot.intake.validSampleIn(robot.getAllianceColor())) {
          robot.intake.update(0, true, Intake.SLIDE_TRANSFER, robot.getAllianceColor());
          setPathState(81);
        }
        break;

      case 81:
        if (pathTimer.getElapsedTimeSeconds() > 0.4) {
          robot.follower.followPath(dropThree);
          setPathState(10);
        }
        break;

      // PLACE 3
      case 10:
        if (!robot.follower.isBusy()) {
          robot.intake.update(-1, true, HSLIDE_3, robot.getAllianceColor());
          setPathState(1001);
        }
        break;

      case 1001:
        if (pathTimer.getElapsedTimeSeconds() > OUTTAKE_TIME_SEC) {
          robot.intake.update(0, true, Intake.SLIDE_TRANSFER, robot.getAllianceColor());
          robot.follower.followPath(grabOne, true);
          robot.claw.setWall();
          robot.claw.clawOpenWall();
          setPathState(11);
        }
        break;

      //GRAB ONE
      case 11:
        if (!robot.follower.isBusy()) {
          robot.claw.clawClose();
          setPathState(1101);
        }
        break;

      case 1101:
        if (pathTimer.getElapsedTimeSeconds() > GRAB_DELAY) {
          robot.claw.setUnder();
          robot.follower.followPath(placeBar2, true);
          setPathState(14);
        }
        break;

      //RELEASE CLAW
      case 14:
        if (!robot.follower.isBusy()) {
          robot.claw.setPlace();
          robot.follower.followPath(grab, true);
          setPathState(1401);
        }
        break;

      case 1401:
        if (pathTimer.getElapsedTimeSeconds() > PLACE_DELAY2) {
          robot.claw.clawOpenWall();
          robot.claw.setWall();
          setPathState(16);
        }
        break;

      //GRAB TWO
      case 16:
        if (!robot.follower.isBusy()) {
          robot.claw.clawClose();
          setPathState(1601);
        }
        break;

      case 1601:
        if (pathTimer.getElapsedTimeSeconds() > GRAB_DELAY) {
          robot.claw.setUnder();
          robot.follower.followPath(placeBar3, true);
          setPathState(18);
        }
        break;

      //RELEASE CLAW
      case 18:
        if (!robot.follower.isBusy()) {
          robot.claw.setPlace();
          robot.follower.followPath(grab, true);
          setPathState(1801);
        }
        break;

      case 1801:
        if (pathTimer.getElapsedTimeSeconds() > PLACE_DELAY2) {
          robot.claw.clawOpenWall();
          robot.claw.setWall();
          setPathState(20);
        }
        break;

      case 20:
        if (!robot.follower.isBusy()) {
          robot.claw.clawClose();
          setPathState(2001);
        }
        break;

      case 2001:
        if (pathTimer.getElapsedTimeSeconds() > GRAB_DELAY) {
          robot.claw.setUnder();
          robot.follower.followPath(placeBar4, true);
          setPathState(22);
        }
        break;

      case 22:
        if (!robot.follower.isBusy()) {
          robot.claw.setPlace();
          robot.follower.followPath(grab, true);
          setPathState(2201);
        }
        break;

      case 2201:
        if (pathTimer.getElapsedTimeSeconds() > PLACE_DELAY2) {
          robot.claw.clawOpenWall();
          robot.claw.setWall();
          setPathState(24);
        }
        break;

      case 24:
        if (!robot.follower.isBusy()) {
          robot.claw.clawClose();
          setPathState(2401);
        }
        break;

      case 2401:
        if (pathTimer.getElapsedTimeSeconds() > GRAB_DELAY) {
          robot.follower.followPath(placeBar5, true);
          robot.claw.setUnder();
          setPathState(25);
        }
        break;

      case 25:
        if (!robot.follower.isBusy()) {
          robot.claw.setPlace();
          robot.intake.update(0, true, Intake.SLIDE_OUT, robot.getAllianceColor());
          robot.follower.followPath(park, false);
          setPathState(99);
        }
        break;
    }
  }

  public void run() {
    pathTimer = new Timer();
    buildPaths();
    robot.initAuton();

    // INIT LOOP
    while (this.opMode.opModeInInit()) {
      telemetry.addData("ALLIANCE", robot.getAllianceColor());
      telemetry.update();
    }

    // START
    robot.follower.setStartingPose(startPose);

    while (this.opMode.opModeIsActive()) {
      robot.follower.update();
      robot.slides.updatePIDControl();
      autonomousPathUpdate();

      telemetry.addData("Path State", pathState);
      telemetry.addData("Position", robot.follower.getPose().toString());
      telemetry.update();
    }
  }
}

 */
