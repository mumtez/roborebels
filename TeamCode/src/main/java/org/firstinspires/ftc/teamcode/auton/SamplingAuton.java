package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Robot.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides;

// TODO tune
@Config
@Autonomous(name = "SPECIMEN", group = "PEDRO")
public class SamplingAuton extends LinearOpMode {

  Robot robot;

  public static double HSLIDE_1 = Intake.SLIDE_OUT;
  public static double HSLIDE_2 = Intake.SLIDE_OUT;
  public static double HSLIDE_3 = Intake.SLIDE_OUT;


  // MAIN POINTS
  public static double[] START = {9, 105, Math.toRadians(270)};
  public static double[] PLACE_WALL = {25, 118, Math.toRadians(180)};
  public static double[] INTAKE_ONE = {28, 118, Math.toRadians(0)};
  public static double[] INTAKE_TWO = {30, 124, Math.toRadians(0)};
  public static double[] INTAKE_THREE = {38, 120, Math.toRadians(50)};
  public static double[] INTAKE_SUB = {60, 98, Math.toRadians(270)};

  //TODO: TUNE THESE
  public static double[] GRAB_WALL = {12, 118, Math.toRadians(180)};
  public static double[] PRE_BAR = {40, 70, Math.toRadians(0)};
  public static double[] PLACE_BAR = {45, 70, Math.toRadians(0)};
  public static double[] POST_BAR = {35, 60, Math.toRadians(0)};


  public static double[] INTAKE_SUB_SECONDARY = {63, 101, Math.toRadians(280)};
  public static double[] END = {60, 98, Math.toRadians(90)};

  // CONTROL POINTS
  public static double[] START_BUCKET_CONTROL = {26, 118};
  public static double[] BUCKET_INTAKE_SUB_CONTROL = {54, 126};

  // PATHS
  private Pose startPose, placeWallPose, grabWallPose, preBarPose, placeBarPose, postBarPose, intakeOnePose, intakeTwoPose, intakeThreePose, intakeSubPose,
          intakeSubSecondaryPose, endPose;
  private Point startBucketControl, bucketIntakeSubControl;

  PathChain placePreLoad,
          pickupOne, placeOne,
          pickupTwo, placeTwo,
          pickupThree, placeThree,
          pickupFour,
          pickupSubMovementOne, pickupSubMovementTwo,
          grabOne, grabTwo, grabThree,
          placeFour,
          end;

  private int pathState = 0;
  private Timer pathTimer;

  public Pose poseFromArr(double[] arr) {
    return new Pose(arr[0], arr[1], arr[2]);
  }

  public void buildPaths() {
    // POSE SETUP
    startPose = poseFromArr(START);
    placeWallPose = poseFromArr(PLACE_WALL);
    grabWallPose = poseFromArr(GRAB_WALL);
    preBarPose = poseFromArr(PRE_BAR);
    placeBarPose = poseFromArr(PLACE_BAR);
    postBarPose = poseFromArr(POST_BAR);
    intakeOnePose = poseFromArr(INTAKE_ONE);
    intakeTwoPose = poseFromArr(INTAKE_TWO);
    intakeThreePose = poseFromArr(INTAKE_THREE);
    intakeSubPose = poseFromArr(INTAKE_SUB);
    intakeSubSecondaryPose = poseFromArr(INTAKE_SUB_SECONDARY);
    endPose = poseFromArr(END);

    // CONTROL POINT SETUP
    startBucketControl = new Point(START_BUCKET_CONTROL[0], START_BUCKET_CONTROL[1]);
    bucketIntakeSubControl = new Point(BUCKET_INTAKE_SUB_CONTROL[0], BUCKET_INTAKE_SUB_CONTROL[1]);

    // PATH CHAIN SETUP

    placePreLoad = robot.follower.pathBuilder()
            .addBezierCurve(
                    new Point(startPose),
                    startBucketControl,
                    new Point(placeWallPose)
            )
            .setLinearHeadingInterpolation(startPose.getHeading(), placeWallPose.getHeading())
            .build();

    pickupOne = robot.follower.pathBuilder()
            .addBezierLine(
                    new Point(placeWallPose),
                    new Point(intakeOnePose)
            )
            .setLinearHeadingInterpolation(placeWallPose.getHeading(), intakeOnePose.getHeading())
            .build();

    placeOne = robot.follower.pathBuilder()
            .addBezierLine(
                    new Point(intakeOnePose),
                    new Point(placeWallPose)
            )
            .setLinearHeadingInterpolation(intakeOnePose.getHeading(), placeWallPose.getHeading())
            .build();

    pickupTwo = robot.follower.pathBuilder()
            .addBezierLine(
                    new Point(placeWallPose),
                    new Point(intakeTwoPose)
            )
            .setLinearHeadingInterpolation(placeWallPose.getHeading(), intakeTwoPose.getHeading())
            .build();

    placeTwo = robot.follower.pathBuilder()
            .addBezierLine(
                    new Point(intakeTwoPose),
                    new Point(placeWallPose)
            )
            .setLinearHeadingInterpolation(intakeTwoPose.getHeading(), placeWallPose.getHeading())
            .build();

    pickupThree = robot.follower.pathBuilder()
            .addBezierLine(
                    new Point(placeWallPose),
                    new Point(intakeThreePose)
            )
            .setLinearHeadingInterpolation(placeWallPose.getHeading(), intakeThreePose.getHeading())
            .build();

    placeThree = robot.follower.pathBuilder()
            .addBezierLine(
                    new Point(intakeThreePose),
                    new Point(placeWallPose)
            )
            .setLinearHeadingInterpolation(intakeThreePose.getHeading(), placeWallPose.getHeading())
            .build();

    pickupFour = robot.follower.pathBuilder()
            .addBezierCurve(
                    new Point(placeWallPose),
                    bucketIntakeSubControl,
                    new Point(intakeSubPose)
            )
            .setLinearHeadingInterpolation(placeWallPose.getHeading(), intakeSubPose.getHeading())
            .build();

    pickupSubMovementOne = robot.follower.pathBuilder()
            .addBezierLine(
                    new Point(intakeSubPose),
                    new Point(intakeSubSecondaryPose)
            )
            .setLinearHeadingInterpolation(intakeSubPose.getHeading(), intakeSubSecondaryPose.getHeading())
            .build();

    pickupSubMovementTwo = robot.follower.pathBuilder()
            .addBezierLine(
                    new Point(intakeSubSecondaryPose),
                    new Point(intakeSubPose)
            )
            .setLinearHeadingInterpolation(intakeSubSecondaryPose.getHeading(), intakeSubPose.getHeading())
            .build();

    placeFour = robot.follower.pathBuilder()
            .addBezierCurve(
                    new Point(intakeSubPose),
                    bucketIntakeSubControl,
                    new Point(placeWallPose)
            )
            .setLinearHeadingInterpolation(intakeSubPose.getHeading(), placeWallPose.getHeading())
            .build();

    end = robot.follower.pathBuilder()
            .addBezierCurve(
                    new Point(placeWallPose),
                    bucketIntakeSubControl,
                    new Point(endPose)
            )
            .setLinearHeadingInterpolation(placeWallPose.getHeading(), endPose.getHeading())
            .build();
  }

  public void setPathState(int pState) {
    pathState = pState;
    pathTimer.resetTimer();
  }

  public void autonomousPathUpdate() {
    switch (pathState) {

      // MOVE TO SCORE PRELOAD
      case 0:
        robot.follower.followPath(placePreLoad, true);
        robot.slides.setMode(RunMode.RUN_WITHOUT_ENCODER);
        robot.slides.setTarget(VerticalSlides.DEFAULT);
        setPathState(101);
        break;

      // SCORE PRELOAD
      case 101:
        if (robot.slides.atTarget(30)) {
          robot.slides.setTarget(VerticalSlides.UP);
          robot.claw.setBucket();
          setPathState(1);
        }
        break;

      case 1:
        if (!robot.follower.isBusy() && robot.slides.atTarget(30)) {
          place(pickupOne);
          setPathState(2);
        }
        break;

      // MOVE TO INTAKE 1
      case 2:
        robot.slides.setTarget(VerticalSlides.PRE_TRANSFER);
        if (robot.slides.atTarget(30) && pathTimer.getElapsedTimeSeconds() > 1) {
          robot.intake.update(1, false, HSLIDE_1, robot.getAllianceColor());
        }

        if (!robot.follower.isBusy() && robot.intake.validSampleIn(robot.getAllianceColor())) {
          robot.intake.update(0, true, Intake.SLIDE_TRANSFER, robot.getAllianceColor());
          robot.follower.followPath(placeOne, true);
          setPathState(4);
        }

        break;


      // PLACE 1
      case 4:
        if (!robot.follower.isBusy() && robot.slides.atTarget(30)) {
          robot.intake.update(-1, true, HSLIDE_1, robot.getAllianceColor());
          setPathState(5);
        }
        break;

      // INTAKE 2
      case 5:
        robot.slides.setTarget(VerticalSlides.PRE_TRANSFER);
        if (robot.slides.atTarget(30) && pathTimer.getElapsedTimeSeconds() > 1) {
          robot.intake.update(1, false, HSLIDE_2, robot.getAllianceColor());
        }
        if (!robot.follower.isBusy() && robot.intake.validSampleIn(robot.getAllianceColor())) {
          robot.intake.update(0, true, Intake.SLIDE_TRANSFER, robot.getAllianceColor());
          robot.follower.followPath(placeTwo, true);
          setPathState(7);
        }
        break;


      // PLACE 2
      case 7:
        if (!robot.follower.isBusy() && robot.slides.atTarget(30)) {
          place(pickupThree);
          robot.intake.update(-1, true, HSLIDE_2, robot.getAllianceColor());
          setPathState(8);
        }
        break;

      // INTAKE 3

      case 8:
        robot.slides.setTarget(VerticalSlides.PRE_TRANSFER);
        if (robot.slides.atTarget(30) && pathTimer.getElapsedTimeSeconds() > 1) {
          robot.intake.update(1, false, HSLIDE_3, robot.getAllianceColor());
        }
        if (!robot.follower.isBusy() && robot.intake.validSampleIn(robot.getAllianceColor())) {
          robot.intake.update(0, true, Intake.SLIDE_TRANSFER, robot.getAllianceColor());
          robot.follower.followPath(placeThree, true);
          setPathState(9);
        }
        break;


      // PLACE 3
      case 10:
        if (!robot.follower.isBusy() && robot.slides.atTarget(30)) {
          robot.intake.update(-1, true, HSLIDE_3, robot.getAllianceColor());
          setPathState(11);
        }
        break;


      case 11:
        if(!robot.follower.isBusy()){
          robot.claw.setWall();
          robot.claw.clawOpen();
         // robot.follower.followPath();

        }
        break;

      // LV1 ASCENT
      case 14:
        if (!robot.follower.isBusy()) {
          robot.claw.setPlace();
          setPathState(15);
        }
    }

  }

  private void place(PathChain nextPath) {
    robot.waitTime(500);
    robot.claw.clawOpen();
    robot.waitTime(500);

    robot.follower.followPath(nextPath, true);
    robot.slides.setTarget(VerticalSlides.DEFAULT);
    robot.claw.setTransfer();
  }


  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);

    pathTimer = new Timer();
    buildPaths();
    robot.initAuton();

    AllianceColor color = AllianceColor.RED;
    // INIT LOOP
    while (opModeInInit()) {
      if (gamepad1.square) {
        color = AllianceColor.RED;
      }
      if (gamepad1.cross) {
        color = AllianceColor.BLUE;
      }
      robot.setAllianceColor(color);

      telemetry.addLine("SQUARE = RED | CROSS = BLUE");
      telemetry.addData("ALLIANCE", robot.getAllianceColor());
      telemetry.update();
    }

    // START
    robot.follower.setStartingPose(startPose);

    while (opModeIsActive()) {
      robot.follower.update();
      robot.slides.updatePIDControl();
      autonomousPathUpdate();
      telemetry.addData("Path State", pathState);
      telemetry.addData("Position", robot.follower.getPose().toString());

      telemetry.update();
    }
  }
}
