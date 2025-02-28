
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
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlides;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides;

@Config
public class BaseBucketAuton {

  public static int HSLIDE_1 = HorizontalSlides.OUT_POS;
  public static int HSLIDE_2 = HorizontalSlides.OUT_POS;
  public static int HSLIDE_3 = HorizontalSlides.OUT_POS;

  public static double SUB_TIMER = 0.1;
  public static double INTAKE_OVERIDE = 4;

  public static double TRANSFER_DELAY = 0.8;
  public static double TRANSFER_DELAY_2 = TRANSFER_DELAY + 0.4;

  // MAIN POINTS
  public static double[] START = {9, 105, Math.toRadians(270)};
  public static double[] PLACE_BUCKET = {23.5, 121, Math.toRadians(315)};
  public static double[] PLACE_BUCKET_TWO = {27, 122, Math.toRadians(315)};

  public static double[] INTAKE_ONE = {28, 118, Math.toRadians(0)};
  public static double[] INTAKE_TWO = {30, 123, Math.toRadians(0)};
  public static double[] INTAKE_THREE = {38, 120, Math.toRadians(50)};
  public static double[] INTAKE_SUB = {60, 98, Math.toRadians(270)};
  public static double[] INTAKE_SUB_SECONDARY = {63, 101, Math.toRadians(280)};
  public static double[] END = {60, 98, Math.toRadians(90)};

  // CONTROL POINTS
  public static double[] START_BUCKET_CONTROL = {35, 105};
  public static double[] BUCKET_INTAKE_SUB_CONTROL = {54, 126};

  private Pose startPose, placeBucketPose, placeBucketPoseTwo, intakeOnePose, intakeTwoPose, intakeThreePose, intakeSubPose,
      intakeSubSecondaryPose, endPose;
  private Point startBucketControl, bucketIntakeSubControl;

  PathChain placePreLoad,
      pickupOne, placeOne,
      pickupTwo, placeTwo,
      failedOne, failedTwo, failedThree,
      pickupThree, placeThree,
      pickupFour,
      pickupSubMovementOne, pickupSubMovementTwo,
      placeFour,
      end;

  private int pathState = 0;
  private Timer pathTimer;
  private Timer globalTimer;


  final NewRobot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;

  public BaseBucketAuton(LinearOpMode opMode, NewRobot robot) {
    this.opMode = opMode;
    this.telemetry = opMode.telemetry;
    this.robot = robot;
  }

  public Pose poseFromArr(double[] arr) {
    return new Pose(arr[0], arr[1], arr[2]);
  }

  public void buildPaths() {
    // POSE SETUP
    startPose = poseFromArr(START);
    placeBucketPose = poseFromArr(PLACE_BUCKET);
    placeBucketPoseTwo = poseFromArr(PLACE_BUCKET_TWO);

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
            new Point(placeBucketPose)
        )
        .setLinearHeadingInterpolation(startPose.getHeading(), placeBucketPose.getHeading())
        .build();

    pickupOne = robot.follower.pathBuilder()
        .addBezierLine(
            new Point(placeBucketPose),
            new Point(intakeOnePose)
        )
        .setLinearHeadingInterpolation(placeBucketPose.getHeading(), intakeOnePose.getHeading())
        .build();

    failedOne = robot.follower.pathBuilder()
        .addBezierLine(
            new Point(intakeOnePose),
            new Point(intakeTwoPose)
        )
        .setLinearHeadingInterpolation(intakeOnePose.getHeading(), intakeTwoPose.getHeading())
        .build();

    placeOne = robot.follower.pathBuilder()
        .addBezierLine(
            new Point(intakeOnePose),
            new Point(placeBucketPose)
        )
        .setLinearHeadingInterpolation(intakeOnePose.getHeading(), placeBucketPose.getHeading())
        .build();

    pickupTwo = robot.follower.pathBuilder()
        .addBezierLine(
            new Point(placeBucketPose),
            new Point(intakeTwoPose)
        )
        .setLinearHeadingInterpolation(placeBucketPose.getHeading(), intakeTwoPose.getHeading())
        .build();

    failedTwo = robot.follower.pathBuilder()
        .addBezierLine(
            new Point(intakeTwoPose),
            new Point(intakeThreePose)
        )
        .setLinearHeadingInterpolation(intakeTwoPose.getHeading(), intakeThreePose.getHeading())
        .build();

    placeTwo = robot.follower.pathBuilder()
        .addBezierLine(
            new Point(intakeTwoPose),
            new Point(placeBucketPoseTwo)
        )
        .setLinearHeadingInterpolation(intakeTwoPose.getHeading(), placeBucketPose.getHeading())
        .build();

    pickupThree = robot.follower.pathBuilder()
        .addBezierLine(
            new Point(placeBucketPoseTwo),
            new Point(intakeThreePose)
        )
        .setLinearHeadingInterpolation(placeBucketPose.getHeading(), intakeThreePose.getHeading())
        .build();

    failedThree = robot.follower.pathBuilder()
        .addBezierLine(
            new Point(intakeThreePose),
            new Point(intakeSubPose)
        )
        .setLinearHeadingInterpolation(intakeThreePose.getHeading(), intakeSubPose.getHeading())
        .build();

    placeThree = robot.follower.pathBuilder()
        .addBezierLine(
            new Point(intakeThreePose),
            new Point(placeBucketPoseTwo)
        )
        .setLinearHeadingInterpolation(intakeThreePose.getHeading(), placeBucketPose.getHeading())
        .build();

    pickupFour = robot.follower.pathBuilder()
        .addBezierCurve(
            new Point(placeBucketPoseTwo),
            bucketIntakeSubControl,
            new Point(intakeSubPose)
        )
        .setLinearHeadingInterpolation(placeBucketPose.getHeading(), intakeSubPose.getHeading())
        .build();

    pickupSubMovementOne = robot.follower.pathBuilder()
        .addBezierLine(
            new Point(intakeSubPose),
            new Point(intakeSubSecondaryPose)
        )
        .setLinearHeadingInterpolation(intakeSubPose.getHeading(),
            intakeSubSecondaryPose.getHeading())
        .build();

    pickupSubMovementTwo = robot.follower.pathBuilder()
        .addBezierLine(
            new Point(intakeSubSecondaryPose),
            new Point(intakeSubPose)
        )
        .setLinearHeadingInterpolation(intakeSubSecondaryPose.getHeading(),
            intakeSubPose.getHeading())
        .build();

    placeFour = robot.follower.pathBuilder()
        .addBezierCurve(
            new Point(intakeSubPose),
            bucketIntakeSubControl,
            new Point(placeBucketPose)
        )
        .setLinearHeadingInterpolation(intakeSubPose.getHeading(), placeBucketPose.getHeading())
        .build();

    end = robot.follower.pathBuilder()
        .addBezierCurve(
            new Point(placeBucketPose),
            bucketIntakeSubControl,
            new Point(endPose)
        )
        .setLinearHeadingInterpolation(placeBucketPose.getHeading(), endPose.getHeading())
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
        robot.follower.followPath(placePreLoad, 1, true);  //TODO: may need reduced power
        robot.slides.setMode(RunMode.RUN_WITHOUT_ENCODER);
        robot.slides.setTarget(VerticalSlides.TRANSFER);
        setPathState(101);
        break;

      // SCORE PRELOAD
      case 101:
        if (robot.slides.atTarget(30)) {
          robot.slides.setTarget(VerticalSlides.UP + 100);
          robot.claw.setBucket();
          robot.intake.update(0, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HSLIDE_1);
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
        if (!robot.follower.isBusy()) {
          robot.intake.update(1, false, robot.getAllianceColor());
          robot.horSlide.setTarget(HSLIDE_1);
        }

        if (robot.intake.validSampleIn(robot.getAllianceColor())) {
          robot.intake.update(0, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
          robot.follower.followPath(placeOne, true);
          setPathState(3);
        }

        if (pathTimer.getElapsedTimeSeconds() > INTAKE_OVERIDE) {
          robot.intake.update(-1, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HSLIDE_1);
          robot.follower.followPath(failedOne);
          setPathState(5);
        }
        break;

      // TRANSFER INTAKE 1
      case 3:
        if (pathTimer.getElapsedTimeSeconds() > TRANSFER_DELAY) {
          robot.slides.setTarget(VerticalSlides.TRANSFER);
        }
        if (pathTimer.getElapsedTimeSeconds() > TRANSFER_DELAY_2 && robot.slides.atTarget(30)) {
          robot.claw.clawClose();
          setPathState(31);
        }
        break;

      case 31:
        if (pathTimer.getElapsedTimeSeconds() > 0.5) {
          robot.slides.setTarget(VerticalSlides.TRANSFER);
          setPathState(32);
        }
        break;

      case 32:
        if (robot.slides.atTarget(30)) {
          robot.slides.setTarget(VerticalSlides.UP + 100);
          robot.claw.setBucket();
          robot.intake.update(0, true, robot.getAllianceColor());
            robot.horSlide.setTarget(HSLIDE_2 / 2);
          setPathState(4);
        }
        break;

      // SCORE 1
      case 4:
        if (!robot.follower.isBusy() && robot.slides.atTarget(30)) {
          place(pickupTwo);
          setPathState(5);
        }
        break;

      // INTAKE 2
      case 5:
        if (!robot.follower.isBusy()) {
          robot.intake.update(1, false, robot.getAllianceColor());
          robot.horSlide.setTarget(HSLIDE_2);
        }

        if (!robot.follower.isBusy() && robot.intake.validSampleIn(robot.getAllianceColor())) {
          robot.intake.update(0, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
          robot.follower.followPath(placeTwo, true);
          setPathState(6);
        }

        if (pathTimer.getElapsedTimeSeconds() > INTAKE_OVERIDE) {
          robot.intake.update(-1, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HSLIDE_2);
          robot.follower.followPath(failedTwo);
          setPathState(8);

        }
        break;

      // TRANSFER 2
      case 6:
        if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds() > TRANSFER_DELAY) {
          robot.slides.setTarget(VerticalSlides.TRANSFER);
        }
        if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds() > TRANSFER_DELAY_2
            && robot.slides.atTarget(
            30)) {
          robot.claw.clawClose();
          setPathState(61);
        }
        break;

      case 61:
        if (pathTimer.getElapsedTimeSeconds() > 0.5) {
          robot.slides.setTarget(VerticalSlides.TRANSFER);
          setPathState(62);
        }
        break;

      case 62:
        if (robot.slides.atTarget(30)) {
          robot.slides.setTarget(VerticalSlides.UP + 100);
          robot.claw.setBucket();
          robot.intake.update(0, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HSLIDE_3 / 2);
          setPathState(7);
        }
        break;

      // SCORE 2
      case 7:
        if (!robot.follower.isBusy() && robot.slides.atTarget(30)) {
          place(pickupThree);
          setPathState(8);
        }
        break;

      // INTAKE 3

      case 8:
        if (!robot.follower.isBusy()) {
          robot.intake.update(1, false, robot.getAllianceColor());
          robot.horSlide.setTarget(HSLIDE_3);
        }
        if (!robot.follower.isBusy() && robot.intake.validSampleIn(robot.getAllianceColor())) {
          robot.intake.update(0, true, robot.getAllianceColor());
            robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
          robot.follower.followPath(placeThree, true);
          setPathState(9);
        }

        if (pathTimer.getElapsedTimeSeconds() > INTAKE_OVERIDE) {
          robot.intake.update(-1, true, robot.getAllianceColor());
            robot.horSlide.setTarget(HSLIDE_3);
          robot.follower.followPath(failedThree);
          setPathState(11);

        }
        break;

      // TRANSFER 3
      case 9:
        if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds() > TRANSFER_DELAY) {
          robot.slides.setTarget(VerticalSlides.TRANSFER);
        }
        if (!robot.follower.isBusy() && pathTimer.getElapsedTimeSeconds() > TRANSFER_DELAY_2
            && robot.slides.atTarget(
            30)) {
          robot.claw.clawClose();
          setPathState(91);
        }
        break;

      case 91:
        if (pathTimer.getElapsedTimeSeconds() > 0.5) {
          robot.slides.setTarget(VerticalSlides.TRANSFER);
          setPathState(92);
        }
        break;

      case 92:
        if (robot.slides.atTarget(30)) {
          robot.slides.setTarget(VerticalSlides.UP + 100);
          robot.claw.setBucket();
          setPathState(10);
        }
        break;

      // SCORE 3
      case 10:
        if (!robot.follower.isBusy() && robot.slides.atTarget(30)) {
          place(pickupFour);
          setPathState(11);
        }
        break;

      // INTAKE SUBMERSIBLE
      case 11:
        robot.intake.update(0, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
        if (!robot.follower.isBusy()) {
          robot.intake.update(1, true, robot.getAllianceColor());
            robot.horSlide.setTarget(HorizontalSlides.OUT_POS);
          setPathState(111);
        }
        break;

      case 111:
        if (pathTimer.getElapsedTimeSeconds() > SUB_TIMER) {
          robot.intake.update(1, false, robot.getAllianceColor());
            robot.horSlide.setTarget(HorizontalSlides.OUT_POS);
        }
        if (robot.intake.validSampleIn(robot.getAllianceColor())) {
          robot.intake.update(-1, true, robot.getAllianceColor());
            robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
          Pose current = robot.follower.getPose();
          robot.follower.followPath(
              robot.follower.pathBuilder()
                  .addBezierCurve(
                      new Point(current),
                      bucketIntakeSubControl,
                      new Point(placeBucketPose)
                  )
                  .setLinearHeadingInterpolation(current.getHeading(), placeBucketPose.getHeading())
                  .build());
          setPathState(12);
        }
        if (!robot.follower.isBusy()) {
          robot.follower.followPath(pickupSubMovementOne, true);
          setPathState(112);
        }
        break;

      case 112:
        if (pathTimer.getElapsedTimeSeconds() > SUB_TIMER) {
          robot.intake.update(1, false, robot.getAllianceColor());
            robot.horSlide.setTarget(HorizontalSlides.OUT_POS);
        }
        if (robot.intake.validSampleIn(robot.getAllianceColor())) {
          robot.intake.update(-1, true, robot.getAllianceColor());
            robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
          Pose current = robot.follower.getPose();
          robot.follower.followPath(
              robot.follower.pathBuilder()
                  .addBezierCurve(
                      new Point(current),
                      bucketIntakeSubControl,
                      new Point(placeBucketPose)
                  )
                  .setLinearHeadingInterpolation(current.getHeading(), placeBucketPose.getHeading())
                  .build());
          setPathState(12);
        }
        if (!robot.follower.isBusy()) {
          robot.follower.followPath(pickupSubMovementTwo, true);
          setPathState(111);
        }
        break;

      // TRANSFER SUBMERSIBLE

      case 12:
        if (pathTimer.getElapsedTimeSeconds() > 0.1) {
          if (robot.intake.validSampleIn(robot.getAllianceColor())) {
            robot.intake.update(0, true, robot.getAllianceColor());
              robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
          } else {
            robot.intake.update(1, true, robot.getAllianceColor());
              robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
          }
        }

        if (pathTimer.getElapsedTimeSeconds() > 1) {
          robot.slides.setTarget(VerticalSlides.TRANSFER);
        }
        if (pathTimer.getElapsedTimeSeconds() > 1.01 && robot.slides.atTarget(30)) {
          robot.claw.clawClose();
          setPathState(121);
        }
        break;

      case 121:
        if (pathTimer.getElapsedTimeSeconds() > 0.5) {
          robot.slides.setTarget(VerticalSlides.TRANSFER);
          setPathState(122);
        }
        break;

      case 122:
        // TODO: if final move is too fast for slides to go up, should instead make it slightly slower bc raising
        //  slides after move takes more time than slowing the move and raising simul
        if (!robot.follower.isBusy() && robot.slides.atTarget(30)) {
          robot.slides.setTarget(VerticalSlides.UP);
          robot.claw.setBucket();
          setPathState(13);
        }
        break;

      // SCORE SUB
      case 13:
        if (!robot.follower.isBusy() && robot.slides.atTarget(30)) {
          place(end);
          robot.claw.setWall();
          setPathState(14);
        }
        break;

      // LV1 ASCENT
      case 14:
        if (!robot.follower.isBusy()) {
//          robot.claw.setPlace();
          setPathState(15);
        }
    }

  }

  private void place(PathChain nextPath) {
    robot.waitTime(500);
    robot.claw.clawOpen();
    robot.waitTime(500);

    robot.follower.followPath(nextPath, true);
    robot.slides.setTarget(VerticalSlides.TRANSFER);
    robot.claw.setTransfer();
  }

  public void run() {
    pathTimer = new Timer();
    globalTimer = new Timer();
    buildPaths();
    robot.initAuton();

    // INIT LOOP
    while (this.opMode.opModeInInit()) {
      globalTimer.resetTimer();
      telemetry.addData("ALLIANCE", robot.getAllianceColor());
      telemetry.update();
    }

    // START
    robot.follower.setStartingPose(startPose);
    globalTimer.resetTimer();

    while (this.opMode.opModeIsActive()) {
      robot.follower.update();
      robot.slides.updatePIDControl();
      robot.horSlide.updatePIDControl(); //TODO: hopefully in the right spot
      //autonomousPathUpdate();

      telemetry.addData("Path State", pathState);
      telemetry.addData("Position", robot.follower.getPose().toString());
      telemetry.update();

      if (globalTimer.getElapsedTimeSeconds() > 29) {
        robot.claw.setWall();
      } else {
        autonomousPathUpdate();
      }
    }
  }


}


