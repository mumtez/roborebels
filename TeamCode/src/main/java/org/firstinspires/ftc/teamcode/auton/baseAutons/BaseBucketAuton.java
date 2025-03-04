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

  public static double[] START = {9, 105, 270};
  public static double[] PLACE_BUCKET = {17, 130, 315};
  public static double[] PLACE_BUCKET_TWO = {27, 122, 315};

  public static double[] INTAKE_ONE = {28, 118, 0};
  public static double[] INTAKE_TWO = {30, 123, 0};
  public static double[] INTAKE_THREE = {38, 120, 50};
  public static double[] INTAKE_SUB = {60, 98, 270};
  public static double[] INTAKE_SUB_SECONDARY = {63, 101, 280};
  public static double[] END = {60, 98, 90};

  public static double TURN_ONE = 35;
  public static double TURN_TWO = 45;
  public static double TURN_THREE = 90;

  // CONTROL POINTS
  public static double[] START_BUCKET_CONTROL = {35, 105};
  public static double[] BUCKET_INTAKE_SUB_CONTROL = {54, 126};

  PathChain placePreLoad,
      pickupOne, placeOne,
      pickupTwo, placeTwo,
      failedOne, failedTwo, failedThree,
      pickupThree, placeThree,
      pickupFour,
      pickupSubMovementOne, pickupSubMovementTwo,
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

  public Point pointFromArr(double[] arr) {
    return new Point(arr[0], arr[1]);
  }

  public Pose poseFromArr(double[] arr) {
    return new Pose(arr[0], arr[1], Math.toRadians(arr[2]));
  }

  public void buildPaths() {

    placePreLoad = robot.follower.pathBuilder()
        .addBezierCurve(
            pointFromArr(START),
            pointFromArr(START_BUCKET_CONTROL),
            pointFromArr(PLACE_BUCKET)
        )
        .setLinearHeadingInterpolation(Math.toRadians(START[2]), Math.toRadians(PLACE_BUCKET[2]))
        .build();

    pickupOne = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(PLACE_BUCKET),
            pointFromArr(INTAKE_ONE)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PLACE_BUCKET[2]),
            Math.toRadians(INTAKE_ONE[2]))
        .build();

    failedOne = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_ONE),
            pointFromArr(INTAKE_TWO)
        )
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_ONE[2]), Math.toRadians(INTAKE_TWO[2]))
        .build();

    placeOne = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_TWO),
            pointFromArr(PLACE_BUCKET)
        )
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_TWO[2]),
            Math.toRadians(PLACE_BUCKET[2]))
        .build();

    pickupTwo = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(PLACE_BUCKET),
            pointFromArr(INTAKE_TWO)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PLACE_BUCKET[2]),
            Math.toRadians(INTAKE_TWO[2]))
        .build();

    failedTwo = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_TWO),
            pointFromArr(INTAKE_THREE)
        )
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_TWO[2]),
            Math.toRadians(INTAKE_THREE[2]))
        .build();

    placeTwo = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_TWO),
            pointFromArr(PLACE_BUCKET_TWO)
        )
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_TWO[2]),
            Math.toRadians(PLACE_BUCKET_TWO[2]))
        .build();

    pickupThree = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(PLACE_BUCKET_TWO),
            pointFromArr(INTAKE_THREE)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PLACE_BUCKET_TWO[2]),
            Math.toRadians(INTAKE_THREE[2]))
        .build();

    failedThree = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_THREE),
            pointFromArr(INTAKE_SUB)
        )
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_THREE[2]),
            Math.toRadians(INTAKE_SUB[2]))
        .build();

    placeThree = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_THREE),
            pointFromArr(PLACE_BUCKET_TWO)
        )
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_THREE[2]),
            Math.toRadians(PLACE_BUCKET_TWO[2]))
        .build();

    // TODO: should pickup movements back and forth from the sub be TANGENTIAL heading for speed?
    pickupFour = robot.follower.pathBuilder()
        .addBezierCurve(
            pointFromArr(PLACE_BUCKET_TWO),
            pointFromArr(BUCKET_INTAKE_SUB_CONTROL),
            pointFromArr(INTAKE_SUB)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PLACE_BUCKET_TWO[2]),
            Math.toRadians(INTAKE_SUB[2]))
        .build();

    pickupSubMovementOne = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_SUB),
            pointFromArr(INTAKE_SUB_SECONDARY)
        )
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_SUB[2]),
            Math.toRadians(INTAKE_SUB_SECONDARY[2]))
        .build();

    pickupSubMovementTwo = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_SUB_SECONDARY),
            pointFromArr(INTAKE_SUB)
        )
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_SUB_SECONDARY[2]),
            Math.toRadians(INTAKE_SUB[2]))
        .build();

    // TODO: could maybe also be tangential?
    end = robot.follower.pathBuilder()
        .addBezierCurve(
            pointFromArr(PLACE_BUCKET),
            pointFromArr(BUCKET_INTAKE_SUB_CONTROL),
            pointFromArr(END)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PLACE_BUCKET[2]), Math.toRadians(END[2]))
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
        robot.horSlide.setMode(RunMode.RUN_WITHOUT_ENCODER);
        robot.slides.setTarget(VerticalSlides.UP);
        robot.horSlide.setTarget(HSLIDE_1 / 3); // first raise
        robot.claw.setBucket();
        setPathState(101);
        break;

      // SCORE PRELOAD
      case 101:
        if (!robot.follower.isTurning() && robot.slides.atTarget(100)) {
          placeTurn(TURN_ONE, true); // place preload turn to first pickup
          robot.intake.update(1, false, robot.getAllianceColor());
          robot.horSlide.setTarget(HSLIDE_1);// after turn put intake down spin and extend
          setPathState(2);
        }
        break;

      // Check if 1 intaken
      case 2:

        if (robot.intake.validSampleIn(robot.getAllianceColor())) {
          telemetry.addData("correct color in", 1);
          robot.intake.update(0, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
          robot.follower.turnToDegrees(315);
          setPathState(3);
        }

        if (pathTimer.getElapsedTimeSeconds() > INTAKE_OVERIDE) { // if it misses first pickup
          robot.intake.update(-1, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HSLIDE_1 / 2);
          robot.follower.turnDegrees(TURN_TWO - TURN_ONE, true);
          robot.intake.update(1, false, robot.getAllianceColor());
          robot.horSlide.setTarget(HSLIDE_1);
          setPathState(5);
        }
        break;

      // TRANSFER INTAKE 1
      case 3:

        if (robot.horSlide.atTarget() && !robot.follower.isTurning()) {
          robot.claw.clawClose();
          setPathState(32);
        }
        break;

      case 32:
        robot.slides.setTarget(VerticalSlides.UP + 100);
        robot.claw.setBucket();
        robot.intake.update(0, true, robot.getAllianceColor());
        robot.horSlide.setTarget(HSLIDE_2 / 2);
        setPathState(4);

        break;

      // SCORE 1
      case 4:
        if (!robot.follower.isBusy() && robot.slides.atTarget()) {
          robot.horSlide.setTarget(HSLIDE_2 / 2);
          placeTurn(TURN_TWO, true);
          setPathState(5);
        }
        break;

      // INTAKE 2
      case 5:
        if (!robot.follower.isBusy()) {
          robot.intake.update(1, false, robot.getAllianceColor());
          robot.horSlide.setTarget(HSLIDE_2);
        }

        if (robot.intake.validSampleIn(robot.getAllianceColor())) {
          robot.intake.update(0, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
          robot.follower.turn(TURN_TWO, false);
          setPathState(6);
        }

        if (pathTimer.getElapsedTimeSeconds() > INTAKE_OVERIDE) { // if it fails second
          robot.intake.update(-1, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HSLIDE_1 - 400);
          robot.follower.turnDegrees(TURN_THREE - TURN_TWO, true);
          robot.intake.update(1, false, robot.getAllianceColor());
          robot.horSlide.setTarget(HSLIDE_1);
          setPathState(8);

        }
        break;

      // TRANSFER 2
      case 6:

        if (robot.horSlide.atTarget()) {
          robot.claw.clawClose();
          setPathState(62);
        }
        break;

      case 62:
        if (robot.slides.atTarget(100)) {
          robot.slides.setTarget(VerticalSlides.UP + 100);
          robot.horSlide.setTarget(HSLIDE_3 / 2);
          robot.claw.setBucket();
          robot.intake.update(0, true, robot.getAllianceColor());
          setPathState(7);
        }
        break;

      // SCORE 2
      case 7:
        if (!robot.follower.isBusy() && robot.slides.atTarget()) {
          placeTurn(TURN_THREE, true);
          setPathState(8);
        }
        break;

      // INTAKE 3

      case 8:
        if (!robot.follower.isTurning()) {
          robot.intake.update(1, false, robot.getAllianceColor());
          robot.horSlide.setTarget(HSLIDE_3);
        }
        if (!robot.follower.isTurning() && robot.intake.validSampleIn(robot.getAllianceColor())) {
          robot.intake.update(0, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
          robot.follower.turn(TURN_THREE, false);
          setPathState(9);
        }

        if (pathTimer.getElapsedTimeSeconds() > INTAKE_OVERIDE) { // if it fails 3rd
          robot.intake.update(-1, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
          robot.follower.followPath(failedThree);
          setPathState(11);

        }
        break;

      // TRANSFER 3
      case 9:
        if (robot.horSlide.atTarget()) {
          robot.claw.clawClose();
          setPathState(92);
        }
        break;

      case 92:
        robot.slides.setTarget(VerticalSlides.UP + 100);
        robot.claw.setBucket();
        setPathState(10);
        break;

      // SCORE 3
      case 10:
        if (!robot.follower.isBusy() && robot.slides.atTarget()) {
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
          robot.horSlide.setTarget(HorizontalSlides.OUT_POS / 2);
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
                      pointFromArr(BUCKET_INTAKE_SUB_CONTROL),
                      pointFromArr(PLACE_BUCKET)
                  )
                  .setLinearHeadingInterpolation(current.getHeading(),
                      Math.toRadians(PLACE_BUCKET[2]))
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
                      pointFromArr(BUCKET_INTAKE_SUB_CONTROL),
                      pointFromArr(PLACE_BUCKET)
                  )
                  .setLinearHeadingInterpolation(current.getHeading(),
                      Math.toRadians(PLACE_BUCKET[2]))
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

        if (pathTimer.getElapsedTimeSeconds() > 1.01 && robot.horSlide.atTarget()) {
          robot.claw.clawClose();
          setPathState(122);
        }
        break;

      case 122:
        // TODO: if final move is too fast for slides to go up, should instead make it slightly slower bc raising
        //  slides after move takes more time than slowing the move and raising simul
        if (!robot.follower.isBusy() && robot.slides.atTarget(80)) {
          robot.slides.setTarget(VerticalSlides.UP);
          robot.claw.setBucket();
          setPathState(13);
        }
        break;

      // SCORE SUB
      case 13:
        if (!robot.follower.isBusy() && robot.slides.atTarget()) {
          place(end);
          robot.claw.setTransfer();
          setPathState(14);
        }
        break;

      // LV1 ASCENT
      case 14:
        if (!robot.follower.isBusy()) {
//          robot.claw.setPlace();
          setPathState(15);
        }
        break;
    }

  }

  // TODO: optimize wait times here
  private void place(PathChain nextPath) {
    robot.waitTime(500);
    robot.claw.clawOpen();
    robot.waitTime(500);

    robot.follower.followPath(nextPath, true);
    robot.slides.setTarget(VerticalSlides.TRANSFER);
    robot.claw.setTransfer();
  }

  // TODO: optimize wait times here
  private void placeTurn(double degrees, boolean left) {

    robot.claw.clawOpen();
    robot.waitTime(300);

    robot.follower.turn(Math.toRadians(degrees), left);
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
    robot.follower.setStartingPose(poseFromArr(START));
    globalTimer.resetTimer();

    while (this.opMode.opModeIsActive()) {
      robot.follower.update();
      robot.slides.updatePIDControl();
      robot.horSlide.updatePosition();
      robot.horSlide.updatePIDControl();

      telemetry.addData("Path State", pathState);
      telemetry.addData("Position", robot.follower.getPose().toString());
      telemetry.addData("horslide pos", robot.horSlide.position);
      telemetry.addData("horslide target", robot.horSlide.getTarget());

      telemetry.update();

      if (globalTimer.getElapsedTimeSeconds() > 29) {
        robot.claw.setTransfer();
      } else {
        autonomousPathUpdate();
      }
    }
  }


}


