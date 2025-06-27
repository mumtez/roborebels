package org.firstinspires.ftc.teamcode.auton.baseAutons;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NewRobot;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlides;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides;

@Config
public class BaseBucketAuton {

  public static int MOVE_ARM_HEIGHT_OFFSET = 800;
  public static int VALIDATE_SAMPLE_MS = 50;

  public static double HSLIDE_1 = HorizontalSlides.OUT_POS - 55;
  public static double HSLIDE_2 = HorizontalSlides.OUT_POS;
  public static double HSLIDE_3 = HorizontalSlides.OUT_POS;

  public static double HSLIDE_SUB = (HorizontalSlides.TRANSFER_POS + 10);

  public static double INTAKE_OVERRIDE = 2;

  // MAIN POINTS

  public static double[] START = {10, 114.5, 270};
  public static double[] PLACE_BUCKET = {15, 132, 342};

  public static double[] PLACE_BUCKET_ONE = {17, 135.5, 353};

  public static double[] PLACE_BUCKET_TWO = {17, 135.5, 340};

  public static double[] PLACE_BUCKET_THREE = {18, 134, 345};

  public static double[] PLACE_BUCKET_SUB = {20, 132, 315};

  public static double[] INTAKE_ONE = {20, 130, 342};
  public static double[] INTAKE_TWO = {19, 135.5, 353};
  public static double[] INTAKE_THREE = {23, 128, 40};
  public static double[] INTAKE_SUB_PRIME = {61, 94, 270};

  public static double[] INTAKE_SUB_LEFT = {68, 94, 270};

  public static double[] INTAKE_SUB_MIDDLE = {64, 94, 270};

  public static double[] BUCKET_INTAKE_SUB_CONTROL = {60.7, 118};
  public static double[] BUCKET_INTAKE_SUB_CONTROL_LEFT = {63.7, 118};

  public static double[] BUCKET_INTAKE_SUB_CONTROL_DOUBLE_LEFT = {67.7, 118};

  PathChain
      placePreLoad,
      intakeOne, placeOne,
      intakeTwo, placeTwo,
      intakeThree, placeThree,
      bucketToSub, subToSubLeft,
      subLeftToSubMiddle, subMiddleToSubRight;

  private int pathState = 0;

  private final Timer pathTimer = new Timer();
  private final ElapsedTime subTimer = new ElapsedTime();
  private final ElapsedTime intakeTimer = new ElapsedTime();
  private final ElapsedTime transferTimer = new ElapsedTime();

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

  public void setPathState(int pState) {
    pathState = pState;
    pathTimer.resetTimer();
  }

  public void buildPaths() {
    placePreLoad = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(START),
            pointFromArr(PLACE_BUCKET)
        )
        .setLinearHeadingInterpolation(Math.toRadians(START[2]), Math.toRadians(PLACE_BUCKET[2]))
        .setPathEndTimeoutConstraint(500) // TODO: tune lower?
        .addParametricCallback(.5, robot.claw::setBucket)
        .build();

    intakeOne = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(PLACE_BUCKET),
            pointFromArr(INTAKE_ONE)
        )
        .addParametricCallback(.5, () -> robot.intake.update(1, false, robot.getAllianceColor()))
        .setConstantHeadingInterpolation(Math.toRadians(PLACE_BUCKET[2]))
        .setPathEndTimeoutConstraint(350)
        .build();

    placeOne = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_ONE),
            pointFromArr(PLACE_BUCKET_ONE)
        )
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_ONE[2]), Math.toRadians(PLACE_BUCKET_ONE[2]))
        .build();

    intakeTwo = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(PLACE_BUCKET_ONE),
            pointFromArr(INTAKE_TWO)
        )
        .addParametricCallback(.5, () -> robot.intake.update(1, false, robot.getAllianceColor()))
        .setConstantHeadingInterpolation(Math.toRadians(PLACE_BUCKET_ONE[2]))
        .setPathEndTimeoutConstraint(250)
        .build();

    placeTwo = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_TWO),
            pointFromArr(PLACE_BUCKET_TWO)
        )
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_TWO[2]), Math.toRadians(PLACE_BUCKET_TWO[2]))
        .build();

    intakeThree = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(PLACE_BUCKET_TWO),
            pointFromArr(INTAKE_THREE)
        )
        .addParametricCallback(.5, () -> robot.intake.update(1, false, robot.getAllianceColor()))

        .setLinearHeadingInterpolation(Math.toRadians(PLACE_BUCKET[2]), Math.toRadians(INTAKE_THREE[2]))
        .build();

    placeThree = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_THREE),
            pointFromArr(PLACE_BUCKET_THREE)
        )
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_THREE[2]), Math.toRadians(PLACE_BUCKET_THREE[2]))
        .build();

    bucketToSub = robot.follower.pathBuilder()
        .addBezierCurve(
            pointFromArr(PLACE_BUCKET_THREE),
            pointFromArr(BUCKET_INTAKE_SUB_CONTROL),
            pointFromArr(INTAKE_SUB_PRIME)
        )
        .setTangentHeadingInterpolation()
        .addParametricCallback(.90, () -> robot.intake.sweepOut(true))
        .setPathEndTimeoutConstraint(0) // todo test
        .setZeroPowerAccelerationMultiplier(5)
        .build();

    subToSubLeft = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_SUB_PRIME),
            pointFromArr(INTAKE_SUB_LEFT)
        )
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_SUB_PRIME[2]), Math.toRadians(INTAKE_SUB_LEFT[2]))
        .addParametricCallback(.89, () -> robot.horSlide.setTarget(HorizontalSlides.OUT_POS))
        .addParametricCallback(.91, () -> robot.intake.sweepOut(false))
        .addParametricCallback(.91, () -> robot.intake.update(1, false, robot.getAllianceColor()))

        .setZeroPowerAccelerationMultiplier(5)
        .setPathEndTimeoutConstraint(50)
        .build();
    subLeftToSubMiddle = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_SUB_LEFT),
            pointFromArr(INTAKE_SUB_MIDDLE)
        )
        .setConstantHeadingInterpolation(Math.toRadians(INTAKE_SUB_LEFT[2]))
        .addParametricCallback(.9, () -> robot.intake.update(1, false, robot.getAllianceColor()))
        .addParametricCallback(.9, () -> robot.horSlide.setTarget(HorizontalSlides.OUT_POS))
        .setZeroPowerAccelerationMultiplier(5)
        .setPathEndTimeoutConstraint(0)
        .build();

    subMiddleToSubRight = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_SUB_MIDDLE),
            pointFromArr(INTAKE_SUB_PRIME)
        )
        .setConstantHeadingInterpolation(Math.toRadians(INTAKE_SUB_LEFT[2]))
        .addParametricCallback(.9, () -> robot.intake.update(1, false, robot.getAllianceColor()))
        .addParametricCallback(.9, () -> robot.horSlide.setTarget(HorizontalSlides.OUT_POS))
        .setZeroPowerAccelerationMultiplier(5)
        .setPathEndTimeoutConstraint(0)
        .build();
  }

  public void autonomousPathUpdate() {
    switch (pathState) {

      // MOVE TO SCORE PRELOAD
      case 0:
        robot.slides.setTarget(VerticalSlides.UP_AUTO);
        robot.horSlide.setTarget(HSLIDE_1);
        robot.follower.followPath(placePreLoad, true);

        setPathState(100);
        break;

      case 100:
        if (robot.slides.atTarget()) {
          setPathState(101);
        }
        break;

      // SCORE PRELOAD
      case 101:
        if (!robot.follower.isBusy() && robot.slides.atTarget()) {
          place(intakeOne, 0, 20);
          robot.horSlide.setTarget(HorizontalSlides.OUT_POS);

          setPathState(2);
        }
        break;

      // INTAKE 1
      case 2:
        robot.intake.update(1, false, robot.getAllianceColor());

        boolean validCollected1 = robot.intake.validSampleIn(robot.getAllianceColor());

        if (!robot.follower.isBusy() && validCollected1) {
          robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
          robot.intake.update(0.05, true, robot.getAllianceColor());
          robot.follower.followPath(placeOne, true);
          setPathState(3);
        }

        if (pathTimer.getElapsedTimeSeconds() > INTAKE_OVERRIDE) { // if it misses first pickup
          robot.intake.update(-1, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HSLIDE_2);
          robot.follower.followPath(placeOne, true);
          setPathState(201);
        }
        break;

      // FAILED FIRST PICKUP --> PICKUP SECOND
      case 201:
        if (!robot.follower.isBusy()) {
          robot.follower.followPath(intakeTwo, true);
          setPathState(5); //TODO: test
        }
        break;

      // TRANSFER INTAKE 1
      case 3:
        if (robot.horSlide.magLim.isPressed()) {
          setPathState(31);
        }
        break;

      case 31:
        robot.claw.clawClose();
        setPathState(32);
        break;

      case 32:
        if (pathTimer.getElapsedTime() > 40) { // changed from 70
          robot.slides.setTarget(VerticalSlides.UP_AUTO);
          robot.horSlide.setTarget(HSLIDE_2);
          setPathState(33);
        }
        break;

      case 33:
        if (robot.slides.atTarget(MOVE_ARM_HEIGHT_OFFSET)) {
          robot.claw.setBucket();
          setPathState(4);
        }
        break;

      // SCORE 1
      case 4:
        if (!robot.follower.isBusy() && robot.slides.atTarget()) {
          place(intakeTwo, 250, 50);
          setPathState(5);
        }
        break;

      // INTAKE 2
      case 5:
        robot.intake.update(1, false, robot.getAllianceColor());

        boolean validCollected2 = robot.intake.validSampleIn(robot.getAllianceColor());

        if (!robot.follower.isBusy() && !validCollected2) {
          robot.horSlide.setTarget(HorizontalSlides.OUT_POS);
        }

        if (!robot.follower.isBusy() && validCollected2) {
          robot.intake.update(0.15, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
          robot.follower.followPath(placeTwo, true);
          setPathState(6);
        }

        if (pathTimer.getElapsedTimeSeconds() > INTAKE_OVERRIDE) { // if it fails second
          robot.intake.update(-1, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HSLIDE_2);
          robot.follower.followPath(placeTwo, true);
          setPathState(52);
        }
        break;

      case 52:
        if (!robot.follower.isBusy()) {
          robot.follower.followPath(intakeThree, true);
          setPathState(8);
        }
        break;

      // TRANSFER 2
      case 6:
        if (robot.horSlide.magLim.isPressed()) {
          setPathState(61);
        }
        break;

      case 61:
        robot.claw.clawClose();
        setPathState(62);
        break;

      case 62:
        if (pathTimer.getElapsedTime() > 40) { // changed from 70
          robot.slides.setTarget(VerticalSlides.UP_AUTO);
          robot.horSlide.setTarget(HSLIDE_3);
          setPathState(63);
        }
        break;

      case 63:
        // TODO: possible optimization: move arm earlier (needs tuning)
        if (robot.slides.atTarget(MOVE_ARM_HEIGHT_OFFSET)) {
          robot.claw.setBucket();
          setPathState(7);
        }
        break;

      // SCORE 2
      case 7:
        if (!robot.follower.isBusy() && robot.slides.atTarget()) {
          place(intakeThree, 250, 50);
          setPathState(8);
        }
        break;

      // INTAKE 3
      case 8:
        robot.intake.update(1, false, robot.getAllianceColor());

        boolean validCollected3 = robot.intake.validSampleIn(robot.getAllianceColor());

        if (validCollected3) {
          robot.intake.update(0.35, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);

          robot.follower.followPath(robot.follower.pathBuilder()
              .addBezierLine(
                  pointFromArr(
                      new double[]{robot.follower.getClosestPose().getX(), robot.follower.getClosestPose().getY(),
                          robot.follower.getClosestPose().getHeading()}),
                  pointFromArr(PLACE_BUCKET_THREE)
              )
              .setLinearHeadingInterpolation(Math.toRadians(robot.follower.getClosestPose().getHeading()),
                  Math.toRadians(PLACE_BUCKET_THREE[2]))
              .build(), true);
          setPathState(9);
        }

        if (pathTimer.getElapsedTimeSeconds() > INTAKE_OVERRIDE) { // if it misses pickup
          robot.intake.update(-1, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
          robot.follower.followPath(placeThree, true);
          setPathState(81);
        }
        break;

      case 81:
        if (!robot.follower.isBusy()) {
          robot.intake.update(0, true, robot.getAllianceColor());
          robot.follower.followPath(bucketToSub);
          setPathState(11);
        }
        break;

      // TRANSFER 3
      case 9:
        if (robot.horSlide.magLim.isPressed()) {
          setPathState(91);
        }
        break;

      case 91:
        robot.claw.clawClose();
        setPathState(92);
        break;

      case 92:
        if (pathTimer.getElapsedTime() > 70) { // changed from 100
          robot.slides.setTarget(VerticalSlides.UP_AUTO);
          setPathState(93);
        }
        break;

      case 93:
        robot.horSlide.setTarget(HSLIDE_SUB);
        robot.intake.update(0, true, robot.getAllianceColor());
        if (robot.slides.atTarget(MOVE_ARM_HEIGHT_OFFSET)) {
          robot.claw.setBucket();
          setPathState(10);
        }
        break;

      case 10:
        if (!robot.follower.isBusy() && robot.slides.atTarget()) {
          place(bucketToSub, 250, 75);
          setPathState(11);
        }
        break;

      case 11:
        if (!robot.follower.isBusy()) {
          robot.intake.sweepOut(true);
          robot.follower.followPath(subToSubLeft);
          intakeTimer.reset();
          setPathState(12);
        }
        break;

      case 12:
        if (!robot.follower.isBusy()) {
          if (intakeTimer.milliseconds() < 1550) {
            robot.intake.update(1, false, robot.getAllianceColor());
            robot.horSlide.setTarget(HorizontalSlides.OUT_POS);
          }
          if (robot.intake.validSampleIn(robot.getAllianceColor())) {
            intakeTimer.reset();
            setPathState(13);
          } else if (!robot.intake.validSampleIn(robot.getAllianceColor())
              && intakeTimer.milliseconds() > 1550) {
            robot.intake.update(-.5, true, robot.getAllianceColor());
            robot.horSlide.setTarget(HSLIDE_SUB + 10);
            if (robot.horSlide.atTarget()) {
              robot.follower.followPath(subLeftToSubMiddle);
              setPathState(17);
            }
          }
        }
        break;

      case 13:
        robot.intake.update(0.15, true, robot.getAllianceColor());
        if (intakeTimer.milliseconds() > VALIDATE_SAMPLE_MS && !robot.intake.isSpitting()) {
          setPathState(14);
        } else if (intakeTimer.milliseconds() > VALIDATE_SAMPLE_MS) { // it wrong color
          robot.horSlide.setTarget(HSLIDE_SUB + 10);
          setPathState(135);
        }
        break;

      case 135:
        if (robot.horSlide.atTarget()) {
          robot.follower.followPath(subToSubLeft);
          setPathState(17);
        }
        break;

      case 14:
        subCycleToBucket(new double[]{INTAKE_SUB_LEFT[0], INTAKE_SUB_LEFT[1], INTAKE_SUB_LEFT[2]}, 15,
            BUCKET_INTAKE_SUB_CONTROL_DOUBLE_LEFT);
        break;

      case 15:
        if (robot.horSlide.magLim.isPressed()) {
          transferTimer.reset();
          setPathState(16);
        }
        break;

      case 16:
        subCycleTransfer(INTAKE_SUB_MIDDLE, 17, BUCKET_INTAKE_SUB_CONTROL_LEFT);
        break;

      case 17:
        if (!robot.follower.isBusy()) {
          intakeTimer.reset();
          setPathState(18);
        }
        break;

      case 18:
        if (intakeTimer.milliseconds() < 1550) {
          robot.intake.update(1, false, robot.getAllianceColor());
          robot.horSlide.setTarget(HorizontalSlides.OUT_POS);
        }
        if (robot.intake.validSampleIn(robot.getAllianceColor())) {
          intakeTimer.reset();
          setPathState(19);
        }
        if (!robot.intake.validSampleIn(robot.getAllianceColor()) && intakeTimer.milliseconds() > 1550) {
          robot.intake.update(-.5, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HSLIDE_SUB + 10);
          if (robot.horSlide.atTarget()) {
            robot.follower.followPath(subMiddleToSubRight);
            setPathState(23);
          }

        }
        break;

      case 19:
        robot.intake.update(0.15, true, robot.getAllianceColor());
        if (intakeTimer.milliseconds() > VALIDATE_SAMPLE_MS && !robot.intake.isSpitting()) {
          setPathState(20);
        } else if (intakeTimer.milliseconds() > VALIDATE_SAMPLE_MS) { // it wrong color
          robot.horSlide.setTarget(HSLIDE_SUB + 10);
          setPathState(195);
        }
        break;

      case 195:
        if (robot.horSlide.atTarget()) {
          robot.follower.followPath(subToSubLeft);
          setPathState(23);
        }
        break;

      case 20:
        subCycleToBucket(INTAKE_SUB_MIDDLE, 21, BUCKET_INTAKE_SUB_CONTROL_LEFT);
        break;

      case 21:
        if (robot.horSlide.magLim.isPressed()) {
          transferTimer.reset();
          setPathState(22);
        }
        break;

      case 22:
        subCycleTransfer(INTAKE_SUB_PRIME, 23, BUCKET_INTAKE_SUB_CONTROL);
        break;

      case 23:
        if (!robot.follower.isBusy()) {
          intakeTimer.reset();
          setPathState(24);
        }
        break;

      case 24:
        if (intakeTimer.milliseconds() < 1550) { // if we are less then sub timer and nothing in
          robot.intake.update(1, false, robot.getAllianceColor());
          robot.horSlide.setTarget(HorizontalSlides.OUT_POS);
        }
        if (robot.intake.validSampleIn(robot.getAllianceColor())) { // if we get sample within timeframe
          intakeTimer.reset();
          setPathState(25);
        }
        if (!robot.intake.validSampleIn(robot.getAllianceColor()) // if we went over time and no sample so failed
            && intakeTimer.milliseconds() > 1550) {
          robot.intake.update(-.5, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HSLIDE_SUB + 10);
          if (robot.horSlide.atTarget()) {
            robot.follower.followPath(subToSubLeft);
            setPathState(245);
          }
        }
        break;

      case 245:
        if (!robot.follower.isBusy()) {
          intakeTimer.reset();
          setPathState(12);
        }
        break;

      case 25:
        robot.intake.update(0.15, true, robot.getAllianceColor());
        if (intakeTimer.milliseconds() > VALIDATE_SAMPLE_MS && !robot.intake.isSpitting()) {
          setPathState(26);
        } else if (intakeTimer.milliseconds() > VALIDATE_SAMPLE_MS) { // it wrong color
          robot.horSlide.setTarget(HSLIDE_SUB + 10);
          setPathState(255);
        }

        break;
      case 255:
        if (robot.horSlide.atTarget()) {
          robot.follower.followPath(subToSubLeft);
          setPathState(12);
        }
        break;

      case 26:
        subCycleToBucket(INTAKE_SUB_PRIME, 27, BUCKET_INTAKE_SUB_CONTROL);
        break;

      case 27:
        if (robot.horSlide.magLim.isPressed()) {
          transferTimer.reset();
          setPathState(28);
        }
        break;

      case 28: // go back to start if all 3 done pray for luck
        if (transferTimer.milliseconds() > 150) {
          robot.claw.clawClose();
          if (transferTimer.milliseconds() > 175) {
            robot.slides.setTarget(VerticalSlides.UP_AUTO + 45);
            robot.horSlide.setTarget(HorizontalSlides.OUT_POS - 20);
            if (robot.slides.atTarget(MOVE_ARM_HEIGHT_OFFSET)) {
              robot.claw.setBucket();
              if (!robot.follower.isBusy()) { //test
                place(robot.follower.pathBuilder()
                    .addBezierLine(
                        pointFromArr(PLACE_BUCKET_SUB),
                        pointFromArr(PLACE_BUCKET_SUB)
                    )
                    .setConstantHeadingInterpolation(PLACE_BUCKET[2])
                    .addParametricCallback(.1, () -> robot.horSlide.setTarget(HSLIDE_SUB))
                    .addParametricCallback(.91, () -> robot.intake.update(1, false, robot.getAllianceColor()))
                    .addParametricCallback(.91, () -> robot.horSlide.setTarget(HorizontalSlides.OUT_POS))
                    .setZeroPowerAccelerationMultiplier(5)
                    .setPathEndTimeoutConstraint(50)
                    .build(), 65, 100);
                setPathState(29);
              }
            }
          }
        }
        break;
      case 29:

        break;
    }
  }

  private void subCycleToBucket(double[] subPos, int next, double[] control) {
    robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
    if (pathTimer.getElapsedTime() < 200) {
      robot.intake.update(-1, true, robot.getAllianceColor());
    } else if (pathTimer.getElapsedTime() > 200 && pathTimer.getElapsedTime() < 500) {
      robot.intake.update(1, true, robot.getAllianceColor());
    } else if (pathTimer.getElapsedTime() > 500) {
      robot.intake.update(0.15, true, robot.getAllianceColor());
      setPathState(next);
    }
    robot.follower.followPath(robot.follower.pathBuilder()
        .addBezierCurve(
            pointFromArr(subPos),
            pointFromArr(control),
            pointFromArr(PLACE_BUCKET_SUB)
        )
        .setLinearHeadingInterpolation(Math.toRadians(subPos[2]),
            Math.toRadians(PLACE_BUCKET_SUB[2]))
        .setZeroPowerAccelerationMultiplier(5)
        .setPathEndTimeoutConstraint(50)
        .build(), true);

  }

  private void subCycleTransfer(double[] postPos, int next, double[] control) {
    if (transferTimer.milliseconds() > 150) {
      robot.claw.clawClose();
      if (transferTimer.milliseconds() > 175) {
        robot.slides.setTarget(VerticalSlides.UP_AUTO + 100);
        robot.horSlide.setTarget(HorizontalSlides.OUT_POS - 20);
        if (robot.slides.atTarget(MOVE_ARM_HEIGHT_OFFSET)) {
          robot.claw.setBucket();
          if (!robot.follower.isBusy() && robot.slides.atTarget()) { //test
            place(robot.follower.pathBuilder()
                .addBezierCurve(
                    pointFromArr(PLACE_BUCKET_SUB),
                    pointFromArr(control),
                    pointFromArr(postPos)
                )
                .setTangentHeadingInterpolation()
                .addParametricCallback(.1, () -> robot.horSlide.setTarget(HSLIDE_SUB))
                .addParametricCallback(.91, () -> robot.intake.update(1, false, robot.getAllianceColor()))
                .addParametricCallback(.91, () -> robot.horSlide.setTarget(HorizontalSlides.OUT_POS))
                .setZeroPowerAccelerationMultiplier(5)
                .setPathEndTimeoutConstraint(50)
                .build(), 65, 40);
            setPathState(next);
          }
        }
      }
    }
  }

  private void place(PathChain nextPath, int initialDelay, int delay) {
    subTimer.reset();
    while (opMode.opModeIsActive() && subTimer.milliseconds() < initialDelay) {
      robot.updateAutoControls();
    }

    robot.claw.clawOpen();

    subTimer.reset();
    while (opMode.opModeIsActive() && subTimer.milliseconds() < delay) {
      robot.updateAutoControls();
    }

    robot.claw.setTransfer();
    robot.slides.setTarget(VerticalSlides.TRANSFER);
    robot.follower.followPath(nextPath, true);
  }

  public void run() {
    buildPaths();
    robot.initAuton();

    // INIT LOOP
    while (this.opMode.opModeInInit()) {
      telemetry.addData("ALLIANCE", robot.getAllianceColor());
      telemetry.update();
    }

    // START
    robot.follower.setStartingPose(poseFromArr(START));
    robot.slides.setMode(RunMode.RUN_WITHOUT_ENCODER);

    while (this.opMode.opModeIsActive()) {
      robot.updateAutoControls();
      autonomousPathUpdate();

      telemetry.addData("Path State", pathState);
      telemetry.addData("Horiz Pos", robot.horSlide.getCurrentPosition());
      telemetry.update();
    }
  }
}