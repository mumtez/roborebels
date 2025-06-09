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

  public static int HSLIDE_1 = (int) (HorizontalSlides.OUT_POS * .8);
  public static int HSLIDE_2 = (HorizontalSlides.OUT_POS);
  public static int HSLIDE_3 = (int) (HorizontalSlides.OUT_POS * .8);

  public static int HSLIDE_SUB = (int) (HorizontalSlides.OUT_POS * .2);

  public static double INTAKE_OVERRIDE = 4;

  // MAIN POINTS

  public static double[] START = {9.5, 113.5, 270};
  public static double[] PLACE_BUCKET = {16, 131, 345};

  public static double[] PLACE_BUCKET_ONE = {17, 135.5, 355};

  public static double[] PLACE_BUCKET_TWO = {17, 135.5, 355};

  public static double[] PLACE_BUCKET_THREE = {18, 134, 345};


  public static double[] INTAKE_ONE = {20, 130, 345};
  public static double[] INTAKE_TWO = {19, 135, 355};
  public static double[] INTAKE_THREE = {24.5, 128.5, 40};
  public static double[] INTAKE_SUB_PRIME = {64, 97, 270};

  public static double[] BUCKET_INTAKE_SUB_CONTROL = {64, 128};


  PathChain placePreLoad,
      intakeOne, placeOne,
      intakeTwo, placeTwo,
      intakeThree, placeThree,
      bucketToSub, subToBucket;

  private int pathState = 0;

  private final Timer pathTimer = new Timer();
  private final ElapsedTime subTimer = new ElapsedTime();
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
        .setLinearHeadingInterpolation(Math.toRadians(PLACE_BUCKET[2]), Math.toRadians(INTAKE_ONE[2]))
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
        .setLinearHeadingInterpolation(Math.toRadians(PLACE_BUCKET_ONE[2]), Math.toRadians(INTAKE_TWO[2]))
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
        .addParametricCallback(.8, () -> robot.horSlide.setTarget(HorizontalSlides.OUT_POS + 10))
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
        .addParametricCallback(.5, () -> robot.horSlide.setTarget(HSLIDE_SUB))
        .setZeroPowerAccelerationMultiplier(5)
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
          setPathState(2);
        }
        break;

      // INTAKE 1
      case 2:
        robot.intake.update(1, false, robot.getAllianceColor());

        boolean validCollected1 = robot.intake.validSampleIn(robot.getAllianceColor());

        if (!robot.follower.isBusy() && !validCollected1) {
          robot.horSlide.setTarget(HorizontalSlides.OUT_POS + 15);// after turn put intake down spin and extend
        }

        if (!robot.follower.isBusy() && validCollected1) {
          robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS - 5);
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
        if (pathTimer.getElapsedTime() > 20) {
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
          place(intakeTwo, 50, 50);
          setPathState(5);
        }
        break;

      // INTAKE 2
      case 5:
        robot.intake.update(1, false, robot.getAllianceColor());

        boolean validCollected2 = robot.intake.validSampleIn(robot.getAllianceColor());

        if (!robot.follower.isBusy() && !validCollected2) {
          robot.horSlide.setTarget(HorizontalSlides.OUT_POS + 25);
        }

        if (!robot.follower.isBusy() && validCollected2) {
          robot.intake.update(0.05, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS - 10);
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
        if (pathTimer.getElapsedTime() > 30) {
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
          place(intakeThree, 150, 115);
          setPathState(8);
        }
        break;

      // INTAKE 3

      case 8:
        robot.intake.update(1, false, robot.getAllianceColor());

        boolean validCollected3 = robot.intake.validSampleIn(robot.getAllianceColor());

        /*if (!robot.follower.isBusy() && !validCollected3) {
          robot.horSlide.setTarget(HorizontalSlides.OUT_POS + 15); // after turn put intake down spin and extend
        }*/

        if (!robot.follower.isBusy() && validCollected3) {
          robot.intake.update(0.05, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS - 20);

          robot.follower.followPath(placeThree, true);
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
        if (pathTimer.getElapsedTime() > 100) {
          robot.slides.setTarget(VerticalSlides.UP_AUTO);
          setPathState(93);
        }
        break;

      case 93:
        if (robot.slides.atTarget(MOVE_ARM_HEIGHT_OFFSET)) {
          robot.claw.setBucket();
          setPathState(10);
        }
        break;

      case 10:
        if (!robot.follower.isBusy() && robot.slides.atTarget()) {
          robot.horSlide.setTarget(HSLIDE_SUB);
          place(bucketToSub, 180, 100);
          setPathState(11);
        }
        break;
      case 11:
        if (!robot.follower.isBusy()) {
          robot.intake.sweepOut(true);
          if (pathTimer.getElapsedTime() > 2700) {
            robot.intake.sweepOut(false);
            setPathState(12);
          }
        }
        break;
      case 12:
        robot.intake.update(1, false, robot.getAllianceColor());
        robot.horSlide.setTarget(HorizontalSlides.OUT_POS);
        if (robot.intake.validSampleIn(robot.getAllianceColor())) {
          setPathState(13);
        }
        break;
      case 13:
        if (pathTimer.getElapsedTime() < 200) {
          robot.intake.update(-1, true, robot.getAllianceColor());
        } else if (pathTimer.getElapsedTime() > 200 && pathTimer.getElapsedTime() < 400) {
          robot.intake.update(1, true, robot.getAllianceColor());
        } else if (pathTimer.getElapsedTime() > .4) {
          robot.intake.update(0.05, true, robot.getAllianceColor());
          setPathState(14);
        }

        break;
      case 14:
        subCycleToBucket(new double[]{INTAKE_SUB_PRIME[0], INTAKE_SUB_PRIME[1], INTAKE_SUB_PRIME[2]}, 15);
        break;
      case 15:
        if (robot.horSlide.magLim.isPressed()) {
          transferTimer.reset();
          setPathState(16);
        }
        break;
      case 16:
        subCycleTransfer(new double[]{INTAKE_SUB_PRIME[0] + 3, INTAKE_SUB_PRIME[1], INTAKE_SUB_PRIME[2]}, 17);
        break;
      case 17:
        if (!robot.follower.isBusy()) {
          robot.intake.sweepOut(true);
          if (pathTimer.getElapsedTime() > 2700) {
            robot.intake.sweepOut(false);
            setPathState(12);
          }
        }
        break;
      case 18:
        robot.intake.update(1, false, robot.getAllianceColor());
        robot.horSlide.setTarget(HorizontalSlides.OUT_POS);
        if (robot.intake.validSampleIn(robot.getAllianceColor())) {
          setPathState(13);
        }
        break;
      case 19:
        if (pathTimer.getElapsedTime() < 200) {
          robot.intake.update(-1, true, robot.getAllianceColor());
        } else if (pathTimer.getElapsedTime() > 200 && pathTimer.getElapsedTime() < 400) {
          robot.intake.update(1, true, robot.getAllianceColor());
        } else if (pathTimer.getElapsedTime() > .4) {
          robot.intake.update(0.05, true, robot.getAllianceColor());
          setPathState(14);
        }

        break;
      case 20:
        subCycleToBucket(new double[]{INTAKE_SUB_PRIME[0] + 3, INTAKE_SUB_PRIME[1], INTAKE_SUB_PRIME[2]}, 21);
        break;
      case 21:
        if (robot.horSlide.magLim.isPressed()) {
          transferTimer.reset();
          setPathState(22);
        }
        break;
      case 22:
        subCycleTransfer(new double[]{INTAKE_SUB_PRIME[0], INTAKE_SUB_PRIME[1], INTAKE_SUB_PRIME[2]}, 23);
        break;
    }
  }

  private void subCycleToBucket(double[] subPos, int next) {
    robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS - 20);
    robot.follower.followPath(robot.follower.pathBuilder()
        .addBezierCurve(
            pointFromArr(subPos),
            pointFromArr(BUCKET_INTAKE_SUB_CONTROL),
            pointFromArr(PLACE_BUCKET_THREE)
        )
        .setLinearHeadingInterpolation(Math.toRadians(subPos[2]),
            Math.toRadians(PLACE_BUCKET_THREE[2]))
        .addParametricCallback(.1, () -> {

        })
        .setZeroPowerAccelerationMultiplier(5)
        .build(), true);
    setPathState(next);
  }

  private void subCycleTransfer(double[] postPos, int next) {
    if (transferTimer.milliseconds() > 200) {
      robot.claw.clawClose();
      if (transferTimer.milliseconds() > 250) {
        robot.slides.setTarget(VerticalSlides.UP_AUTO);
        if (robot.slides.atTarget(MOVE_ARM_HEIGHT_OFFSET)) {
          robot.claw.setBucket();
          if (!robot.follower.isBusy()) {
            robot.horSlide.setTarget(HSLIDE_SUB);
            place(robot.follower.pathBuilder()
                .addBezierCurve(
                    pointFromArr(PLACE_BUCKET_THREE),
                    pointFromArr(BUCKET_INTAKE_SUB_CONTROL),
                    pointFromArr(postPos)
                )
                .setTangentHeadingInterpolation()
                .addParametricCallback(.5, () -> robot.horSlide.setTarget(HSLIDE_SUB))
                .setZeroPowerAccelerationMultiplier(5)
                .build(), 30, 50);
            setPathState(next);
          }
        }
      }
    }
  }


  private void place(PathChain nextPath) {
    this.place(nextPath, 350, 50);
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