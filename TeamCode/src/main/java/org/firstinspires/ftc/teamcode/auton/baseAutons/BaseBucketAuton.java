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

  public static int MOVE_ARM_HEIGHT_OFFSET = 400;

  public static int HSLIDE_1 = HorizontalSlides.OUT_POS / 4;
  public static int HSLIDE_2 = HorizontalSlides.OUT_POS / 3;
  public static int HSLIDE_3 = HorizontalSlides.OUT_POS / 3;

  public static double INTAKE_OVERRIDE = 4;

  public static double SUB_SLIDE_EXTEND_T = 0.78;
  public static double OUT_IN_MS = 100;

  // MAIN POINTS

  public static double[] START = {9.5, 105, 270};
  public static double[] PLACE_BUCKET = {15, 129, 315};
  public static double[] PLACE_BUCKET_SAFE = {16, 126, 315};

  public static double[] PLACE_BUCKET_MORE_SAFE = {16, 126, 315};

  public static double[] PLACE_BUCKET_EVEN_MORE_SAFE = {15, 126, 315};


  public static double[] INTAKE_ONE = {19, 124, 360};
  public static double[] INTAKE_TWO = {19, 130, 360};
  public static double[] INTAKE_THREE = {28, 125, 50};
  public static double[] INTAKE_SUB = {64, 105, 270};
  public static double[] INTAKE_SUB_PRIME = {64, 97, 270};

  public static double[] INTAKE_SUB_2 = {67, 104, 270};
  public static double[] INTAKE_SUB_2_PRIME = {67, 97, 285};
  // CONTROL POINTS
  public static double[] BUCKET_INTAKE_SUB_CONTROL = {64, 128};
  public static double[] BUCKET_INTAKE_SUB_CONTROL_2 = {67, 128};

  PathChain placePreLoad,
      intakeOne, placeOne,
      intakeTwo, placeTwo,
      intakeThree, placeThree,
      bucketToSub,
      subPickupForward, subPickupBackward,
      bucketToSub2,
      subPickupForward2, subPickupBackward2,
      park;

  private int pathState = 0;

  private final Timer pathTimer = new Timer();
  private final ElapsedTime subTimer = new ElapsedTime();

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
            pointFromArr(PLACE_BUCKET_SAFE)
        )
        .setLinearHeadingInterpolation(Math.toRadians(START[2]), Math.toRadians(PLACE_BUCKET_SAFE[2]))
        .setPathEndTimeoutConstraint(500) // TODO: tune lower?
        .build();

    intakeOne = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(PLACE_BUCKET_SAFE),
            pointFromArr(INTAKE_ONE)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PLACE_BUCKET_SAFE[2]), Math.toRadians(INTAKE_ONE[2]))
        .setPathEndTimeoutConstraint(350)
        .build();

    placeOne = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_ONE),
            pointFromArr(PLACE_BUCKET)
        )
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_ONE[2]), Math.toRadians(PLACE_BUCKET[2]))
        .build();

    intakeTwo = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(PLACE_BUCKET),
            pointFromArr(INTAKE_TWO)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PLACE_BUCKET[2]), Math.toRadians(INTAKE_TWO[2]))
        .setPathEndTimeoutConstraint(250)
        .build();

    placeTwo = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_TWO),
            pointFromArr(PLACE_BUCKET)
        )
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_TWO[2]), Math.toRadians(PLACE_BUCKET[2]))
        .build();

    intakeThree = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(PLACE_BUCKET),
            pointFromArr(INTAKE_THREE)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PLACE_BUCKET[2]), Math.toRadians(INTAKE_THREE[2]))
        .build();

    placeThree = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_THREE),
            pointFromArr(PLACE_BUCKET)
        )
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_THREE[2]), Math.toRadians(PLACE_BUCKET[2]))
        .build();

    bucketToSub = robot.follower.pathBuilder()
        .addBezierCurve(
            pointFromArr(PLACE_BUCKET),
            pointFromArr(BUCKET_INTAKE_SUB_CONTROL),
            pointFromArr(INTAKE_SUB)
        )
        .setTangentHeadingInterpolation()
        .addParametricCallback(SUB_SLIDE_EXTEND_T, () -> robot.horSlide.setTarget(HorizontalSlides.OUT_POS))
        .addParametricCallback(1.0, () -> robot.intake.update(-1, true, robot.getAllianceColor()))
        .setPathEndTimeoutConstraint(300)
        .build();

    bucketToSub2 = robot.follower.pathBuilder()
        .addBezierCurve(
            pointFromArr(PLACE_BUCKET),
            pointFromArr(BUCKET_INTAKE_SUB_CONTROL_2),
            pointFromArr(INTAKE_SUB_2)
        )
        .setTangentHeadingInterpolation()
        .addParametricCallback(SUB_SLIDE_EXTEND_T, () -> robot.horSlide.setTarget(HorizontalSlides.OUT_POS))
        .addParametricCallback(1.0, () -> robot.intake.update(-1, true, robot.getAllianceColor()))
        .setPathEndTimeoutConstraint(300)
        .build();

    subPickupForward = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_SUB),
            pointFromArr(INTAKE_SUB_PRIME)
        )
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_SUB[2]), Math.toRadians(INTAKE_SUB_PRIME[2]))
        .setPathEndTimeoutConstraint(200)
        .setZeroPowerAccelerationMultiplier(5)
        .build();

    subPickupBackward = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_SUB_PRIME),
            pointFromArr(INTAKE_SUB)
        )
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_SUB_PRIME[2]), Math.toRadians(INTAKE_SUB[2]))
        .setPathEndTimeoutConstraint(200)
        .setZeroPowerAccelerationMultiplier(5)
        .build();

    subPickupForward2 = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_SUB_2),
            pointFromArr(INTAKE_SUB_2_PRIME)
        )
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_SUB_2[2]), Math.toRadians(INTAKE_SUB_2_PRIME[2]))
        .setPathEndTimeoutConstraint(200)
        .setZeroPowerAccelerationMultiplier(5)
        .build();

    subPickupBackward2 = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_SUB_2_PRIME),
            pointFromArr(INTAKE_SUB_2)
        )
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE_SUB_2_PRIME[2]), Math.toRadians(INTAKE_SUB_2[2]))
        .setPathEndTimeoutConstraint(200)
        .setZeroPowerAccelerationMultiplier(5)
        .build();

    park = robot.follower.pathBuilder()
        .addBezierCurve(
            pointFromArr(PLACE_BUCKET),
            pointFromArr(BUCKET_INTAKE_SUB_CONTROL),
            pointFromArr(INTAKE_SUB_PRIME)
        )
        .setTangentHeadingInterpolation()
        .addParametricCallback(SUB_SLIDE_EXTEND_T, () -> robot.horSlide.setTarget(HorizontalSlides.OUT_POS))
        .setZeroPowerAccelerationMultiplier(5)
        .build();
  }

  public void autonomousPathUpdate() {
    switch (pathState) {

      // MOVE TO SCORE PRELOAD
      case 0:
        robot.follower.followPath(placePreLoad, true);
        robot.slides.setTarget(VerticalSlides.UP_AUTO);
        robot.horSlide.setTarget(HSLIDE_1);
        setPathState(100);
        break;

      case 100:
        if (robot.slides.atTarget()) {
          robot.claw.setBucket();
          setPathState(101);
        }
        break;

      // SCORE PRELOAD
      case 101:
        if (!robot.follower.isBusy() && robot.slides.atTarget()) {
          place(intakeOne, 400, 75);

          robot.intake.update(1, false, robot.getAllianceColor());
          setPathState(2);
        }
        break;

      // INTAKE 1
      case 2:
        robot.intake.update(1, false, robot.getAllianceColor());

        boolean validCollected1 = robot.intake.validSampleIn(robot.getAllianceColor());

        if (!robot.follower.isBusy() && !validCollected1) {
          robot.horSlide.setTarget(HorizontalSlides.OUT_POS);// after turn put intake down spin and extend
        }

        if (!robot.follower.isBusy() && validCollected1) {
          robot.intake.update(0, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);

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
        if (robot.horSlide.atTarget()) {
          setPathState(31);
        }
        break;

      case 31:
        if (pathTimer.getElapsedTime() > 50) {
          robot.claw.clawClose();
          setPathState(32);
        }
        break;

      case 32:
        if (pathTimer.getElapsedTime() > 50) {
          robot.slides.setTarget(VerticalSlides.UP_AUTO);
          robot.horSlide.setTarget(HSLIDE_2);
          setPathState(33);
        }
        break;

      case 33:
        // TODO: possible optimization: move arm earlier (needs tuning)
        if (robot.slides.atTarget(MOVE_ARM_HEIGHT_OFFSET)) {
          robot.claw.setBucket();
          setPathState(4);
        }
        break;

      // SCORE 1
      case 4:
        if (!robot.follower.isBusy() && robot.slides.atTarget()) {
          place(intakeTwo);
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
          robot.intake.update(0, true, robot.getAllianceColor());
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
        if (robot.horSlide.atTarget()) {
          setPathState(61);
        }
        break;

      case 61:
        if (pathTimer.getElapsedTime() > 50) {
          robot.claw.clawClose();
          setPathState(62);
        }
        break;

      case 62:
        if (pathTimer.getElapsedTime() > 50) {
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
          place(intakeThree, 400, 75);
          setPathState(8);
        }
        break;

      // INTAKE 3

      case 8:
        robot.intake.update(1, false, robot.getAllianceColor());

        boolean validCollected3 = robot.intake.validSampleIn(robot.getAllianceColor());

        if (!robot.follower.isBusy() && !validCollected3) {
          robot.horSlide.setTarget(HorizontalSlides.OUT_POS); // after turn put intake down spin and extend
        }

        if (!robot.follower.isBusy() && validCollected3) {
          robot.intake.update(0, true, robot.getAllianceColor());
          robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);

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
          setPathState(1100);
        }
        break;

      // TRANSFER 3
      case 9:
        if (robot.horSlide.atTarget()) {
          setPathState(91);
        }
        break;

      case 91:
        if (pathTimer.getElapsedTime() > 50) {
          robot.claw.clawClose();
          setPathState(92);
        }
        break;

      case 92:
        if (pathTimer.getElapsedTime() > 50) {
          robot.slides.setTarget(VerticalSlides.UP_AUTO);
          robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
          setPathState(93);
        }
        break;

      case 93:
        // TODO: possible optimization: move arm earlier (needs tuning)
        if (robot.slides.atTarget(MOVE_ARM_HEIGHT_OFFSET)) {
          robot.claw.setBucket();
          setPathState(10);
        }
        break;

      case 10:
        if (!robot.follower.isBusy() && robot.slides.atTarget()) {
          place(bucketToSub);
          setPathState(1100);
        }
        break;

      case 1100:
        if (!robot.follower.isBusy()) {
          cycleSub(subPickupForward, subPickupBackward, PLACE_BUCKET_MORE_SAFE);
          place(bucketToSub2, 400, 500);
          setPathState(1200);
        }
        break;

      case 1200:
        if (!robot.follower.isBusy()) {
          cycleSub(subPickupForward2, subPickupBackward2, PLACE_BUCKET_EVEN_MORE_SAFE);
          place(park, 500, 700);
          setPathState(1300);
        }
        break;
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

  private void cycleSub(PathChain forwardPath, PathChain backPath, double[] place) {
    robot.intake.update(1, false, robot.getAllianceColor());
    robot.slides.setTarget(VerticalSlides.TRANSFER);
    robot.claw.setTransfer();
    robot.claw.clawOpen();

    boolean driveForward = true;
    ElapsedTime validSampleTimer = new ElapsedTime();
    validSampleTimer.reset();
    subTimer.reset();
    while (opMode.opModeIsActive() && validSampleTimer.milliseconds() < 300) {
      // move between the two positions
      if (!robot.follower.isBusy()) {
        robot.follower.followPath(driveForward ? forwardPath : backPath, true);
        driveForward = !driveForward;
      }

      robot.updateAutoControls();

      boolean validIn = robot.intake.validSampleIn(robot.getAllianceColor());
      if (!validIn || robot.intake.spitTimer.milliseconds() < 500) {
        validSampleTimer.reset();
      }

      if (!validIn && subTimer.milliseconds() < 2000) {
        robot.intake.update(1, false, robot.getAllianceColor());
      } else if (!validIn && subTimer.milliseconds() < 2100) {
        robot.intake.update(-.6, false, robot.getAllianceColor());
      } else {
        robot.intake.update(1, false, robot.getAllianceColor());
        subTimer.reset();
      }

    }

    // OUTTAKE
    subTimer.reset();
    while (opMode.opModeIsActive() && subTimer.milliseconds() < OUT_IN_MS) {
      robot.intake.update(-1, true, robot.getAllianceColor());
      robot.updateAutoControls();
    }

    // RE-INTAKE
    robot.intake.update(1, true, robot.getAllianceColor());
    subTimer.reset();
    while (opMode.opModeIsActive() && !robot.intake.validSampleIn(robot.getAllianceColor())) {
      robot.updateAutoControls();
      robot.intake.update(1, true, robot.getAllianceColor());
    }

    // RETRACT H SLIDE, START FOLLOWING PATH TO BUCKET
    Pose curPose = robot.follower.getPose();
    robot.follower.followPath(
        robot.follower.pathBuilder()
            .addBezierCurve(
                new Point(curPose),
                pointFromArr(BUCKET_INTAKE_SUB_CONTROL),
                pointFromArr(place)
            )
            .setLinearHeadingInterpolation(curPose.getHeading(), Math.toRadians(place[2]))
            .setPathEndTimeoutConstraint(800)
            .setZeroPowerAccelerationMultiplier(3.5)
            .build()
        , true);

    robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);

    do {
      robot.updateAutoControls();
    } while (opMode.opModeIsActive() && !robot.horSlide.atTarget());

    robot.intake.update(0, true, robot.getAllianceColor());
    // Let h slide settle for transfer
    subTimer.reset();
    while (opMode.opModeIsActive() && subTimer.milliseconds() < 75) {
      robot.updateAutoControls();
    }

    // Close claw
    robot.claw.clawClose();
    subTimer.reset();
    while (opMode.opModeIsActive() && subTimer.milliseconds() < 75) {
      robot.updateAutoControls();
    }

    // Raise Slides + Rotate Arm
    robot.slides.setTarget(VerticalSlides.UP); // INTENTIONAL USE OF TELEOP POSITION
    do {
      robot.updateAutoControls();
    } while (opMode.opModeIsActive() && !robot.slides.atTarget(MOVE_ARM_HEIGHT_OFFSET - 50));

    robot.claw.setBucket();

    // wait for path and slide move end
    while (opMode.opModeIsActive() && !robot.slides.atTarget() && robot.follower.isBusy()) {
      robot.updateAutoControls();
    }
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
      telemetry.update();
    }
  }

}