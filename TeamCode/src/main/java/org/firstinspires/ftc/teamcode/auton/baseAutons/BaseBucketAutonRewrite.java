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
public class BaseBucketAutonRewrite {

  public static int MOVE_ARM_HEIGHT_OFFSET = 400;

  public static int HSLIDE_1 = HorizontalSlides.OUT_POS / 4;
  public static int HSLIDE_2 = HorizontalSlides.OUT_POS / 3;
  public static int HSLIDE_3 = HorizontalSlides.OUT_POS / 3;
  public static int HSLIDE_SUB = 150; // TODO: TUNE IN HSLIDE PID MODE

  public static double INTAKE_OVERRIDE_MS = 4000;
  public static double SUB_BUCKET_MAX_POW = 1.0;
  public static double SWEEP_MS = 300;

  public static double SUB_SLIDE_EXTEND_T = 0.78;
  public static double OUT_IN_MS = 100;

  public static int[] INTAKE_TIMEOUTS = {500, 500, 500, 500, 500};
  public static int[] PLACE_TIMEOUTS = {500, 500, 500, 500, 500};

  // MAIN POINTS

  public static double[] START = {9, 105, 270};
  public static double[] BUCKET = {16, 128, 315};

  public static double[] INTAKE_1 = {21, 124.5, 0};
  public static double[] INTAKE_2 = {21, 129, 0};
  public static double[] INTAKE_3 = {28, 125, 50};

  public static double[] INTAKE_SUB = {64, 97, 270};
  public static double[] INTAKE_SUB_2 = {67, 97, 270};


  PathChain placePreLoad,
      intakeOne, placeOne, intakeOneFail,
      intakeTwo, placeTwo, intakeTwoFail,
      intakeThree, placeThree, intakeThreeFail,
      bucketToSub, subToBucket,
      bucketToSub2, subToBucket2,
      park;

  private int pathState = 0;

  private final Timer pathTimer = new Timer();
  private final ElapsedTime timer = new ElapsedTime();

  final NewRobot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;

  public BaseBucketAutonRewrite(LinearOpMode opMode, NewRobot robot) {
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

  public double headingFromArr(double[] arr) {
    return Math.toRadians(arr[2]);
  }

  public void setPathState(int pState) {
    pathState = pState;
    pathTimer.resetTimer();
  }


  public void buildPaths() {

    double[] SUB_CONTROL_BUCKET = {BUCKET[0] + 16, BUCKET[1] - 16};
    double[] SUB_CONTROL_SUB = {INTAKE_SUB[0], INTAKE_SUB[1] + 15};
    double[] SUB_CONTROL_SUB_2 = {INTAKE_SUB_2[0], INTAKE_SUB_2[1] + 15};

    placePreLoad = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(START),
            pointFromArr(BUCKET)
        )
        .setLinearHeadingInterpolation(headingFromArr(START), headingFromArr(BUCKET))
        .setPathEndTimeoutConstraint(PLACE_TIMEOUTS[0])
        .addParametricCallback(0.0, () -> robot.slides.setTarget(VerticalSlides.UP_AUTO))
        .build();

    intakeOne = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(BUCKET),
            pointFromArr(INTAKE_1)
        )
        .setLinearHeadingInterpolation(headingFromArr(BUCKET), headingFromArr(INTAKE_1))
        .setPathEndTimeoutConstraint(INTAKE_TIMEOUTS[0])
        .addParametricCallback(0.0, () -> robot.horSlide.setTarget(HSLIDE_1))
        .addParametricCallback(1.0, () -> robot.horSlide.setTarget(HorizontalSlides.OUT_POS))
        .build();

    placeOne = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_1),
            pointFromArr(BUCKET)
        )
        .setLinearHeadingInterpolation(headingFromArr(INTAKE_1), headingFromArr(BUCKET))
        .setPathEndTimeoutConstraint(PLACE_TIMEOUTS[1])
        .build();

    intakeOneFail = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_1),
            pointFromArr(INTAKE_2)
        )
        .setLinearHeadingInterpolation(headingFromArr(INTAKE_1), headingFromArr(INTAKE_2))
        .addParametricCallback(0.0, () -> robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS))
        .addParametricCallback(1.0, () -> robot.horSlide.setTarget(HorizontalSlides.OUT_POS))
        .build();

    intakeTwo = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(BUCKET),
            pointFromArr(INTAKE_2)
        )
        .setLinearHeadingInterpolation(headingFromArr(BUCKET), headingFromArr(INTAKE_2))
        .setPathEndTimeoutConstraint(INTAKE_TIMEOUTS[1])
        .addParametricCallback(0.0, () -> robot.horSlide.setTarget(HSLIDE_2))
        .addParametricCallback(1.0, () -> robot.horSlide.setTarget(HorizontalSlides.OUT_POS))
        .build();

    placeTwo = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_2),
            pointFromArr(BUCKET)
        )
        .setLinearHeadingInterpolation(headingFromArr(INTAKE_2), headingFromArr(BUCKET))
        .setPathEndTimeoutConstraint(PLACE_TIMEOUTS[2])
        .build();

    intakeTwoFail = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_2),
            pointFromArr(INTAKE_3)
        )
        .setLinearHeadingInterpolation(headingFromArr(INTAKE_2), headingFromArr(INTAKE_3))
        .addParametricCallback(0.0, () -> robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS))
        .addParametricCallback(1.0, () -> robot.horSlide.setTarget(HorizontalSlides.OUT_POS))
        .build();

    intakeThree = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(BUCKET),
            pointFromArr(INTAKE_3)
        )
        .setLinearHeadingInterpolation(headingFromArr(BUCKET), headingFromArr(INTAKE_3))
        .setPathEndTimeoutConstraint(INTAKE_TIMEOUTS[2])
        .addParametricCallback(0.0, () -> robot.horSlide.setTarget(HSLIDE_3))
        .addParametricCallback(1.0, () -> robot.horSlide.setTarget(HorizontalSlides.OUT_POS))
        .build();

    placeThree = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_3),
            pointFromArr(BUCKET)
        )
        .setLinearHeadingInterpolation(headingFromArr(INTAKE_3), headingFromArr(BUCKET))
        .setPathEndTimeoutConstraint(PLACE_TIMEOUTS[3])
        .build();

    intakeThreeFail = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE_3),
            pointFromArr(INTAKE_SUB)
        )
        .setLinearHeadingInterpolation(headingFromArr(INTAKE_3), headingFromArr(INTAKE_SUB))
        .addParametricCallback(0.0, () -> robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS))
        .addParametricCallback(1.0, () -> {
          robot.horSlide.setTarget(HSLIDE_SUB);
          robot.intake.sweepOut(true);
        })
        .build();

    bucketToSub = robot.follower.pathBuilder()
        .addBezierCurve(
            pointFromArr(BUCKET),
            pointFromArr(SUB_CONTROL_BUCKET),
            pointFromArr(SUB_CONTROL_SUB),
            pointFromArr(INTAKE_SUB)
        )
        .setTangentHeadingInterpolation()
        .addParametricCallback(SUB_SLIDE_EXTEND_T, () -> robot.horSlide.setTarget(HSLIDE_SUB))
        .addParametricCallback(1.0, () -> robot.intake.sweepOut(true))
        .setPathEndTimeoutConstraint(INTAKE_TIMEOUTS[4])
        .build();

    subToBucket = robot.follower.pathBuilder()
        .addBezierCurve(
            pointFromArr(INTAKE_SUB),
            pointFromArr(SUB_CONTROL_SUB),
            pointFromArr(SUB_CONTROL_BUCKET),
            pointFromArr(BUCKET)
        )
        .setTangentHeadingInterpolation()
        .setPathEndTimeoutConstraint(PLACE_TIMEOUTS[4])
        .build();

    bucketToSub2 = robot.follower.pathBuilder()
        .addBezierCurve(
            pointFromArr(BUCKET),
            pointFromArr(SUB_CONTROL_BUCKET),
            pointFromArr(SUB_CONTROL_SUB_2),
            pointFromArr(INTAKE_SUB_2)
        )
        .setTangentHeadingInterpolation()
        .addParametricCallback(SUB_SLIDE_EXTEND_T, () -> robot.horSlide.setTarget(HSLIDE_SUB))
        .addParametricCallback(1.0, () -> robot.intake.sweepOut(true))
        .setPathEndTimeoutConstraint(INTAKE_TIMEOUTS[5])
        .build();

    subToBucket2 = robot.follower.pathBuilder()
        .addBezierCurve(
            pointFromArr(INTAKE_SUB_2),
            pointFromArr(SUB_CONTROL_SUB_2),
            pointFromArr(SUB_CONTROL_BUCKET),
            pointFromArr(BUCKET)
        )
        .setTangentHeadingInterpolation()
        .setPathEndTimeoutConstraint(PLACE_TIMEOUTS[5])
        .build();

    park = robot.follower.pathBuilder()
        .addBezierCurve(
            pointFromArr(BUCKET),
            pointFromArr(SUB_CONTROL_BUCKET),
            pointFromArr(SUB_CONTROL_SUB),
            pointFromArr(INTAKE_SUB)
        )
        .setTangentHeadingInterpolation()
        .addParametricCallback(0.0, robot.claw::setInit)
        .setZeroPowerAccelerationMultiplier(5)
        .build();
  }

  public void autonomousPathUpdate() {
    switch (pathState) {

      // --- PRELOAD ---
      case 0:
        robot.follower.followPath(placePreLoad, true);
        setPathState(100);
        break;

      case 100:
        if (robot.slides.atTarget(MOVE_ARM_HEIGHT_OFFSET)) {
          robot.claw.setBucket();
          setPathState(200);
        }
        break;

      case 200:
        if (!robot.follower.isBusy() && robot.slides.atTarget()) {
          place(intakeOne);
          setPathState(1000);
        }
        break;

      // --- INTAKE 1 ---
      case 1000:
        robot.intake.update(1, false, robot.getAllianceColor());
        if (!robot.follower.isBusy()) {
          robot.horSlide.setTarget(HorizontalSlides.OUT_POS);
          setPathState(1500);
        }
        break;

      case 1500:
        if (robot.horSlide.atTarget()) {
          boolean success = intakeBlock();
          if (success) {
            robot.follower.followPath(placeOne, true);
            setPathState(2000);
          } else {
            robot.follower.followPath(intakeOneFail, true);
            setPathState(3000);
          }
        }
        break;

      // --- PLACE 1 ---
      case 2000:
        transferBlock();
        place(intakeTwo);
        setPathState(3000);
        break;

      // --- INTAKE 2 ---
      case 3000:
        robot.intake.update(1, false, robot.getAllianceColor());
        if (!robot.follower.isBusy()) {
          robot.horSlide.setTarget(HorizontalSlides.OUT_POS);
          setPathState(3500);
        }
        break;

      case 3500:
        if (robot.horSlide.atTarget()) {
          boolean success = intakeBlock();
          if (success) {
            robot.follower.followPath(placeTwo, true);
            setPathState(4000);
          } else {
            robot.follower.followPath(intakeTwoFail, true);
            setPathState(5000);
          }
        }
        break;

      // --- PLACE 2 ---
      case 4000:
        transferBlock();
        place(intakeThree);
        setPathState(5000);
        break;

      // --- INTAKE 3 ---
      case 5000:
        robot.intake.update(1, false, robot.getAllianceColor());
        if (!robot.follower.isBusy()) {
          robot.horSlide.setTarget(HorizontalSlides.OUT_POS);
          setPathState(5500);
        }
        break;

      case 5500:
        if (robot.horSlide.atTarget()) {
          boolean success = intakeBlock();
          if (success) {
            robot.follower.followPath(placeThree, true);
            setPathState(6000);
          } else {
            robot.follower.followPath(intakeThreeFail, true);
            setPathState(7000);
          }
        }
        break;

      // --- PLACE 3 ---
      case 6000:
        transferBlock();
        place(bucketToSub);
        setPathState(7000);
        break;

      case 7000:
        if (!robot.follower.isBusy()) {
          cycleSub(poseFromArr(INTAKE_SUB), subToBucket);
          place(bucketToSub2);
          setPathState(8000);
        }
        break;

      case 8000:
        if (!robot.follower.isBusy()) {
          cycleSub(poseFromArr(INTAKE_SUB_2), subToBucket2);
          place(park);
          setPathState(9000);
        }
        break;

    }
  }

  private boolean intakeBlock() {
    timer.reset();
    do {
      robot.intake.update(1, false, robot.getAllianceColor());
      robot.updateAutoControls();
    } while (opMode.opModeIsActive()
        && !robot.intake.validSampleIn(robot.getAllianceColor())
        && timer.milliseconds() < INTAKE_OVERRIDE_MS
    );

    return robot.intake.validSampleIn(robot.getAllianceColor());
  }

  private void transferBlock() {
    robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
    robot.intake.update(0, true, robot.getAllianceColor());

    do {
      robot.updateAutoControls();
    } while (opMode.opModeIsActive() && !robot.horSlide.atTarget());

    // Let h slide settle for transfer
    timer.reset();
    while (opMode.opModeIsActive() && timer.milliseconds() < 75) {
      robot.updateAutoControls();
    }

    // Close claw
    robot.claw.clawClose();
    timer.reset();
    while (opMode.opModeIsActive() && timer.milliseconds() < 75) {
      robot.updateAutoControls();
    }

    // Raise Slides + Rotate Arm
    robot.slides.setTarget(VerticalSlides.UP_AUTO);
    do {
      robot.updateAutoControls();
    } while (opMode.opModeIsActive() && !robot.slides.atTarget(MOVE_ARM_HEIGHT_OFFSET));

    robot.claw.setBucket();

    // wait for path and slide move end
    while (opMode.opModeIsActive() && !robot.slides.atTarget() && robot.follower.isBusy()) {
      robot.updateAutoControls();
    }

  }

  private void place(PathChain nextPath) {
    this.place(nextPath, 300, 50);
  }

  private void place(PathChain nextPath, int initialDelay, int delay) {
    timer.reset();
    while (opMode.opModeIsActive() && timer.milliseconds() < initialDelay) {
      robot.updateAutoControls();
    }
    robot.claw.clawOpen();

    timer.reset();
    while (opMode.opModeIsActive() && timer.milliseconds() < delay) {
      robot.updateAutoControls();
    }

    robot.claw.setTransfer();
    robot.slides.setTarget(VerticalSlides.TRANSFER);
    robot.follower.followPath(nextPath, true);
  }

  private void cycleSub(Pose p1, PathChain returnPath) {
    robot.follower.holdPoint(p1);

    timer.reset();
    while (opMode.opModeIsActive() && timer.milliseconds() < SWEEP_MS) {
      robot.updateAutoControls();
    }
    robot.intake.sweepOut(false);

    timer.reset();
    while (opMode.opModeIsActive() && timer.milliseconds() < SWEEP_MS) {
      robot.updateAutoControls();
    }

    robot.intake.update(1, false, robot.getAllianceColor());
    robot.slides.setTarget(VerticalSlides.TRANSFER);
    robot.claw.setTransfer();
    robot.claw.clawOpen();
    robot.horSlide.setTarget(HorizontalSlides.OUT_POS / 2);
    robot.updateAutoControls();

    timer.reset();
    while (opMode.opModeIsActive() && !robot.intake.validSampleIn(robot.getAllianceColor())) {
      robot.updateAutoControls();
      if (timer.milliseconds() < 2000) {
        robot.intake.update(1, false, robot.getAllianceColor());
      } else if (timer.milliseconds() < 2100) {
        robot.intake.update(-.5, false, robot.getAllianceColor());
      } else {
        robot.intake.update(1, false, robot.getAllianceColor());
        timer.reset();
      }
      if (robot.horSlide.atTarget()) {
        robot.horSlide.setTarget(HorizontalSlides.OUT_POS);
      }
    }

    // OUTTAKE
    timer.reset();
    while (opMode.opModeIsActive() && timer.milliseconds() < OUT_IN_MS) {
      robot.intake.update(-1, true, robot.getAllianceColor());
      robot.updateAutoControls();
    }

    // RE-INTAKE
    robot.intake.update(1, true, robot.getAllianceColor());
    timer.reset();
    while (opMode.opModeIsActive() && !robot.intake.validSampleIn(robot.getAllianceColor())) {
      robot.updateAutoControls();
      robot.intake.update(1, true, robot.getAllianceColor());
    }

    // RETRACT H SLIDE, START FOLLOWING PATH TO BUCKET
    robot.follower.followPath(returnPath, SUB_BUCKET_MAX_POW, true);
    robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);

    do {
      robot.updateAutoControls();
    } while (opMode.opModeIsActive() && !robot.horSlide.atTarget());

    robot.intake.update(0, true, robot.getAllianceColor());
    // Let h slide settle for transfer
    timer.reset();
    while (opMode.opModeIsActive() && timer.milliseconds() < 75) {
      robot.updateAutoControls();
    }

    // Close claw
    robot.claw.clawClose();
    timer.reset();
    while (opMode.opModeIsActive() && timer.milliseconds() < 75) {
      robot.updateAutoControls();
    }

    // Raise Slides + Rotate Arm
    robot.slides.setTarget(VerticalSlides.UP_AUTO);
    do {
      robot.updateAutoControls();
    } while (opMode.opModeIsActive() && !robot.slides.atTarget(MOVE_ARM_HEIGHT_OFFSET));

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
    robot.horSlide.setMode(RunMode.RUN_WITHOUT_ENCODER);

    while (this.opMode.opModeIsActive()) {
      robot.updateAutoControls();
      autonomousPathUpdate();

      telemetry.addData("Path State", pathState);
      telemetry.update();
    }
  }

}