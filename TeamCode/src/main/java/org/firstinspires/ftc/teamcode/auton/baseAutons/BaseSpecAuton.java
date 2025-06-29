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
public class BaseSpecAuton {

  public static double[] START = {10, 58, 180};
  public double[] PLACE_SPEC = {35, 69, 180};


  public static double[] PLACE_SPEC_FIRST = {35, 67, 180};

  public static double[] DRIVE_ONE = {45, 36, 180};

  public static double[] CONTROL_DRIVE_ONE = {2, 35};
  public static double[] PUSH_ONE = {25, 22, 180};

  public static double[] CONTROL_PUSH_ONE = {61, 25};
  public static double[] DRIVE_TWO = {45, 28, 180};

  public static double[] PUSH_TWO = {25, 17, 180};
  public static double[] CONTROL_PUSH_TWO = {61, 8};

  public static double[] DRIVE_THREE = {45, 18, 180};

  public static double[] PUSH_THREE = {25, 12, 180};
  public static double[] CONTROL_PUSH_THREE = {61, 5};

  public static double[] PICKUP_ONE = {18, 32, 180};
  public static double[] PICKUP = {15, 32, 180};

  public static double[] CONTROL_PICKUP = {35, 30};

  public static double[] INTAKE = {16, 50, 240};

  public static double[] BUCKET = {12, 60, 290}; // change to 125


  private int pathState = 1000;
  private int specCounter = 0;
  private Timer timer;
  private Timer pathTimer;

  final NewRobot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;


  PathChain
      placePreLoad,
      driveOne,
      pickup, place,
      intake, scoreSample;


  public BaseSpecAuton(LinearOpMode opMode, NewRobot robot) {
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
    // TODO maybe tangential for speed?
    placePreLoad = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(START),
            pointFromArr(PLACE_SPEC_FIRST)
        )
        .setLinearHeadingInterpolation(Math.toRadians(START[2]), Math.toRadians(PLACE_SPEC_FIRST[2]))
        .setPathEndTimeoutConstraint(100)
        .setZeroPowerAccelerationMultiplier(5)
        .build();

    driveOne = robot.follower.pathBuilder()
        .addBezierCurve(
            pointFromArr(PLACE_SPEC),
            pointFromArr(CONTROL_DRIVE_ONE),
            pointFromArr(DRIVE_ONE)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PLACE_SPEC[2]), Math.toRadians(DRIVE_ONE[2]))
        .setPathEndTimeoutConstraint(0)

        .addBezierCurve(
            pointFromArr(DRIVE_ONE),
            pointFromArr(CONTROL_PUSH_ONE),
            pointFromArr(PUSH_ONE)
        )
        .setLinearHeadingInterpolation(Math.toRadians(DRIVE_ONE[2]), Math.toRadians(PUSH_ONE[2]))
        .setPathEndTimeoutConstraint(0)

        .addBezierLine(
            pointFromArr(PUSH_ONE),
            pointFromArr(DRIVE_TWO)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PUSH_ONE[2]), Math.toRadians(DRIVE_TWO[2]))
        .setPathEndTimeoutConstraint(0)

        .addBezierCurve(
            pointFromArr(DRIVE_TWO),
            pointFromArr(CONTROL_PUSH_TWO),
            pointFromArr(PUSH_TWO)
        )
        .setLinearHeadingInterpolation(Math.toRadians(DRIVE_TWO[2]), Math.toRadians(PUSH_TWO[2]))
        .setPathEndTimeoutConstraint(0)

        .addBezierLine(
            pointFromArr(PUSH_TWO),
            pointFromArr(DRIVE_THREE)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PUSH_TWO[2]), Math.toRadians(DRIVE_THREE[2]))
        .setPathEndTimeoutConstraint(0)

        .addBezierCurve(
            pointFromArr(DRIVE_THREE),
            pointFromArr(CONTROL_PUSH_THREE),
            pointFromArr(PUSH_THREE)
        )
        .setLinearHeadingInterpolation(Math.toRadians(DRIVE_THREE[2]), Math.toRadians(PUSH_THREE[2]))
        .setPathEndTimeoutConstraint(0)

        .addBezierCurve(
            pointFromArr(PUSH_THREE),
            pointFromArr(CONTROL_PICKUP),
            pointFromArr(PICKUP_ONE)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PUSH_THREE[2]), Math.toRadians(PICKUP_ONE[2]))
        .setZeroPowerAccelerationMultiplier(8) // TODO: test if this improves push speed?
        .setPathEndTimeoutConstraint(50)
        .build();

    pickup = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(PLACE_SPEC),
            pointFromArr(PICKUP)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PLACE_SPEC[2]), Math.toRadians(PICKUP[2]))
        .setPathEndTimeoutConstraint(100)
        .setZeroPowerAccelerationMultiplier(5)
        .build();

    intake = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(PLACE_SPEC),
            pointFromArr(INTAKE)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PLACE_SPEC[2]), Math.toRadians(INTAKE[2]))
        .addParametricCallback(.5, () -> robot.horSlide.setTarget(HorizontalSlides.OUT_POS))
        .addParametricCallback(.5, () -> robot.intake.update(1, false, robot.getAllianceColor()))
        .addParametricCallback(.9, robot.claw::setTransfer)
        .addParametricCallback(.7, robot.claw::clawOpen)
        .addParametricCallback(.8, () -> robot.slides.setTarget(VerticalSlides.TRANSFER))
        .setPathEndTimeoutConstraint(50)
        .build();

    scoreSample = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(INTAKE),
            pointFromArr(BUCKET)

        )
        .setLinearHeadingInterpolation(Math.toRadians(INTAKE[2]), Math.toRadians(BUCKET[2]))
        .setPathEndTimeoutConstraint(50)
        .build();
  }

  public void setPathState(int pState) {
    pathState = pState;
    pathTimer.resetTimer();
  }

  public void pickupPlace(double[] poostCoords) {
    PLACE_SPEC = new double[]{35, PLACE_SPEC[1] + 2, 180};
    robot.claw.clawClose();

    timer.resetTimer();
    while (opMode.opModeIsActive() && timer.getElapsedTime() < 50) {
      robot.updateAutoControls();
    }

    robot.slides.setTarget(VerticalSlides.BAR_PLACE_UNDER_AUTO - 40);
    robot.claw.setPlace();

    robot.follower.followPath(robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(PICKUP),
            pointFromArr(PLACE_SPEC)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PICKUP[2]), Math.toRadians(PLACE_SPEC[2]))
        .setPathEndTimeoutConstraint(50)
        .build());

    while (opMode.opModeIsActive() && (robot.follower.isBusy() || !robot.slides.atTarget())) {
      robot.updateAutoControls();
    }

    robot.slides.setTarget(VerticalSlides.BAR_PLACE_TELEOP);

    while (opMode.opModeIsActive() && !robot.slides.atTarget()) {
      robot.updateAutoControls();
    }

    robot.claw.clawOpenWall();
    timer.resetTimer();
    while (opMode.opModeIsActive() && timer.getElapsedTime() < 50) {
      robot.updateAutoControls();
    }

    robot.claw.setWall();
    robot.slides.setTarget(VerticalSlides.TRANSFER);
    robot.follower.followPath(robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(PLACE_SPEC),
            pointFromArr(poostCoords)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PLACE_SPEC[2]), Math.toRadians(poostCoords[2]))
        .setPathEndTimeoutConstraint(50)
        .setZeroPowerAccelerationMultiplier(5)
        .build());
  }

  public void autonomousPathUpdate() {
    switch (pathState) {
      case 1000:
        robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
        robot.slides.setTarget(VerticalSlides.TRANSFER);
        robot.intake.update(0, true, robot.getAllianceColor());

        robot.slides.setTarget(VerticalSlides.BAR_PLACE_UNDER - 50);
        robot.claw.setPlace();

        robot.follower.followPath(placePreLoad);
        while (opMode.opModeIsActive() && (robot.follower.isBusy() || !robot.slides.atTarget())) {
          robot.updateAutoControls();
        }

        robot.slides.setTarget(VerticalSlides.BAR_PLACE_TELEOP);

        while (opMode.opModeIsActive() && !robot.slides.atTarget()) {
          robot.updateAutoControls();
        }

        robot.claw.clawOpenWall();
        timer.resetTimer();
        while (opMode.opModeIsActive() && timer.getElapsedTime() < 50) {
          robot.updateAutoControls();
        }

        robot.claw.setWall();
        robot.slides.setTarget(VerticalSlides.TRANSFER);
        robot.follower.followPath(driveOne);
        setPathState(2000);
        break;

      case 2000:
        if (!robot.follower.isBusy() && specCounter < 3) {
          pickupPlace(PICKUP);
          specCounter++;
        }
        if (specCounter >= 3) {
          setPathState(2001);
        }
        break;

      case 2001:
        if (!robot.follower.isBusy()) {
          pickupPlace(INTAKE);
          setPathState(3000);
        }
        break;

      case 3000:
        if (!robot.follower.isBusy()) {
          robot.intake.update(1, false, robot.getAllianceColor());
          boolean validCollected = robot.intake.validSampleIn(robot.getAllianceColor());
          if (validCollected) {
            robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS - 2);
            robot.intake.update(0.15, true, robot.getAllianceColor());
            robot.follower.followPath(scoreSample, true);
            setPathState(4000);
          }
        }
        break;

      case 4000:
        if (robot.horSlide.magLim.isPressed()) {
          robot.claw.clawClose();
          setPathState(5000);
        }
        break;

      case 5000:
        if (pathTimer.getElapsedTime() > 40) {
          robot.slides.setTarget(VerticalSlides.UP_AUTO);
          robot.horSlide.setTarget(HorizontalSlides.OUT_POS);
          setPathState(6000);
        }
        break;

      case 6000:
        if (robot.slides.atTarget(500)) {
          robot.claw.setBucket();
        }
        if (!robot.follower.isBusy() && robot.slides.atTarget() && pathTimer.getElapsedTime() > 200) {
          robot.claw.clawOpen();
          setPathState(7000);
        }
        break;

      case 7000:
        if (pathTimer.getElapsedTime() > 100) {
          robot.claw.setTransfer();
          robot.slides.setTarget(VerticalSlides.TRANSFER);
        }
        break;

    }
  }

  public void run() {
    pathTimer = new Timer();
    timer = new Timer();
    buildPaths();
    robot.initAutonSpec();

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
      telemetry.addData("Position", robot.follower.getPose().toString());
      telemetry.update();
    }
  }
}