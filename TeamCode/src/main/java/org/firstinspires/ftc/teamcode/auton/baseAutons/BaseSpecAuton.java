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

  public static double[] START = {10, 63, 180};
  public static double[] PLACE_SPEC = {36, 67, 180};

  public static double[] PLACE_SPEC_FIRST = {32, 69, 180};

  public static double[] DRIVE_ONE = {53, 34, 180};

  public static double[] CONTROL_DRIVE_ONE = {2, 35};
  public static double[] PUSH_ONE = {25, 22, 180};

  public static double[] CONTROL_PUSH_ONE = {61, 25};
  public static double[] DRIVE_TWO = {50, 26, 180};

  public static double[] PUSH_TWO = {25, 16, 180};

  public static double[] CONTROL_PUSH_TWO = {61, 8};


  public static double[] DRIVE_THREE = {50, 13, 180};

  public static double[] PUSH_THREE = {25, 10, 180};

  public static double[] CONTROL_PUSH_THREE = {61, 5};

  public static double[] PICKUP = {13.5, 30, 180};

  public static double[] CONTROL_PICKUP = {32, 30};


  private int pathState = 1000;
  private Timer pathTimer;

  final NewRobot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;

  PathChain placePreLoad,
      driveOne,
      pickup, place;


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
            pointFromArr(PICKUP)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PUSH_THREE[2]), Math.toRadians(PICKUP[2]))
        .setZeroPowerAccelerationMultiplier(5) // TODO: test if this improves push speed?
        .build();

    // TODO: possible using tangential would be faster for these (add 2 control points in line with the pickup/place pts
    //  to maintain correct heading
    place = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(PICKUP),
            pointFromArr(PLACE_SPEC)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PICKUP[2]), Math.toRadians(PLACE_SPEC[2]))
        .build();

    pickup = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(PLACE_SPEC),
            pointFromArr(PICKUP)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PLACE_SPEC[2]), Math.toRadians(PICKUP[2]))
        .build();
  }

  public void setPathState(int pState) {
    pathState = pState;
    pathTimer.resetTimer();
  }

  public void pickupPlace(PathChain place, PathChain postPlace) {
    robot.claw.clawClose();
    robot.waitTime(50);
    robot.slides.setTarget(VerticalSlides.BAR_PLACE_AUTO);
    robot.claw.setPlaceAuto();

    robot.follower.followPath(place);
    while (opMode.opModeIsActive() && (robot.follower.isBusy() || !robot.slides.atTarget())) {
      robot.updateAutoControls();
    }

    robot.slides.setTarget(VerticalSlides.TRANSFER);

    while (opMode.opModeIsActive() && !robot.slides.atTarget()) {
      robot.updateAutoControls();
    }

    // TODO: added waitTimes here are (100+100)*4 ms
    robot.claw.clawOpenWall();
    // TODO: you only wait 50ms for the claw to close, does it need 100 to open?
    robot.waitTime(50);
    robot.claw.setWall();
    // TODO: maybe remove this one? can alternatively use a parametric callback on the path to do this arm movement while moving

    robot.follower.followPath(postPlace);
  }

  public void autonomousPathUpdate() {
    switch (pathState) {
      case 1000:
        robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
        robot.slides.setTarget(VerticalSlides.TRANSFER);
        robot.intake.update(0, false, robot.getAllianceColor());

        robot.slides.setTarget(VerticalSlides.BAR_PLACE_AUTO);
        robot.claw.setPlaceAuto();

        robot.follower.followPath(placePreLoad);
        while (opMode.opModeIsActive() && (robot.follower.isBusy() || !robot.slides.atTarget())) {
          robot.updateAutoControls();
        }

        robot.slides.setTarget(VerticalSlides.TRANSFER);

        while (opMode.opModeIsActive() && !robot.slides.atTarget()) {
          robot.updateAutoControls();
        }

        robot.claw.clawOpenWall();
        robot.waitTime(100);
        robot.claw.setWall();

        robot.follower.followPath(driveOne);
        setPathState(2000);
        break;

      case 2000:
        if (!robot.follower.isBusy()) {
          pickupPlace(place, pickup);
        }
        break;
    }
  }

  public void run() {
    pathTimer = new Timer();
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
    robot.horSlide.setMode(RunMode.RUN_WITHOUT_ENCODER);

    while (this.opMode.opModeIsActive()) {
      robot.updateAutoControls();

      autonomousPathUpdate();

      telemetry.addData("Path State", pathState);
      telemetry.addData("Position", robot.follower.getPose().toString());
      telemetry.update();
    }
  }
}