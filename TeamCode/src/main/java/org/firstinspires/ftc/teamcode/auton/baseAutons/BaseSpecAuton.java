package org.firstinspires.ftc.teamcode.auton.baseAutons;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NewRobot;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlides;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides;

@Config
public class BaseSpecAuton {

  public static double[] START = {10, 63, 180};
  public static double[] PLACE_SPEC = {37, 63, 180};
  public static double[] DRIVE_ONE = {64, 34, 180};

  public static double[] CONTROL_DRIVE_ONE = {2, 35};
  public static double[] PUSH_ONE = {15, 25, 180};

  public static double[] CONTROL_PUSH_ONE = {68, 25};
  public static double[] DRIVE_TWO = {62, 25, 180};

  public static double[] PUSH_TWO = {14, 16, 180};

  public static double[] CONTROL_PUSH_TWO = {78, 10};


  public static double[] DRIVE_THREE = {62, 16, 180};

  public static double[] PUSH_THREE = {15, 9, 180};

  public static double[] CONTROL_PUSH_THREE = {32, 30};

  public static double[] PICKUP = {12, 30, 180};

  public static double[] CONTROL_PICKUP = {32, 30};


  private int pathState = 0;
  private Timer pathTimer;

  final NewRobot robot;
  final LinearOpMode opMode;
  final Telemetry telemetry;

  PathChain placePreLoad,
      driveOne, pushOne,
      driveTwo, pushTwo,
      driveThree, pushThree,
      pickupFirst, pickup, place,
      park;


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
    // TODO
    placePreLoad = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(START),
            pointFromArr(PLACE_SPEC)
        )
        .setLinearHeadingInterpolation(Math.toRadians(START[2]), Math.toRadians(PLACE_SPEC[2]))
        .build();
    driveOne = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(PLACE_SPEC),
            pointFromArr(DRIVE_ONE)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PLACE_SPEC[2]),
            Math.toRadians(DRIVE_ONE[2]))
        .addBezierCurve(
            pointFromArr(DRIVE_ONE),
            pointFromArr(CONTROL_PUSH_ONE),
            pointFromArr(PUSH_ONE)
        )
        .setLinearHeadingInterpolation(Math.toRadians(DRIVE_ONE[2]),
            Math.toRadians(PUSH_ONE[2]))

        .addBezierLine(
            pointFromArr(PUSH_ONE),
            pointFromArr(DRIVE_TWO)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PUSH_ONE[2]),
            Math.toRadians(DRIVE_TWO[2]))

        .addBezierCurve(
            pointFromArr(DRIVE_TWO),
            pointFromArr(CONTROL_PUSH_TWO),
            pointFromArr(PUSH_TWO)
        )
        .setLinearHeadingInterpolation(Math.toRadians(DRIVE_TWO[2]),
            Math.toRadians(PUSH_TWO[2]))

        .addBezierLine(
            pointFromArr(PUSH_TWO),
            pointFromArr(DRIVE_THREE)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PUSH_TWO[2]),
            Math.toRadians(DRIVE_THREE[2]))

        .addBezierCurve(
            pointFromArr(DRIVE_THREE),
            pointFromArr(CONTROL_PUSH_THREE),
            pointFromArr(PUSH_THREE)
        )
        .setLinearHeadingInterpolation(Math.toRadians(DRIVE_THREE[2]),
            Math.toRadians(PUSH_THREE[2]))
        .build();

    pickupFirst = robot.follower.pathBuilder()
        .addBezierCurve(
            pointFromArr(PUSH_THREE),
            pointFromArr(CONTROL_PICKUP),
            pointFromArr(PICKUP)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PUSH_THREE[2]),
            Math.toRadians(PICKUP[2]))
        .build();

    place = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(PICKUP),
            pointFromArr(PLACE_SPEC)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PICKUP[2]),
            Math.toRadians(PLACE_SPEC[2]))
        .build();

    pickup = robot.follower.pathBuilder()
        .addBezierLine(
            pointFromArr(PLACE_SPEC),
            pointFromArr(PICKUP)
        )
        .setLinearHeadingInterpolation(Math.toRadians(PLACE_SPEC[2]),
            Math.toRadians(PICKUP[2]))
        .build();

  }

  public void setPathState(int pState) {
    pathState = pState;
    pathTimer.resetTimer();
  }

  public void pickupPlace(PathChain place, PathChain postPlace) {
    robot.waitTime(100);
    robot.claw.clawClose();
    robot.waitTime(50);
    robot.slides.setTarget(VerticalSlides.BAR_PLACE);
    robot.waitTime(200);
    robot.claw.setPlaceAuto();

    robot.follower.followPath(place);
    while (robot.follower.isBusy() && (opMode.opModeIsActive() || !robot.slides.atTarget())) {
      robot.follower.update();
      robot.slides.updatePIDControl();
      robot.horSlide.updatePosition();
      robot.horSlide.updatePIDControl();
    }

    robot.slides.setTarget(VerticalSlides.TRANSFER);

    while ((opMode.opModeIsActive() || !robot.slides.atTarget())) {
      robot.follower.update();
      robot.slides.updatePIDControl();
      robot.horSlide.updatePosition();
      robot.horSlide.updatePIDControl();
    }
    robot.claw.clawOpenWall();
    robot.waitTime(100);
    robot.claw.setWallAuto();
    robot.waitTime(100);

    robot.follower.followPath(postPlace);
  }

  public void autonomousPathUpdate() {
    switch (pathState) {
      case 1000:
        robot.horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
        robot.slides.setTarget(VerticalSlides.TRANSFER);
        pickupPlace(placePreLoad, driveOne);
        setPathState(2000);
        break;
      case 2000:
        if (!robot.follower.isBusy()) {
          robot.follower.followPath(pushOne);
          setPathState(3000);
        }
        break;

      case 3000:
        if (!robot.follower.isBusy()) {
          robot.follower.followPath(pickupFirst);
          setPathState(4000);
        }
        break;

      case 4000:
        if (!robot.follower.isBusy()) {
          pickupPlace(place, pickup);
        }
        break;

      // TODO
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
    robot.follower.setStartingPose(poseFromArr(START));

    while (this.opMode.opModeIsActive()) {
      robot.follower.update();
      robot.slides.updatePIDControl();
      robot.horSlide.updatePosition();
      robot.horSlide.updatePIDControl();

      autonomousPathUpdate();

      telemetry.addData("Path State", pathState);
      telemetry.addData("Position", robot.follower.getPose().toString());
      telemetry.update();
    }
  }
}