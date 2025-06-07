package org.firstinspires.ftc.teamcode.auton.baseAutons;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import org.firstinspires.ftc.teamcode.NewRobot;
import org.firstinspires.ftc.teamcode.NewRobot.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlides;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides;

@Config
@Autonomous(name = "TESTCHATAUTO", group = "BaseAutons")
public class TESTCHATAUTO extends LinearOpMode {

  public static int HSLIDE_1 = HorizontalSlides.OUT_POS / 4;
  public static int HSLIDE_2 = HorizontalSlides.OUT_POS / 3;
  public static int HSLIDE_3 = HorizontalSlides.OUT_POS / 3;
  public static double INTAKE_TIMEOUT = 4;
  public static int PLACE_INITIAL_DELAY = 350;
  public static int PLACE_RELEASE_DELAY = 75;

  public static double[] START = {9.5, 105, 270};
  public static double[] SAFE_PLACE = {16, 126, 315};
  public static double[] PLACE = {15, 129, 315};
  public static double[] IN_1 = {19, 124, 360};
  public static double[] IN_2 = {19, 130, 360};
  public static double[] IN_3 = {28, 125, 50};

  PathChain toSafe, in1, place1, in2, place2, in3, place3, park;
  private int state = 0;
  private Timer timer = new Timer();
  private NewRobot robot;

  @Override
  public void runOpMode() {
    robot = new NewRobot(this, AllianceColor.BLUE);
    definePaths();
    robot.initAuton();

    // init loop
    while (opModeInInit()) {
      telemetry.addData("ALLIANCE", robot.getAllianceColor());
      telemetry.update();
    }

    robot.follower.setStartingPose(pose(START));
    robot.slides.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.slides.setTarget(VerticalSlides.UP_AUTO);
    robot.horSlide.setTarget(HSLIDE_1);

    waitForStart();

    while (opModeIsActive()) {
      robot.updateAutoControls();
      updateStateMachine();
      telemetry.addData("State", state);
      telemetry.update();
    }
  }

  private void updateStateMachine() {
    switch (state) {
      case 0:
        robot.follower.followPath(toSafe, true);
        next(1);
        break;
      case 1:
        if (!robot.follower.isBusy() && robot.slides.atTarget()) {
          robot.claw.setBucket();
          next(2);
        }
        break;
      case 2:
        intakeAndPlace(in1, place1, HSLIDE_2, 3);
        break;
      case 3:
        intakeAndPlace(in2, place2, HSLIDE_3, 4);
        break;
      case 4:
        intakeAndPlace(in3, place3, HSLIDE_3, 5);
        break;
      case 5:
        if (!robot.follower.isBusy()) {
          park();
          next(6);
        }
        break;
      default:
        break;
    }
  }

  private void intakeAndPlace(PathChain intakePath, PathChain placePath, int hSlidePos, int nextState) {
    if (timer.getElapsedTimeSeconds() == 0) {
      robot.intake.update(1, false, robot.getAllianceColor());
      robot.follower.followPath(intakePath, true);
    }
    boolean got = robot.intake.validSampleIn(robot.getAllianceColor());
    if (got || (!robot.follower.isBusy() && timer.getElapsedTimeSeconds() > INTAKE_TIMEOUT)) {
      robot.intake.update(0, true, robot.getAllianceColor());
      // transfer positions
      robot.horSlide.setTarget(hSlidePos);
      robot.slides.setTarget(VerticalSlides.UP_AUTO);
      while (opModeIsActive() && (!robot.horSlide.atTarget() || !robot.slides.atTarget())) {
        robot.updateAutoControls();
      }
      // place sequence
      placeSequence(placePath);
      next(nextState);
    }
  }

  private void placeSequence(PathChain path) {
    Timer t = new Timer();
    while (opModeIsActive() && t.getElapsedTime() < PLACE_INITIAL_DELAY) {
      robot.updateAutoControls();
    }
    robot.claw.clawOpen();
    t.resetTimer();
    while (opModeIsActive() && t.getElapsedTime() < PLACE_RELEASE_DELAY) {
      robot.updateAutoControls();
    }
    robot.claw.setTransfer();
    robot.slides.setTarget(VerticalSlides.TRANSFER);
    robot.follower.followPath(path, true);
  }

  private void park() {
    placeSequence(park);
  }

  private void next(int s) {
    state = s;
    timer.resetTimer();
  }

  private void definePaths() {
    toSafe = robot.follower.pathBuilder()
        .addBezierLine(point(START), point(SAFE_PLACE))
        .setLinearHeadingInterpolation(Math.toRadians(START[2]), Math.toRadians(SAFE_PLACE[2]))
        .build();
    in1 = robot.follower.pathBuilder()
        .addBezierLine(point(SAFE_PLACE), point(IN_1))
        .build();
    place1 = robot.follower.pathBuilder()
        .addBezierLine(point(IN_1), point(PLACE))
        .build();
    in2 = robot.follower.pathBuilder()
        .addBezierLine(point(PLACE), point(IN_2))
        .build();
    place2 = robot.follower.pathBuilder()
        .addBezierLine(point(IN_2), point(PLACE))
        .build();
    in3 = robot.follower.pathBuilder()
        .addBezierLine(point(PLACE), point(IN_3))
        .build();
    place3 = robot.follower.pathBuilder()
        .addBezierLine(point(IN_3), point(PLACE))
        .build();
    park = robot.follower.pathBuilder()
        .addBezierLine(point(PLACE), point(START))
        .build();
  }

  private Point point(double[] a) {
    return new Point(a[0], a[1]);
  }

  private Pose pose(double[] a) {
    return new Pose(a[0], a[1], Math.toRadians(a[2]));
  }
}
