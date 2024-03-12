package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.odom.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.BluePropThreshold;
import org.firstinspires.ftc.teamcode.vision.Position;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "Blue Close No Cycle Spline Autonomous")
public class SplineBlueCloseNoCycle extends LinearOpMode {

  public FtcDashboard dash = FtcDashboard.getInstance();
  Robot robot;
  BluePropThreshold processor;

  public static Pose2d BACKDROP_START = new Pose2d(13, 61, Math.toRadians(270));

  public static double BOARD_X = 50;
  public static Vector2d BOARD_LEFT = new Vector2d(BOARD_X, 29);
  public static Vector2d BOARD_CENTER = new Vector2d(BOARD_X, 36);
  public static Vector2d BOARD_RIGHT = new Vector2d(BOARD_X, 43);

  public static Vector2d BOARD_WHITE_LEFT = new Vector2d(BOARD_X - 2, 40);
  public static Vector2d BOARD_WHITE_CENTER = new Vector2d(BOARD_X - 2, 38);
  public static Vector2d BOARD_WHITE_RIGHT = new Vector2d(BOARD_X - 2, 35);

  public static Vector2d SPIKE_LEFT = new Vector2d(7, 35);
  public static Vector2d SPIKE_CENTER = new Vector2d(13, 33);
  public static Vector2d SPIKE_RIGHT = new Vector2d(15.5, 35);

  public static Vector2d PARK_CORNER = new Vector2d(60, 60);
  public static Vector2d PARK_CENTER = new Vector2d(55, 8);

  public static Vector2d AUDIENCE_TRUSS = new Vector2d(-30, 55);
  public static Vector2d BACKDROP_TRUSS = new Vector2d(10.5, 56);
  public static Vector2d NEAR_STACK = new Vector2d(-57, 30);

  public static double STACK_Y = 29.5;
  public static double LEFT_STACK_X = -61;
  public static double CENTER_STACK_X = -61;
  public static double RIGHT_STACK_X = -61;

  public static double SPIKE_LEFT_HEADING = Math.toRadians(180);
  public static double SPIKE_RIGHT_HEADING = Math.toRadians(270);
  public static double SPIKE_CENTER_HEADING = Math.toRadians(225);

  // CONFIGURATION

  public enum PARK_POSITION {
    CENTER, CORNER
  }

  public static PARK_POSITION parkPosition = PARK_POSITION.CENTER; // TODO: change to test all possibilities

  VelConstraint slowVel = new TranslationalVelConstraint(40);
  AccelConstraint slowAccel = new ProfileAccelConstraint(-40, 40);

  @Override
  public void runOpMode() throws InterruptedException {
    MecanumDrive drive = new MecanumDrive(hardwareMap, BACKDROP_START);
    processor = new BluePropThreshold();
    robot = new Robot(this);

    VisionPortal portal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        .setCameraResolution(new Size(640, 480))
        .addProcessor(processor)
        .build();

    Position x = Position.NONE;
    while (opModeInInit() && !isStopRequested()) {
      x = processor.getElePos();
      telemetry.addLine("Case" + ":" + x.name());
      telemetry.addData("RED Prop Position", processor.getElePos());
      telemetry.addData("RED left box avg", processor.averagedLeftBox);
      telemetry.addData("RED right box avg", processor.averagedRightBox);
      telemetry.update();
    }

    Vector2d parkVec;
    switch (parkPosition) {
      case CENTER:
        parkVec = PARK_CENTER;
        break;
      default: // CORNER
        parkVec = PARK_CORNER;
        break;
    }

    Vector2d yellowPlacement;
    Vector2d spikePlacement;
    Vector2d whitePlacement;
    double stackX;
    double spikeHeading;
    switch (x) {
      case CENTER:
        yellowPlacement = BOARD_CENTER;
        spikePlacement = SPIKE_CENTER;
        whitePlacement = BOARD_WHITE_CENTER;
        spikeHeading = SPIKE_CENTER_HEADING;
        stackX = CENTER_STACK_X;
        break;

      case RIGHT:
        yellowPlacement = BOARD_LEFT;
        spikePlacement = SPIKE_LEFT;
        whitePlacement = BOARD_WHITE_LEFT;
        spikeHeading = SPIKE_LEFT_HEADING;
        stackX = LEFT_STACK_X;
        break;

      default:
        yellowPlacement = BOARD_RIGHT;
        spikePlacement = SPIKE_RIGHT;
        whitePlacement = BOARD_WHITE_RIGHT;
        spikeHeading = SPIKE_RIGHT_HEADING;
        stackX = RIGHT_STACK_X;
        break;
    }

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            // Spike
            .splineTo(spikePlacement, spikeHeading)
            // Board
            .setTangent(Math.toRadians(90))
            .splineToConstantHeading(new Vector2d(26, spikePlacement.y - 6), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(yellowPlacement, Math.toRadians(180)),
                Math.toRadians(180))
            .build());
    // TODO: check below for faster
//        drive.actionBuilder(drive.pose)
//            // Spike
//            .splineTo(spikePlacement, spikeHeading)
//            // Board
//            .setReversed(true)
//            .splineTo(yellowPlacement,
//                Math.toRadians(180))
//            .build());
    robot.setSlidePos(1500, 1);
    robot.waitTime(200);
    robot.setSlidePos(0, 1);

    /*
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            // By truss
            .splineToConstantHeading(BACKDROP_TRUSS, Math.toRadians(180))
            .build());
    // intentional break so slowdown
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            //through truss to corner
            .strafeToConstantHeading(AUDIENCE_TRUSS)//, slowVel, slowAccel)
            //near stack
            .splineToConstantHeading(NEAR_STACK, Math.toRadians(90))
            .build());

    robot.flipperControl(true); // open
    //robot.turnByGyro(90);
    robot.waitTime(200);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose).strafeToConstantHeading(new Vector2d(stackX, STACK_Y))
            .build());
    robot.flipperControl(false);
    robot.waitTime(350);
    robot.flipperControl(true);
    robot.waitTime(350);
    robot.flipperControl(false);
    robot.waitTime(350);

    Actions.runBlocking(drive.actionBuilder(drive.pose).lineToX(-58).build());
    robot.flipperControl(true);
    robot.waitTime(200);
    robot.intake.setPower(0);
    robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    robot.intake.setPower(1);
    robot.waitTime(1200);
    robot.intake.setPower(-1);
    robot.waitTime(400);
    robot.intake.setPower(1);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            //back
            .setReversed(true)
            .setTangent(Math.toRadians(270))
            .splineToConstantHeading(AUDIENCE_TRUSS, Math.toRadians(0))
            .build());
    // Intentional stop
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            // Cross truss
            .setReversed(true)
            .strafeToConstantHeading(BACKDROP_TRUSS//, slowVel, slowAccel)
            //board
            .splineToConstantHeading(whitePlacement, Math.toRadians(0))
            .build());

    robot.intake.setPower(0);
    robot.flipperControl(false);
    if (drive.pose.position.x > 40) {
      robot.setSlidePos(1800, 1);
      robot.waitTime(200);
      robot.setSlidePos(0, 1);
      */
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            // park
            .splineToConstantHeading(parkVec, Math.toRadians(0))
            .build());
  }


}