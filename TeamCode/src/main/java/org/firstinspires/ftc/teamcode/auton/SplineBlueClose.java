package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.odom.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.BluePropThreshold;
import org.firstinspires.ftc.teamcode.vision.Position;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "Blue Close Spline Autonimous")
public class SplineBlueClose extends LinearOpMode {

  public FtcDashboard dash = FtcDashboard.getInstance();
  Robot robot;
  private VisionPortal portal;
  BluePropThreshold processor;
  public static Vector2d placeSpikeLeft = new Vector2d(28, 30);
  public static Vector2d placeSpikeCenter = new Vector2d(18, 24);
  public static Vector2d placeSpikeRight = new Vector2d(6, 30);
  public static Vector2d placeBoardLeft = new Vector2d(49, 38);
  public static Vector2d placeBoardCenter = new Vector2d(49, 31.5);
  public static Vector2d placeBoardRight = new Vector2d(49, 25);

  @Override
  public void runOpMode() throws InterruptedException {
    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, 60, Math.toRadians(-90)));
    processor = new BluePropThreshold();
    robot = new Robot(this);

    portal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        .setCameraResolution(new Size(640, 480))
        .addProcessor(processor)
        .build();

    Position x = Position.NONE;
    while (opModeInInit() && !isStopRequested()) {
      x = processor.getElePos();
      telemetry.addLine("Case" + ":" + x.name());
      telemetry.update();
    }

    Vector2d placeSpike;
    Vector2d placeBoard;
    switch (x) {
      case LEFT:
        placeBoard = placeBoardLeft;
        placeSpike = placeSpikeLeft;
        break;
      case CENTER:
        placeBoard = placeBoardCenter;
        placeSpike = placeSpikeCenter;
        break;
      default:
      case RIGHT:
        placeBoard = placeBoardRight;
        placeSpike = placeSpikeRight;
        break;
    }
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .setTangent(0)
            .strafeToLinearHeading(placeBoard, Math.toRadians(180))
            .build());

    robot.setSlidePos(2200, .8);
    robot.waitTime(1500);
    robot.setSlidePos(0, 1);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .strafeToConstantHeading(placeSpike)
            .build());

    robot.flipperControl(true);
    robot.setIntakePos(-100, .1);
    robot.waitTime(750);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .setTangent(0)
            .splineToConstantHeading(new Vector2d(36, 12), Math.toRadians(180))
            .splineToConstantHeading(new Vector2d(3, 8), Math.toRadians(180))
            .build());
    robot.toggleDoor(true);
    robot.waitTime(500);
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .strafeTo(new Vector2d(-36, 12))
            .build());
    robot.toggleDoor(false);
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .setTangent(Math.toRadians(-180))
            .splineToConstantHeading(new Vector2d(-60, 12), Math.toRadians(-90))
            .build());

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .strafeToConstantHeading(new Vector2d(-65, 12))
            .build());
    robot.flipperControl(false);
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .strafeToConstantHeading(new Vector2d(-60, 12))
            .build());
    robot.flipperControl(true);
    robot.setIntakePos(1000, 1);
    robot.waitTime(1000);

    // second pixel grab
    robot.toggleDoor(false);
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .setTangent(Math.toRadians(-180))
            .splineToConstantHeading(new Vector2d(-60, 12), Math.toRadians(-90))
            .build());

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .strafeToConstantHeading(new Vector2d(-66, 12))
            .build());
    robot.flipperControl(false);
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .strafeToConstantHeading(new Vector2d(-60, 12))
            .build());
    robot.flipperControl(true);
    robot.setIntakePos(1000, 1);
    robot.waitTime(1000);
    // slide shimmy
    robot.slideL.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.slideR.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.setSlidePower(1);
    robot.waitTime(200);
    robot.setSlidePower(-1);
    robot.waitTime(200);
    robot.setSlidePower(0);
    robot.setSlidePos(0, 1);

    // going back to board
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .setTangent(Math.toRadians(0))
            .splineToConstantHeading(new Vector2d(44, 36), Math.toRadians(90))
            .build());

    robot.setSlidePos(3000, 1);
    robot.waitTime(500);
    robot.setSlidePos(0, 1);
    robot.waitTime(2000);
    robot.slideL.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.slideR.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.setSlidePower(1);
    robot.waitTime(200);
    robot.setSlidePower(-1);
    robot.waitTime(200);
    robot.setSlidePower(0);
    robot.setSlidePos(0, 1);
    robot.setSlidePos(3000, 1);
    robot.setSlidePos(0, 1);
  }


}
