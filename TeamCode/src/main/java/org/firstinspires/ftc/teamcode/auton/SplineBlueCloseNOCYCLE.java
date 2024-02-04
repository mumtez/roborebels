package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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
@Autonomous(name = "Blue Close Spline Autonimous NO CYCLE")
public class SplineBlueCloseNOCYCLE extends LinearOpMode {

  Robot robot;
  private VisionPortal portal;
  BluePropThreshold processor;
  public static Vector2d placeSpikeLeft = new Vector2d(27, 30);
  public static Vector2d placeSpikeCenter = new Vector2d(18, 24);
  public static Vector2d placeSpikeRight = new Vector2d(6, 30);
  public static Vector2d placeBoardLeft = new Vector2d(49, 38);
  public static Vector2d placeBoardCenter = new Vector2d(48.5, 31.5);
  public static Vector2d placeBoardRight = new Vector2d(48.5, 27);

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

    // READ POSITION
    Position x = Position.NONE;
    while (opModeInInit() && !isStopRequested()) {
      x = processor.getElePos();
      telemetry.addLine("Case" + ":" + x.name());
      telemetry.addData("BLUE Prop Position", processor.getElePos());
      telemetry.addData("BLUE left box avg", processor.averagedLeftBox);
      telemetry.addData("BLUE right box avg", processor.averagedRightBox);
      telemetry.update();
    }

    // START
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

    // PLACE BOARD
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .setTangent(0)
            .strafeToLinearHeading(placeBoard, Math.toRadians(180))
            .build());

    robot.setSlidePos(2200, 1);
    robot.setSlidePos(0, 1);

    // PLACE SPIKE
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .strafeToConstantHeading(placeSpike)
            .build());

    robot.flipperControl(true);
    robot.setIntakePos(-100, .1);
    robot.waitTime(100);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .setTangent(0)
            .splineToConstantHeading(new Vector2d(60, 60), Math.toRadians(0))
            .build());

    /* DRIVE TO GATE
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .setTangent(0)
            .splineToConstantHeading(new Vector2d(24, 0), Math.toRadians(180))
            .splineToConstantHeading(new Vector2d(3, 8), Math.toRadians(180))
            .build());
    robot.toggleDoor(true);
    robot.waitTime(400);

    // GO THROUGH GATE
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
            .strafeToConstantHeading(new Vector2d(-66.5, 12))
            .build());

    // GRAB ONE
    robot.flipperControl(false);
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .strafeToConstantHeading(new Vector2d(-60, 12))
            .build());
    robot.flipperControl(true);
    robot.intake.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.intake.setPower(0.6);
    robot.waitTime(600);
    robot.intake.setPower(0);

    // SHIMMY
    robot.slideL.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.slideR.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.setSlidePower(1);
    robot.waitTime(200);
    robot.setSlidePower(-1);
    robot.waitTime(200);
    robot.setSlidePos(0, 1);

    // GRAB TWO
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .setTangent(Math.toRadians(-180))
            .splineToConstantHeading(new Vector2d(-60, 12), Math.toRadians(-90))
            .build());

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .strafeToConstantHeading(new Vector2d(-66.5, 12))
            .build());
    robot.flipperControl(false);
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .strafeToConstantHeading(new Vector2d(-60, 12))
            .build());
    robot.flipperControl(true);
    robot.intake.setPower(.6);
    robot.waitTime(600);
    robot.intake.setPower(0);
    robot.flipperControl(false);

    // going back to board
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .setTangent(Math.toRadians(0))
            .splineToConstantHeading(new Vector2d(45, 25), Math.toRadians(90))
            .turnTo(Math.toRadians(180))
            .build());

    // FIRST PLACE
    robot.setSlidePos(3000, 1);
    robot.setSlidePos(0, 1);

    // JIGGLE
    robot.setSlidePower(0);
    robot.slideL.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.slideR.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.setSlidePower(1);
    robot.waitTime(200);
    robot.setSlidePower(-1);
    robot.waitTime(200);
    robot.setSlidePos(0, 1);

    // SECOND PLACE
    robot.setSlidePos(3000, 1);
    robot.setSlidePos(0, 1); */
  }


}
