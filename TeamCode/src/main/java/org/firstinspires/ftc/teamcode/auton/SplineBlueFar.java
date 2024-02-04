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
@Autonomous(name = "Blue Far Spline Autonimous")
public class SplineBlueFar extends LinearOpMode {

  public FtcDashboard dash = FtcDashboard.getInstance();
  Robot robot;
  private VisionPortal portal;
  BluePropThreshold processor;

  public static Vector2d LEFT_BOARD = new Vector2d(52, 51);
  public static Vector2d CENTER_BOARD = new Vector2d(52, 44);

  public static Vector2d RIGHT_BOARD = new Vector2d(52, 37.5);

  @Override
  public void runOpMode() throws InterruptedException {
    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-36, 60, Math.toRadians(-90)));
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
      telemetry.addData("BLUE Prop Position", processor.getElePos());
      telemetry.addData("BLUE left box avg", processor.averagedLeftBox);
      telemetry.addData("BLUE right box avg", processor.averagedRightBox);
      telemetry.update();
    }

    switch (x) {
      case LEFT:
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-29, 34, Math.toRadians(0)), Math.toRadians(0))
                .build());

        robot.spitPixel();
        robot.flipperControl(true);

        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-46, 36, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-54, 8, Math.toRadians(180)),
                    Math.toRadians(180))
                .build());

        robot.turnByGyro(-90);
        robot.flipperControl(false);
        robot.waitTime(1000);

        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-50, 7, Math.toRadians(180)), Math.toRadians(0))
                .build());

        robot.flipperControl(true);
        robot.intake.setPower(0);
        robot.intake.setMode(RunMode.RUN_WITHOUT_ENCODER);
        robot.intake.setPower(1);
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(12, 11))
                .setTangent(0)
                //.waitSeconds(0) // TODO: update per match for partner
                .splineToLinearHeading(new Pose2d(50, 56, Math.toRadians(180)), Math.toRadians(90))
                .strafeTo(LEFT_BOARD)
                .build());

        robot.turnByGyro(-90);
        robot.intake.setPower(0);
        robot.setSlidePos(2400, 1);
        robot.waitTime(200);
        robot.setSlidePos(0, 0.6);
        break;

      case CENTER:
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-36, 16, Math.toRadians(90)), Math.toRadians(270))
                .build());
        robot.spitPixel();
        robot.flipperControl(true);

        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-36, 0))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-54, 8, Math.toRadians(180)),
                    Math.toRadians(180))
                .build());
        robot.turnByGyro(-90);
        robot.flipperControl(false);
        robot.waitTime(1000);
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-50, 7, Math.toRadians(180)), Math.toRadians(0))
                .build());
        robot.flipperControl(true);
        robot.intake.setPower(0);
        robot.intake.setMode(RunMode.RUN_WITHOUT_ENCODER);
        robot.intake.setPower(1);
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(12, 12))
                .setTangent(0)
                //.waitSeconds(0) // TODO: update per match for partner
                .splineToLinearHeading(new Pose2d(50, 56, Math.toRadians(180)), Math.toRadians(90))
                .strafeTo(CENTER_BOARD)
                .build());
        robot.turnByGyro(-90);
        robot.intake.setPower(0);
        robot.setSlidePos(2400, 1);
        robot.waitTime(200);
        robot.setSlidePos(0, 0.6);
        break;
      default:
      case RIGHT:
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-46, 24, Math.toRadians(90)), Math.toRadians(270))
                .build());
        robot.spitPixel();
        robot.flipperControl(true);

        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-46, 0))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-54, 8, Math.toRadians(180)),
                    Math.toRadians(180))
                .build());
        robot.turnByGyro(-90);
        robot.flipperControl(false);
        robot.waitTime(1000);
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-50, 7, Math.toRadians(180)), Math.toRadians(0))
                .build());
        robot.flipperControl(true);
        robot.intake.setPower(0);
        robot.intake.setMode(RunMode.RUN_WITHOUT_ENCODER);
        robot.intake.setPower(1);
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(12, 12))
                .setTangent(0)
                //.waitSeconds(0) // TODO: update per match for partner
                .splineToLinearHeading(new Pose2d(50, 56, Math.toRadians(180)), Math.toRadians(90))
                .strafeTo(RIGHT_BOARD)
                .build());
        robot.turnByGyro(-90);
        robot.intake.setPower(0);
        robot.setSlidePos(2400, 1);
        robot.waitTime(200);
        robot.setSlidePos(0, 0.6);
        break;
    }

  }
}