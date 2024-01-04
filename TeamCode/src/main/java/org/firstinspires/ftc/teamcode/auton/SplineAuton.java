package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.odom.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.BluePropThreshold;
import org.firstinspires.ftc.teamcode.vision.Position;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "Spline Auton Close")
public class SplineAuton extends LinearOpMode {

  public FtcDashboard dash = FtcDashboard.getInstance();
  public Telemetry dashTel = dash.getTelemetry();

  public static double lxpos = 36;

  public static double mxpos = 30;

  public static double rxpos = 26;

  public static int ypos = 48;

  public static int lypos = 28;
  public static int mypos = 22;
  public static int rypos = 8;

  public static int finalDir = 180;

  Robot robot;
  private VisionPortal portal;
  BluePropThreshold processor;

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

    while (opModeInInit()) {
      telemetry.addData("Location", processor.getElePos());
      telemetry.addData("Left", processor.averagedLeftBox);
      telemetry.addData("Right", processor.averagedRightBox);
      telemetry.addData("Thresh", BluePropThreshold.blueThreshold);
      telemetry.update();
    }

    Position x = processor.getElePos();

    switch (x) {
      case LEFT:
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(ypos, lxpos, Math.toRadians(finalDir)),
                    -Math.PI / 2)
                .strafeToLinearHeading(new Vector2d(0, 0), 0)
                .build());
        break;

      case CENTER:
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(ypos, mxpos, Math.toRadians(finalDir)),
                    -Math.PI / 2)
                .build());
        break;

      case RIGHT:
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(ypos, rxpos, Math.toRadians(finalDir)),
                    -Math.PI / 2)
                .build());
        break;
    }

    robot.setSlidePos(2000, 1);
    sleep(2000);
    robot.setSlidePos(0, 1);

    sleep(1000);

    switch (x) {
      case LEFT:
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(lypos, mxpos),
                    -Math.PI / 2)
                .build());
        break;
      case CENTER:
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(mypos, rxpos),
                    -Math.PI / 2)
                .build());
        break;

      default:
      case RIGHT:
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(rypos, mxpos),
                    -Math.PI / 2)
                .build());
        break;
    }

    robot.spitPixel();
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .splineToLinearHeading(new Pose2d(60, 12, Math.toRadians(finalDir)),
                Math.PI / 2)
            .build());

    /*
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .splineToLinearHeading(new Pose2d(12, 60, Math.toRadians(-90)),
                -Math.PI / 2)
            .build());
     */

    telemetry.addData("Dir", x);
    telemetry.update();

    dashTel.addData("Dir", x);
    dashTel.update();
  }
}