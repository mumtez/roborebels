package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.odom.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.BluePropThreshold;
import org.firstinspires.ftc.teamcode.vision.Position;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "Spline Auton Far")
public class FarSplineAuton extends LinearOpMode {

  public FtcDashboard dash = FtcDashboard.getInstance();
  public Telemetry dashTel = dash.getTelemetry();

  public static double startxpos = 31;
  public static double startypos = -55;
  public static double startdir = 180;
  public static int spin = 125;
  public static double lxpos = 36;

  public static double mxpos = 30;

  public static double rxpos = 23;

  public static int ypos = 41;

  public static int lypos = 28;
  public static int mypos = 22;
  public static int rypos = 8;

  public static int finalDir = 180;

  Robot robot;
  private VisionPortal portal;
  BluePropThreshold processor;

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

    while (opModeInInit()) {
      telemetry.addData("Location", processor.getElePos());
      telemetry.addData("Left", processor.averagedLeftBox);
      telemetry.addData("Right", processor.averagedRightBox);
      telemetry.addData("Thresh", BluePropThreshold.blueThreshold);
      telemetry.update();
    }

    Position x = processor.getElePos();

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .splineToLinearHeading(new Pose2d(-36, 12, Math.toRadians(startdir)),
                -Math.PI / 2)
            .build());

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .splineToLinearHeading(new Pose2d(startypos + 1, startxpos, Math.toRadians(startdir)),
                -Math.PI / 2)
            .build());

    robot.setIntakePos(spin, 0.3);
    sleep(1500);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .lineToX(startypos + 8)
            .build());

    sleep(200);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .lineToX(startypos + 3)
            .build());

    robot.intake.setMode(RunMode.RUN_USING_ENCODER);
    robot.intake.setPower(0.7);
    sleep(2000);
    robot.intake.setPower(0);




    /*
    switch (x) {
      case LEFT:
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(ypos, lxpos, Math.toRadians(finalDir)),
                    -Math.PI / 2)
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
                .splineToConstantHeading(new Vector2d(mypos, rxpos - 3),
                    -Math.PI / 2)
                .build());
        break;

      case RIGHT:
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(rypos, mxpos),
                    -Math.PI / 2)
                .build());
        break;
    }

    robot.spitPixel();
    sleep(1500);


    telemetry.addData("Dir", x);
    telemetry.update();

    dashTel.addData("Dir", x);
    dashTel.update();



     */
  }

}

