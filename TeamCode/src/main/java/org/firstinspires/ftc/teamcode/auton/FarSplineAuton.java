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


  //Stack of pixels
  public static double startxpos = 29;
  public static double startypos = -56;
  public static double startdir = 180;

  public static int spin = 50;

  //Board
  public static double lypos = 32;
  public static double mypos = 28;
  public static double rypos = 24;

  public static double boardDir = 180;

  public static double xpos = 55;

  //Spike marker
  public static int plxpos = -33;
  public static int pmxpos = -30;
  public static int prxpos = -44;

  public static int plypos = 34;
  public static int pmypos = 30;
  public static int prypos = 34;

  public static int pldir = 0;
  public static int pmdir = -90;
  public static int prdir = -85;

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

    telemetry.addData("Dir", x);
    telemetry.update();

    dashTel.addData("Dir", x);
    dashTel.update();

    //Start

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .lineToY(12)
            .build());

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .splineToLinearHeading(new Pose2d(-36, 4, Math.toRadians(boardDir)),
                -Math.PI / 2)
            .build());

    sleep(200);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .lineToX(24)
            .build());

    // Past the truss

    switch (x) {
      case LEFT:
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(xpos, lypos, Math.toRadians(boardDir)),
                    -Math.PI / 2)
                .build());
        break;

      case CENTER:
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(xpos, mypos, Math.toRadians(boardDir)),
                    -Math.PI / 2)
                .build());
        break;
      default:
      case RIGHT:
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(xpos, rypos, Math.toRadians(boardDir)),
                    -Math.PI / 2)
                .build());
        break;
    }

    robot.setSlidePos(2000, 1);
    sleep(2000);
    robot.setSlidePos(0, 1);

    sleep(300);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .splineToLinearHeading(new Pose2d(26, 30, Math.toRadians(boardDir)),
                -Math.PI / 2)
            .build());

    sleep(400);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .lineToX(-44)
            .build());

    //Past trust again

    /*
    Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                    .splineToLinearHeading(new Pose2d(-36, 58, Math.toRadians(-90)),
                            -Math.PI / 2)
                    .build());

     */

    switch (x) {
      case LEFT:
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(plxpos, plypos, Math.toRadians(pldir)),
                    -Math.PI / 2)
                .build());
        break;

      case CENTER:
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(pmxpos, pmypos, Math.toRadians(pmdir)),
                    -Math.PI / 2)
                .build());
        break;

      case RIGHT:
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(prxpos, prypos, Math.toRadians(prdir)),
                    -Math.PI / 2)
                .build());
        break;
    }

    robot.intake.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.intake.setPower(-0.2);
    sleep(1000);
    robot.intake.setPower(0);

    robot.setIntakePos(spin - 15, 0.3);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .splineToLinearHeading(new Pose2d(startypos + 1, startxpos, Math.toRadians(startdir)),
                -Math.PI / 2)
            .build());

    robot.setIntakePos(spin, 0.3);
    sleep(1000);

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
    robot.intake.setPower(0.8);
    sleep(1500);
    robot.intake.setPower(0);

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .splineToLinearHeading(new Pose2d(startypos + 3, startxpos, Math.toRadians(-90)),
                -Math.PI / 2)
            .build());

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .lineToY(6)
            .build());

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .splineToLinearHeading(new Pose2d(startypos + 3, 6, Math.toRadians(0)),
                -Math.PI / 2)
            .build());

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .lineToX(24)
            .build());

    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .splineToLinearHeading(new Pose2d(xpos, mypos + 15, Math.toRadians(boardDir)),
                -Math.PI / 2)
            .build());

    robot.setSlidePos(2000, 1);
    sleep(1500);
    robot.setSlidePos(0, 1);

    sleep(300);


  }

}

