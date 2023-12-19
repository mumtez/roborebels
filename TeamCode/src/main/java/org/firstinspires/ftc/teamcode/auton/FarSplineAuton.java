package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;
import android.widget.Switch;

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
  public static double startypos = -58;
  public static double startdir = 180;

  public static int spin = 62;

  public static double lypos = 36;
  public static double mypos = 30;
  public static double rypos = 23;

  public static int plxpos = -28;
  public static int pmxpos = -32;
  public static int prxpos = -44;

  public static int plypos = 34;
  public static int pmypos = 30;
  public static int prypos = 34;

  public static int pldir = -95;
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


    switch(x){
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

    Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                    .splineToLinearHeading(new Pose2d(-36, 58, Math.toRadians(-90)),
                            -Math.PI / 2)
                    .build());

robot.setIntakePos(spin-15, 0.3);

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
    robot.intake.setPower(0.8);
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

  */


  }

}

