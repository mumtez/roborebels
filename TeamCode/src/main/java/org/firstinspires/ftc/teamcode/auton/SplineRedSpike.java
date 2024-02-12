package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.odom.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.Position;
import org.firstinspires.ftc.teamcode.vision.RedPropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "Red Far Spike Spline Autonimous")
public class SplineRedSpike extends LinearOpMode {

  public FtcDashboard dash = FtcDashboard.getInstance();
  Robot robot;
  private VisionPortal portal;
  RedPropThreshold processor;

  public static Vector2d RIGHT_BOARD = new Vector2d(55.5, -51 + 17);
  public static Vector2d CENTER_BOARD = new Vector2d(55.5, -44);

  public static Vector2d LEFT_BOARD = new Vector2d(55.5, -37.5);

  @Override
  public void runOpMode() throws InterruptedException {
    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-36, -61, Math.toRadians(90)));
    processor = new RedPropThreshold();
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
      telemetry.addData("RED Prop Position", processor.getElePos());
      telemetry.addData("RED left box avg", processor.averagedLeftBox);
      telemetry.addData("RED right box avg", processor.averagedRightBox);
      telemetry.update();
    }

    switch (x) {
      case RIGHT:
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-29, -34, Math.toRadians(0)), Math.toRadians(0))
                .build());

        robot.spitPixel();
        robot.flipperControl(true);

        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-46, -36, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-55, -8, Math.toRadians(180)),
                    Math.toRadians(180))
                .build());
        break;

      case CENTER:
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-36, -16, Math.toRadians(-90)),
                    Math.toRadians(90))
                .build());
        robot.spitPixel();
        robot.flipperControl(true);

        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-36, -0))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-55, -8, Math.toRadians(180)),
                    Math.toRadians(180))
                .build());

        break;

      default:
      case LEFT:
        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-46, -24, Math.toRadians(-90)),
                    Math.toRadians(90))
                .build());
        robot.spitPixel();
        robot.flipperControl(true);

        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-46, -0))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-55, -8, Math.toRadians(180)),
                    Math.toRadians(180))
                .build());

        break;
    }

  }
}