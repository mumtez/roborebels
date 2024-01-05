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
@Autonomous(name = "Red Far Spline Autonimous")
public class SplineRedFar extends LinearOpMode {

  public FtcDashboard dash = FtcDashboard.getInstance();
  Robot robot;
  private VisionPortal portal;
  RedPropThreshold processor;

  @Override
  public void runOpMode() throws InterruptedException {
    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, 60, Math.toRadians(-90)));
    processor = new RedPropThreshold();
    robot = new Robot(this);

    portal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        .setCameraResolution(new Size(640, 480))
        .addProcessor(processor)
        .build();

    Position x = processor.getElePos();
    waitForStart();
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .strafeTo(new Vector2d(-36, -34))
            .setTangent(-90)
            .splineToLinearHeading(new Pose2d(-55, -36, Math.toRadians(180)), Math.toRadians(-180))
            .setTangent(-0)
            .splineToLinearHeading(new Pose2d(-36, -61, Math.toRadians(180)), Math.toRadians(-0))
            .strafeTo(new Vector2d(36, -60))
            .setTangent(0)
            .splineToLinearHeading(new Pose2d(50, -36, Math.toRadians(180)), Math.toRadians(-0))
            .setTangent(90)
            .splineToSplineHeading(new Pose2d(60, -12, Math.toRadians(0)), Math.toRadians(-0))

            .build());
  }
}
