package org.firstinspires.ftc.teamcode.auton.roadrunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.odom.MecanumDrive;


@Config
@Autonomous(name = "BASIC ROADRUNNER", group = "ROADRUNNER")
public class BasicRoadrunner extends LinearOpMode {

  public static Pose2d START = new Pose2d(-10.5, -62, Math.toRadians(90));

  public static Pose2d bucket = new Pose2d(new Vector2d(-54, -54), Math.toRadians(225));

  public static Pose2d park1 = new Pose2d(new Vector2d(-48, -36), Math.toRadians(180));
  public static Vector2d park2 = new Vector2d(35, -36);
  public static Vector2d park3 = new Vector2d(48, -62);

  Robot robot;
  MecanumDrive drive;

  @Override
  public void runOpMode() throws InterruptedException {
    // INIT
    telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

    robot = new Robot(this);
    drive = robot.drive;
    drive.pose = START;

    waitForStart();
    //START

    //bucket
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .splineToSplineHeading(bucket, Math.toRadians(260))
            .build()
    );

    robot.deposit();

    //park
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .splineToSplineHeading(park1, Math.toRadians(80))
            .waitSeconds(.5)

            .strafeToLinearHeading(park2, Math.toRadians(180))
            .waitSeconds(.5)

            .strafeToLinearHeading(park3, Math.toRadians(180))
            .build()

    );

  }
}