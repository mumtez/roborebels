package org.firstinspires.ftc.teamcode.auton.roadrunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.odom.MecanumDrive;


@Config
@Autonomous(name = "SPECIMEN ROADRUNNER", group = "ROADRUNNER")
public class SpecimenRoadrunner extends LinearOpMode {

  public static Vector2d bar = new Vector2d(0, -35);
  public static Vector2d leftBlock = new Vector2d(42, -5);
  public static Vector2d middleBlock = new Vector2d(54, -5);
  public static Vector2d rightBlock = new Vector2d(56, -5);

  public static Vector2d wallPickup = new Vector2d(42, -54);

  public static TranslationalVelConstraint SLOW = new TranslationalVelConstraint(30);

  //public static VelConstraint slowVel = new TranslationalVelConstraint(40);
  //public static AccelConstraint slowAccel = new ProfileAccelConstraint(-40, 40);

  private Robot robot;
  MecanumDrive drive;

  @Override
  public void runOpMode() throws InterruptedException {

    Pose2d START = new Pose2d(10.5, -62, Math.toRadians(270));

    // INIT
    telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

    robot = new Robot(this);
    drive = robot.drive;
    drive.localizer.setPose(START);

    robot.initAuton();
    waitForStart();
    // START

    robot.startSlideUpPos(Robot.VERTICAL_SLIDE_DEFAULT, 0.8);
    robot.claw.setUnder();

    robot.waitTime(300);

    Actions.runBlocking(
        drive.actionBuilder(drive.localizer.getPose())
            .setTangent(Math.toRadians(90))
            .splineToConstantHeading(bar, Math.toRadians(90))
            .build()
    );

    placeBar();

    //left block

    Actions.runBlocking(
        drive.actionBuilder(drive.localizer.getPose())
            .strafeToConstantHeading(new Vector2d(29, -52))
            .strafeToConstantHeading(new Vector2d(29, -5))
            .setTangent(Math.toRadians(90))
            .splineToConstantHeading(leftBlock, Math.toRadians(270))
            .setTangent(Math.toRadians(270))
            .splineToConstantHeading(new Vector2d(42, -50), Math.toRadians(270))
            .setTangent(Math.toRadians(90))
            .splineToConstantHeading(new Vector2d(42, -5), Math.toRadians(90))
            .build()
    );

    Actions.runBlocking(
        drive.actionBuilder(drive.localizer.getPose())
            .splineToConstantHeading(middleBlock, Math.toRadians(160))
            .splineToConstantHeading(new Vector2d(54, -47), Math.toRadians(90))
            .splineToConstantHeading(new Vector2d(50, -5), Math.toRadians(90), SLOW)
            .build()
    );

    Actions.runBlocking(
        drive.actionBuilder(drive.localizer.getPose())
            .splineToConstantHeading(rightBlock, Math.toRadians(180), SLOW)
            .strafeToConstantHeading(new Vector2d(56, -47), SLOW)
            .strafeToConstantHeading(new Vector2d(wallPickup.x, wallPickup.y + 10), SLOW)
            .build());
    robot.claw.clawOpen();
    robot.claw.setWall();
    robot.waitTime(3000);
    Actions.runBlocking(
        drive.actionBuilder(drive.localizer.getPose())
            .strafeToConstantHeading(wallPickup)
            .build()
    );
    robot.claw.clawClose();
    robot.waitTime(1000);
    robot.claw.setUnder();

    robot.waitTime(1000);

    Actions.runBlocking(
        drive.actionBuilder(drive.localizer.getPose())
            .setTangent(Math.toRadians(90))
            .splineToConstantHeading(bar, Math.toRadians(90))
            .build()
    );

    placeBar();

  }

  public void placeBar() {
    robot.startSlideUpPos(Robot.VERTICAL_SLIDE_DEFAULT, 0.8);

    Actions.runBlocking(
        drive.actionBuilder(drive.localizer.getPose())
            .strafeTo(new Vector2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y + 3))
            .build()
    );

    robot.claw.setPlace();

    Actions.runBlocking(
        drive.actionBuilder(drive.localizer.getPose())
            .strafeTo(new Vector2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y - 10))
            .build()
    );

    robot.claw.clawOpen();

    //robot.claw.setDefault();
  }

}
//mayo
