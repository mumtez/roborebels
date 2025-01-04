package org.firstinspires.ftc.teamcode.auton.roadrunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
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

  public static Vector2d bar = new Vector2d(2, -32);
  public static Vector2d bar2 = new Vector2d(1, -32);
  public static Vector2d bar3 = new Vector2d(1, -32);
  public static Vector2d bar4 = new Vector2d(1, -32);
  public static Vector2d bar5 = new Vector2d(1, -32);

  public static Vector2d leftBlock = new Vector2d(41, -18);
  public static Vector2d middleBlock = new Vector2d(49, -18);
  public static Vector2d rightBlock = new Vector2d(56.5, -18);

  public static Vector2d wallPickup = new Vector2d(42, -63);

  public static TranslationalVelConstraint SLOW = new TranslationalVelConstraint(55);
  public static ProfileAccelConstraint SLOW_ACCEL = new ProfileAccelConstraint(-40, 55);

  Robot robot;
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

    Actions.runBlocking(
        drive.actionBuilder(drive.localizer.getPose())
            .setTangent(Math.toRadians(90))
            .splineToConstantHeading(bar, Math.toRadians(90))
            .build()
    );

    Actions.runBlocking(
        drive.actionBuilder(drive.localizer.getPose())
            .afterTime(0, () -> {
              robot.claw.setPlace();
            })
            .setTangent(Math.toRadians(270))
            .splineToConstantHeading(new Vector2d(31, -40), Math.toRadians(90))
            .afterTime(0.1, () -> {
              robot.claw.clawOpen();
            })

            // AWAY PLACE
            .setTangent(Math.toRadians(90))
            .splineToConstantHeading(new Vector2d(32, leftBlock.y + 5), Math.toRadians(0))

            // LEFT BLOCK
            .setTangent(Math.toRadians(0))
            .splineToConstantHeading(leftBlock, Math.toRadians(0), SLOW, SLOW_ACCEL)
            .setTangent(Math.toRadians(270))
            .splineToConstantHeading(new Vector2d(leftBlock.x, wallPickup.y + 10), Math.toRadians(270))

            // MIDDLE
            .setTangent(Math.toRadians(90))
            .splineToConstantHeading(new Vector2d(leftBlock.x, middleBlock.y + 5), Math.toRadians(0))
            .setTangent(0)
            .splineToConstantHeading(middleBlock, Math.toRadians(0), SLOW, SLOW_ACCEL)
            .setTangent(Math.toRadians(270))
            .splineToConstantHeading(new Vector2d(middleBlock.x, wallPickup.y + 10), Math.toRadians(270))

            // RIGHT
//            .setTangent(Math.toRadians(90))
//            .splineToConstantHeading(new Vector2d(middleBlock.x, rightBlock.y + 5), Math.toRadians(0))
//            .setTangent(0)
//            .splineToConstantHeading(rightBlock, Math.toRadians(0), SLOW, SLOW_ACCEL)
//            .splineToConstantHeading(new Vector2d(rightBlock.x, wallPickup.y + 10), Math.toRadians(90))

            // ALIGN WALL
            //.splineToConstantHeading(new Vector2d(wallPickup.x, wallPickup.y + 18), Math.toRadians(90))
            .build()
    );

    getWall();
    Actions.runBlocking(
        drive.actionBuilder(drive.localizer.getPose())
            .setTangent(Math.toRadians(90))
            .splineToConstantHeading(bar2, Math.toRadians(90))
            .build()
    );
    placeBar();

    getWall();
    Actions.runBlocking(
        drive.actionBuilder(drive.localizer.getPose())
            .setTangent(Math.toRadians(90))
            .splineToConstantHeading(bar3, Math.toRadians(90))
            .build()
    );
    placeBar();

    getWall();
    Actions.runBlocking(
        drive.actionBuilder(drive.localizer.getPose())
            .setTangent(Math.toRadians(90))
            .splineToConstantHeading(bar4, Math.toRadians(90))
            .build()
    );
    placeBar();

    Actions.runBlocking(
        drive.actionBuilder(drive.localizer.getPose())
            .afterTime(0, () -> {
              robot.claw.setPlace();
            })
            .setTangent(Math.toRadians(270))
            .splineToConstantHeading(new Vector2d(wallPickup.x, wallPickup.y), Math.toRadians(270))
            .afterTime(0.1, () -> {
              robot.claw.clawOpen();
            })
            .build()
    );

//    getWall();
//    Actions.runBlocking(
//        drive.actionBuilder(drive.localizer.getPose())
//            .setTangent(Math.toRadians(90))
//            .splineToConstantHeading(bar5, Math.toRadians(90))
//            .build()
//    );
//    placeBar();

  }


  public void placeBar() {

    Actions.runBlocking(
        drive.actionBuilder(drive.localizer.getPose())
            .afterTime(0, () -> {
              robot.claw.setPlace();
            })
            .setTangent(Math.toRadians(270))
            .splineToConstantHeading(new Vector2d(wallPickup.x, wallPickup.y + 18), Math.toRadians(270))
            .afterTime(0.1, () -> {
              robot.claw.clawOpen();
            })
            .build()
    );
  }

  public void getWall() {
    robot.claw.clawOpen();
    robot.claw.setWall();

    Actions.runBlocking(
        drive.actionBuilder(drive.localizer.getPose())
            .setTangent(Math.toRadians(270))
            .splineToConstantHeading(new Vector2d(wallPickup.x, wallPickup.y + 18), Math.toRadians(270))
            .waitSeconds(0.1)
            .splineToConstantHeading(wallPickup, Math.toRadians(270), SLOW, SLOW_ACCEL)
            .build()
    );

    robot.claw.clawClose();
    robot.waitTime(200);
    robot.claw.setUnder();
  }

}
//mayo
