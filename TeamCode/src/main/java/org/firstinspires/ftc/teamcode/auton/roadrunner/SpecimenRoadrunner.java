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
@Autonomous(name = "SPECIMEN ROADRUNNER", group = "ROADRUNNER")
public class SpecimenRoadrunner extends LinearOpMode {

  public static Vector2d barVec = new Vector2d(0, -35);
  public static double barTan = Math.toRadians(0);

  //public static VelConstraint slowVel = new TranslationalVelConstraint(40);
  //public static AccelConstraint slowAccel = new ProfileAccelConstraint(-40, 40);

  private Robot robot;
  MecanumDrive drive;

  @Override
  public void runOpMode() throws InterruptedException {

    Pose2d START = new Pose2d(10.5, -62, Math.toRadians(270));

    Pose2d bar = new Pose2d(new Vector2d(0, -35), Math.toRadians(270));
    Pose2d leftBlock = new Pose2d(new Vector2d(62, -5), Math.toRadians(270));
    Pose2d middleBlock = new Pose2d(new Vector2d(54, -5), Math.toRadians(270));
    Pose2d rightBlock = new Pose2d(new Vector2d(46, -5), Math.toRadians(270));

    Pose2d wallPickup = new Pose2d( new Vector2d(30, 60), Math.toRadians(270));
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

    robot.waitTime(400);


    Actions.runBlocking(
        drive.actionBuilder(drive.localizer.getPose())
            .splineToLinearHeading(bar, Math.toRadians(180))
            .build()
    );

    placeBar();

    //right block

    Actions.runBlocking(
        drive.actionBuilder(drive.localizer.getPose())
            .splineToSplineHeading(rightBlock, Math.toRadians(45))
                .splineToLinearHeading(new Pose2d( new Vector2d(42, -55), Math.toRadians(270)),90 )
                .splineToLinearHeading(new Pose2d( new Vector2d(42, -5), Math.toRadians(270)),90)
            .build()
    );

    robot.waitTime(300);

    Actions.runBlocking(
            drive.actionBuilder(drive.localizer.getPose())
                    .splineToLinearHeading(middleBlock, Math.toRadians(160))

                    .splineToLinearHeading(new Pose2d( new Vector2d(54, -55), Math.toRadians(270)),90 )

                    .splineToLinearHeading(new Pose2d( new Vector2d(54, -5), Math.toRadians(270)),90)
                    .build()
    );

    robot.waitTime(300);

    Actions.runBlocking(
            drive.actionBuilder(drive.localizer.getPose())
                    .splineToLinearHeading(leftBlock, Math.toRadians(180))

                    .strafeTo(new Vector2d(62, -55))
                    .waitSeconds(.2)
                    .build()
    );


  }

  public void placeBar() {
    //robot.setHorizontalSlidePos(Robot.HORIZONTAL_SLIDE_TRANSFER + 0.08);
    //robot.waitTime(200);
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

  private void pickUp() {
    robot.setHorizontalSlidePos(Robot.HORIZONTAL_SLIDE_OUT / 2);
    robot.waitTime(400);

    robot.rotateIntakeOut();
    robot.intake.setPower(1);

    robot.setHorizontalSlidePos(Robot.HORIZONTAL_SLIDE_OUT);

    robot.waitTime(400);

    Actions.runBlocking(
        drive.actionBuilder(drive.localizer.getPose())
            .strafeTo(new Vector2d(drive.localizer.getPose().position.x - 3, drive.localizer.getPose().position.y))
            .build()
    );

    robot.waitTime(400);

    robot.intake.setPower(0);
    robot.rotateIntakeUp();

    robot.waitTime(700);

    robot.setHorizontalSlidePos(Robot.HORIZONTAL_SLIDE_TRANSFER);
    robot.waitTime(500);

    robot.intake.setPower(-1);
    robot.waitTime(500);

    robot.rotateIntakeUp();
  }

}
//mayo
