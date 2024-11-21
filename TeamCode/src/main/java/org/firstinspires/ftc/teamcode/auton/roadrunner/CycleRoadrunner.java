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
@Autonomous(name = "CYCLE ROADRUNNER", group = "ROADRUNNER")
public class CycleRoadrunner extends LinearOpMode {

  public static Pose2d START = new Pose2d(-30.5, -62, Math.toRadians(0));

  public static Pose2d bucket = new Pose2d(new Vector2d(-56, -56), Math.toRadians(45));
  public static Vector2d leftBlock = new Vector2d(-48, -23.5);
  public static Pose2d middleBlock = new Pose2d(new Vector2d(-56, -48), Math.toRadians(90));
  public static Pose2d rightBlock = new Pose2d(new Vector2d(-48, -48), Math.toRadians(90));

  public static Pose2d park1 = new Pose2d(new Vector2d(-48, -36), Math.toRadians(180));
  public static Vector2d park2 = new Vector2d(35, -36);
  public static Vector2d park3 = new Vector2d(48, -62);

  public static double turnBlockAngleDeg = 60;

  //public static VelConstraint slowVel = new TranslationalVelConstraint(40);
  //public static AccelConstraint slowAccel = new ProfileAccelConstraint(-40, 40);

  private Robot robot;
  MecanumDrive drive;

  @Override
  public void runOpMode() throws InterruptedException {


    // INIT
    telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

    robot = new Robot(this);
    drive = robot.drive;
    drive.pose = START;
    //drive = new MecanumDrive(hardwareMap, START);

    robot.initAuton();
    waitForStart();
    // START

    //bucket
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
                .setTangent(90)
            .splineToLinearHeading(bucket, Math.toRadians(225))
            .build()
    );

    robot.deposit();

    //right block
    /*
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .splineToLinearHeading(rightBlock, Math.toRadians(260))
            .build()
    );

     */

    pickUpVertical();

    //bucket
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .splineToSplineHeading(bucket, Math.toRadians(200))
            .build()
    );

    robot.deposit();

    //middle block
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .splineToLinearHeading(middleBlock, Math.toRadians(160))
            .build()
    );

    pickUpVertical();

    //bucket
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .splineToSplineHeading(bucket, Math.toRadians(280))
            .build()
    );

    robot.deposit();

    //left block
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .strafeToLinearHeading(leftBlock, Math.toRadians(180))
            .build()
    );

    robot.pickUp();

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

  public void pickUpVertical() {

    //turn right
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .turnTo(Math.toRadians(turnBlockAngleDeg))
            .build()
    );

    //slide out
    robot.setHorizontalSlidePos(Robot.HORIZONTAL_SLIDE_OUT);

    robot.waitTime(1000);

    //intake down
    robot.rotateIntakeOut();

    robot.waitTime(1000);

    //intake on
    robot.intake.setPower(1);

    robot.waitTime(1000);

    //turn left
    Actions.runBlocking(
        drive.actionBuilder(drive.pose)
            .turnTo(Math.toRadians(90))
            .build()
    );

    //wait
    robot.waitTime(1000);

    //intake off
    robot.intake.setPower(0);

    //intake up
    robot.rotateIntakeUp();

    robot.waitTime(500);

    //slide in
    robot.setHorizontalSlidePos(Robot.HORIZONTAL_SLIDE_IN);

    robot.waitTime(500);

    robot.intake.setPower(-1);
  }

}
//mayo
