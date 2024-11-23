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
@Autonomous(name = "BUCKET ROADRUNNER", group = "ROADRUNNER")
public class BasicRoadrunner extends LinearOpMode {


  //public static VelConstraint slowVel = new TranslationalVelConstraint(40);
  //public static AccelConstraint slowAccel = new ProfileAccelConstraint(-40, 40);

  public static Vector2d bucketVec = new Vector2d (-56, -56);

  public static Vector2d park1Vec = new Vector2d(-48, -36);
  public static Vector2d park2Vec;
  public static Vector2d park3Vec;

  private Robot robot;
  MecanumDrive drive;

  @Override
  public void runOpMode() throws InterruptedException {

    Pose2d START = new Pose2d(-30.5, -62, Math.toRadians(0));

    Pose2d bucket = new Pose2d(bucketVec, Math.toRadians(45));

    Pose2d park1 = new Pose2d(park1Vec, Math.toRadians(180));
    Vector2d park2 = new Vector2d(35, -36);
    Vector2d park3 = new Vector2d(48, -62);

    // INIT
    telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

    robot = new Robot(this);
    drive = robot.drive;
    drive.pose = START;

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


    //TODO CHECK IF PARKING WORKS


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
