package org.firstinspires.ftc.teamcode.auton.roadrunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.odom.MecanumDrive;

// TODO: ENABLE
@Disabled
@Config
@Autonomous(name = "BUCKET ROADRUNNER", group = "ROADRUNNER")
public class BucketRoadRunner extends LinearOpMode {

  // TODO: SET TO CORRECT START POS
  public static Vector2d START_VEC = new Vector2d(10.5, -62);

  public static TranslationalVelConstraint SLOW = new TranslationalVelConstraint(55);
  public static ProfileAccelConstraint SLOW_ACCEL = new ProfileAccelConstraint(-40, 55);

  Robot robot;
  MecanumDrive drive;

  @Override
  public void runOpMode() throws InterruptedException {
    Pose2d START = new Pose2d(START_VEC.x, START_VEC.y, Math.toRadians(270));

    // INIT
    telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

    robot = new Robot(this);
    drive = robot.drive;
    drive.localizer.setPose(START);

    robot.initAuton();
    waitForStart();
    // START

    robot.startSlideUpPos(Robot.VERTICAL_SLIDE_DEFAULT, 0.8);

    // TODO: IMPLEMENT
  }
}
