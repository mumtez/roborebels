package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.NewRobot;
import org.firstinspires.ftc.teamcode.NewRobot.AllianceColor;

@Config
@TeleOp(name = "TEST TURN", group = "TEST")
public class TurnTest extends LinearOpMode {

  public static double TARGET_DEGREES = 180.0;

  public static double kP = 0.0;
  public static double kD = 0.0;


  NewRobot robot;
  ElapsedTime globalTimer = new ElapsedTime();
  ElapsedTime pidTimer = new ElapsedTime();
  double[] resultTimes = {0, 0, 0};

  @Override
  public void runOpMode() throws InterruptedException {
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    robot = new NewRobot(this, AllianceColor.BLUE);

    waitForStart();
    robot.follower.setStartingPose(new Pose(0, 0, 0));

    while (opModeIsActive()) {
      if (gamepad1.cross) {
        turnByHeading(true);
      }

      if (gamepad1.triangle) {
        turnByHeading(false);
      }

      if (gamepad1.circle) {
        turnByFollower();
      }

      telemetry.addData("IMU TIME", resultTimes[0]);
      telemetry.addData("FOLLOWER TIME", resultTimes[1]);
      telemetry.addData("CUSTOM FOLLOWER TIME", resultTimes[2]);
      telemetry.update();
    }
  }

  void turnByHeading(boolean useIMU) {
    globalTimer.reset();

    robot.fr.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.fl.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.br.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.bl.setMode(RunMode.RUN_WITHOUT_ENCODER);

    int x = 0;
    ElapsedTime timer = new ElapsedTime();
    pidTimer.reset();
    double lastHeadingError = 0;
    // keep looping while we are still active, and not on heading.
    // Max time: 1/2 second
    while (opModeIsActive() && x < 5 && timer.milliseconds() < 500) {

      double curHeading;
      if (useIMU) {
        curHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
      } else {
        robot.follower.update();
        curHeading = robot.follower.getHeadingOffset(); // TODO: could be wrong
      }
      double headingError = TARGET_DEGREES - curHeading;

      // Normalize the error to be within +/- 180 degrees
      while (headingError > 180) {
        headingError -= 360;
      }
      while (headingError <= -180) {
        headingError += 360;
      }

      double dt = pidTimer.seconds();
      double d = (headingError - lastHeadingError) / dt;

      lastHeadingError = headingError;
      pidTimer.reset();

      double turnSpeed = headingError * kP + d * kD;

      robot.fr.setPower(turnSpeed);
      robot.br.setPower(turnSpeed);
      robot.fl.setPower(-turnSpeed);
      robot.bl.setPower(-turnSpeed);

      if (Math.abs(headingError) <= 0.1) {
        x++;
      } else {
        x = 0;
      }

      telemetry.addData("cur", curHeading);
      telemetry.addData("error", headingError);
      telemetry.addData("speed", turnSpeed);
      telemetry.update();
    }

    if (useIMU) {
      resultTimes[0] = globalTimer.milliseconds();
    } else {
      resultTimes[2] = globalTimer.milliseconds();
    }

    robot.fr.setPower(0);
    robot.fl.setPower(0);
    robot.bl.setPower(0);
    robot.br.setPower(0);
  }

  void turnByFollower() {
    globalTimer.reset();

    robot.follower.turnToDegrees(TARGET_DEGREES);
    robot.follower.update();
    while (opModeIsActive() && robot.follower.isTurning()) {
      robot.follower.update();
      telemetry.addLine("=== TURNING BY FOLLOWER ===");
      telemetry.update();
    }
    resultTimes[1] = globalTimer.milliseconds();
  }

}
