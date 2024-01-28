package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "Tune Turn", group = "TESTING")
public class TuneTurning extends LinearOpMode {

  Robot robot;
  public static int TARGET = 90;
  public static int THRESHOLD = 1;

  public static double P = 0.025;

  public static double MAX_SPEED = 0.6;

  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);

    robot.fr.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.fl.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.br.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.bl.setMode(RunMode.RUN_WITHOUT_ENCODER);

    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    waitForStart();

    double headingError = TARGET - robot.getHeading();
    // Normalize the error to be within +/- 180 degrees
    while (headingError > 180) {
      headingError -= 360;
    }
    while (headingError <= -180) {
      headingError += 360;
    }

    // keep looping while we are still active, and not on heading.
    while (opModeIsActive()) {

      headingError = TARGET - robot.getHeading();

      // Normalize the error to be within +/- 180 degrees
      while (headingError > 180) {
        headingError -= 360;
      }
      while (headingError <= -180) {
        headingError += 360;
      }

      double turnSpeed = Range.clip(headingError * P, -1, 1);

      // Clip the speed to the maximum permitted value.
      turnSpeed = Range.clip(turnSpeed, -MAX_SPEED, MAX_SPEED);
      robot.setDriveTrainPower(turnSpeed, -turnSpeed, turnSpeed, -turnSpeed);

      telemetry.addData("error", headingError);
      telemetry.addData("speed", turnSpeed);
      telemetry.addData("IN THRESH", headingError < THRESHOLD);
      telemetry.update();
    }
  }
}
