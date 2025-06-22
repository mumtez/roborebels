package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.NewRobot;
import org.firstinspires.ftc.teamcode.NewRobot.AllianceColor;

@Config
@TeleOp(name = "HORIZ SLIDE PID TESTING", group = "TESTING")
public class HorizSlidePIDTesting extends LinearOpMode {

  public static int TARGET = 0;

  @Override
  public void runOpMode() throws InterruptedException {
    // use dashboard telemetry
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    NewRobot robot = new NewRobot(this, AllianceColor.RED);

    waitForStart();

    while (opModeIsActive()) {
      if (gamepad1.square) {
        robot.horSlide.setTarget(TARGET);
      }

      robot.horSlide.updatePosition();
      double curPow = robot.horSlide.updatePIDControl();

      double y = -gamepad1.left_stick_y;
      double x = gamepad1.left_stick_x;
      double rx = gamepad1.right_stick_x;

      double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

      double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
      double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
      rotX = rotX * 1.1;  // Counteract imperfect strafing

      double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
      double frontLeftPower = (rotY + rotX + rx) / denominator;
      double backLeftPower = (rotY - rotX + rx) / denominator;
      double frontRightPower = (rotY - rotX - rx) / denominator;
      double backRightPower = (rotY + rotX - rx) / denominator;

      robot.fr.setPower(frontRightPower);
      robot.fl.setPower(frontLeftPower);
      robot.br.setPower(backRightPower);
      robot.bl.setPower(backLeftPower);

      telemetry.addData("TARGET", TARGET);
      telemetry.addData("REFERENCE", robot.horSlide.position);
      telemetry.addData("ERROR:", TARGET - robot.horSlide.position);
      telemetry.addData("OUTPUT", curPow);
      telemetry.update();
    }

  }

}
