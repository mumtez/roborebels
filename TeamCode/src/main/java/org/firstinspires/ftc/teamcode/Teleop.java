package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {

  Robot robot;

  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);
    robot.slideR.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.slideL.setMode(RunMode.RUN_WITHOUT_ENCODER);
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    waitForStart();
    // START

    // LOOP
    while (opModeIsActive()) {

      double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
      double x = gamepad1.left_stick_x;
      double rx = gamepad1.right_stick_x;

      // This button choice was made so that it is hard to hit on accident,
      // it can be freely changed based on preference.
      // The equivalent button is start on Xbox-style controllers.
      if (gamepad1.left_bumper) {
        robot.imu.resetYaw();
      }

      double botHeading = AngleUnit.DEGREES.toRadians(robot.getHeading());

      // Rotate the movement direction counter to the bot's rotation
      double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
      double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

      rotX = rotX * 1.1;  // Counteract imperfect strafing

      // Denominator is the largest motor power (absolute value) or 1
      // This ensures all the powers maintain the same ratio,
      // but only if at least one is out of the range [-1, 1]
      double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
      double frontLeftPower = (rotY + rotX + rx) / denominator;
      double backLeftPower = (rotY - rotX + rx) / denominator;
      double frontRightPower = (rotY - rotX - rx) / denominator;
      double backRightPower = (rotY + rotX - rx) / denominator;

      robot.fl.setPower(frontLeftPower);
      robot.bl.setPower(backLeftPower);
      robot.fr.setPower(frontRightPower);
      robot.br.setPower(backRightPower);

      robot.intake.setPower((gamepad1.right_trigger * 0.6) - (gamepad1.left_trigger * 0.5));
      if (robot.intake.getPower() != 0 || gamepad1.x) {
        robot.flipperControl(true);
      } else {
        robot.flipperControl(false);
      }
      robot.setSlidePower(-gamepad2.right_stick_y);

      robot.toggleDoor(gamepad1.y);
      /* if (gamepad1.x && timer2.milliseconds() > 500) {
        robot.flipper();
        timer.reset();
      } */
      if (gamepad2.a && gamepad2.x) {
        robot.fly();
      }

      telemetry.addData("IMU HEADING", AngleUnit.RADIANS.toDegrees(botHeading));
      telemetry.addData("SlideL Pos", robot.slideL.getCurrentPosition());
      telemetry.addData("SlideR Pos", robot.slideR.getCurrentPosition());
      telemetry.update();
    }
  }
}