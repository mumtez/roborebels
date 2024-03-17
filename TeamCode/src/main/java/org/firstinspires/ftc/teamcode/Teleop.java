package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {

  Robot robot;
  private boolean gateClosed = true;
  private boolean yHeld = false;

  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);
    robot.slideR.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.slideL.setMode(RunMode.RUN_WITHOUT_ENCODER);
    robot.intake.setMode(RunMode.RUN_WITHOUT_ENCODER);
    waitForStart();
    // START

    // LOOP
    while (opModeIsActive()) {

      if (gamepad1.left_bumper) {
        robot.imu.resetYaw();
      }

      double y = -gamepad1.left_stick_y;
      double x = gamepad1.left_stick_x;
      double rx = gamepad1.right_stick_x;

      double botHeading = AngleUnit.DEGREES.toRadians(robot.getHeading());

      double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
      double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
      rotX = rotX * 1.1;  // Counteract imperfect strafing

      double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
      double frontLeftPower = (rotY + rotX + rx) / denominator;
      double backLeftPower = (rotY - rotX + rx) / denominator;
      double frontRightPower = (rotY - rotX - rx) / denominator;
      double backRightPower = (rotY + rotX - rx) / denominator;

      // Slow mode
      if (gamepad1.right_bumper) {
        frontLeftPower *= 0.4;
        backLeftPower *= 0.4;
        frontRightPower *= 0.4;
        backRightPower *= 0.4;
      }

      robot.fl.setPower(frontLeftPower);
      robot.bl.setPower(backLeftPower);
      robot.fr.setPower(frontRightPower);
      robot.br.setPower(backRightPower);

      robot.intake.setPower(
          (gamepad1.right_trigger) - (Range.clip(gamepad1.left_trigger, -0.8, 0.8)));

      if (gamepad2.dpad_up) {
        if (!yHeld) {
          gateClosed = !gateClosed;
        }
        yHeld = true;
      } else {
        yHeld = false;
      }
      // Hang mode
      if (gamepad2.a) {
        robot.setSlidePower(-1);
        robot.flipperControl(false);
        robot.setGate(true);
      } else {
        robot.setGate(!gateClosed || (robot.slideL.getCurrentPosition() >= 100));
        robot.flipperControl(!gamepad1.x);
        robot.setSweepOut(gamepad1.dpad_up);
        robot.setSlidePower(!gateClosed ? -gamepad2.right_stick_y : 0);
      }

      if (gamepad2.y && gamepad2.x) {
        robot.fly();
      }

      telemetry.update();
    }
  }
}