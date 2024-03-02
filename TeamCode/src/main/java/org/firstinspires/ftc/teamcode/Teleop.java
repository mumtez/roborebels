package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
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

      robot.fl.setPower(frontLeftPower);
      robot.bl.setPower(backLeftPower);
      robot.fr.setPower(frontRightPower);
      robot.br.setPower(backRightPower);

      robot.intake.setPower((gamepad1.right_trigger) - (gamepad1.left_trigger));
      if (gamepad2.y) {
        if (!yHeld) {
          gateClosed = !gateClosed;
        }
        yHeld = true;
      } else {
        yHeld = false;
      }
      robot.toggleDoor(!gateClosed || (robot.slideL.getCurrentPosition() >= 100));
      robot.flipperControl(robot.intake.getPower() != 0 || gamepad1.x);
      robot.setSlidePower(-gamepad2.right_stick_y);

      if (gamepad2.a && gamepad2.x) {
        robot.fly();
      }

      telemetry.update();
    }
  }
}