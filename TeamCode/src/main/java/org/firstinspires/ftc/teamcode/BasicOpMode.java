package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Basic Teleop", group = "Teleop")
public class BasicOpMode extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {

    DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
    DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
    DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
    DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

    frontLeftMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
    backLeftMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
    frontRightMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
    backRightMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);

    frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    DcMotor slideMotor = hardwareMap.dcMotor.get("slideMotor");

    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
    slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

    // Reverse the right side motors. This may be wrong for your setup.
    // If your robot moves backwards when commanded to go forwards,
    // reverse the left side instead.
    frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    IMU imu = hardwareMap.get(IMU.class, "imu");
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
        RevHubOrientationOnRobot.UsbFacingDirection.UP));
    imu.initialize(parameters);

    waitForStart();

    if (isStopRequested()) {
      return;
    }

    while (opModeIsActive()) {
      double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
      double x = gamepad1.left_stick_x;
      double rx = gamepad1.right_stick_x;

      double slide = gamepad1.right_trigger;
      double unSlide = -gamepad1.left_trigger;

      float toGo = 0;

      if (gamepad1.options) {
        imu.resetYaw();
      }

      double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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

      frontLeftMotor.setPower(frontLeftPower);
      backLeftMotor.setPower(backLeftPower);
      frontRightMotor.setPower(frontRightPower);
      backRightMotor.setPower(backRightPower);

      slideMotor.setPower(slide + unSlide);

      int slidePos = slideMotor.getCurrentPosition();

      if (gamepad1.dpad_down) {
        toGo = -5;
      }
      if (gamepad1.dpad_up) {
        toGo = -1200;
      }

      telemetry.addData("Slide pos", slidePos);
      telemetry.update();
    }
  }
}