package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "Basic Teleop", group = "Teleop")
public class BasicOpMode extends LinearOpMode {

  DcMotor intake;
  DcMotor frontLeftMotor;
  DcMotor backLeftMotor;
  DcMotor frontRightMotor;
  DcMotor backRightMotor;
  DcMotor slideMotor;

  Servo airplane;
  IMU imu;

  @Override
  public void runOpMode() throws InterruptedException {
    imu = hardwareMap.get(IMU.class, "imu");

    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
        RevHubOrientationOnRobot.UsbFacingDirection.UP));
    imu.initialize(parameters);

    intake = hardwareMap.dcMotor.get("intake");
    frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
    backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
    frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
    backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
    slideMotor = hardwareMap.dcMotor.get("slideMotor");

    intake.setMode(RunMode.RUN_WITHOUT_ENCODER);

    frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    airplane = hardwareMap.servo.get("airplane");

    // Starting here you would also add this as a private field and change the references to .this

    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    waitForStart();

    if (isStopRequested()) {
      return;
    }

    while (opModeIsActive()) {
      double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
      double x = gamepad1.left_stick_x;
      double rx = -gamepad1.right_stick_x;

      double slide = gamepad2.right_stick_y;

      if (gamepad1.options) {
        imu.resetYaw();
      }

      if (gamepad2.a) {
        airplane.setPosition(-1.0);
      }

      if (gamepad2.b) {
        airplane.setPosition(1.0);
      }

      double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

      // Rotate the movement direction counter to the bot's rotation
      double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
      double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

      rotX = rotX * 1.1;

      double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
      double frontLeftPower = (rotY + rotX + rx) / denominator;
      double backLeftPower = (rotY - rotX + rx) / denominator;
      double frontRightPower = (rotY - rotX - rx) / denominator;
      double backRightPower = (rotY + rotX - rx) / denominator;

      frontLeftMotor.setPower(frontLeftPower);
      backLeftMotor.setPower(backLeftPower);
      frontRightMotor.setPower(frontRightPower);
      backRightMotor.setPower(backRightPower);

      intake.setPower(gamepad1.right_trigger - (gamepad1.left_trigger * 0.5));

      slideMotor.setPower(slide * 0.7);

      telemetry.update();
    }
  }
}