package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@TeleOp(name = "Basic Teleop", group = "Teleop")
public class BasicOpMode extends LinearOpMode {

  DcMotor intake;
  DcMotor frontLeftMotor;
  DcMotor backLeftMotor;
  DcMotor frontRightMotor;
  DcMotor backRightMotor;
  DcMotor sl;
  DcMotor sr;

  IMU imu;

  @Override
  public void runOpMode() throws InterruptedException {
    imu = hardwareMap.get(IMU.class, "imu");

    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        LogoFacingDirection.RIGHT,
        RevHubOrientationOnRobot.UsbFacingDirection.UP));
    imu.initialize(parameters);

    intake = hardwareMap.dcMotor.get("intake");
    frontLeftMotor = hardwareMap.dcMotor.get("fl");
    backLeftMotor = hardwareMap.dcMotor.get("bl");
    frontRightMotor = hardwareMap.dcMotor.get("fr");
    backRightMotor = hardwareMap.dcMotor.get("br");
    sl = hardwareMap.dcMotor.get("sl");
    sr = hardwareMap.dcMotor.get("sr");

    intake.setMode(RunMode.RUN_WITHOUT_ENCODER);

    frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // Starting here you would also add this as a private field and change the references to .this

    sl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    sr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    sl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    sr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    intake.setMode(RunMode.RUN_WITHOUT_ENCODER);

    frontLeftMotor.setDirection(Direction.REVERSE);
    backRightMotor.setDirection(Direction.FORWARD);

    sr.setDirection(Direction.REVERSE);

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

      double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

      // Rotate the movement direction counter to the bot's rotation
      double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
      double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

      rotX = rotX * 1.1; // Very important

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

      sl.setPower(slide);
      sr.setPower(slide);

      telemetry.update();
    }
  }
}