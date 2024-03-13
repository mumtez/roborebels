package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

@TeleOp(name = "Slide Test", group = "TESTING")
@Disabled
public class SlideTest extends LinearOpMode {

  DcMotor sl;
  DcMotor sr;

  public void runOpMode() throws InterruptedException {
    sl = hardwareMap.dcMotor.get("sl");
    sr = hardwareMap.dcMotor.get("sr");
    sr.setDirection(Direction.REVERSE);
    sl.setDirection(Direction.FORWARD);
    sr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    sl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    sr.setMode(RunMode.STOP_AND_RESET_ENCODER);
    sl.setMode(RunMode.STOP_AND_RESET_ENCODER);

    waitForStart();
    if (isStopRequested()) {
      return;
    }

    sr.setMode(RunMode.RUN_WITHOUT_ENCODER);
    sl.setMode(RunMode.RUN_WITHOUT_ENCODER);

    while (opModeIsActive()) {
      double slide = gamepad2.right_stick_y;
      if (sr.getCurrentPosition() < -1000 || sl.getCurrentPosition() < -1000) {
        slide = 0;
      }
      sr.setPower(slide);
      sl.setPower(slide);
      telemetry.addData("current pos both", sl.getCurrentPosition() + " " + sr.getCurrentPosition());
      telemetry.update();

    }
  }
}