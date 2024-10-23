package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {

  Robot robot;
  double out;

  boolean flipIn = true;
  boolean slideOut = false;

  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);
    //robot.slideUP.setMode(RunMode.RUN_WITHOUT_ENCODER);


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

      if (gamepad1.dpad_up){
        robot.slideOUT.setPosition(0);
        slideOut = false;
      }
      if (gamepad1.dpad_down){
        robot.slideOUT.setPosition(0.5);
        slideOut = true;
      }

      if (slideOut){
        if (gamepad1.a){
          robot.flipper.setPosition(1.6);
        }
        else{
          robot.flipper.setPosition(1.2);
        }
      }
      else{
        robot.flipper.setPosition(0.01);
      }

      /*

      if (gamepad1.a && slideOut){
        flipIn = false;
        //robot.flipper.setPosition(0);
      }
      else{
        flipIn = true;
      }


      if (flipIn && !slideOut){
        robot.flipper.setPosition(0.01);
      }

      if(!flipIn)
      {
        robot.flipper.setPosition(1.6);
      }

      if (slideOut && flipIn){
        robot.flipper.setPosition(1.2);
      }

       */

      robot.slideUP.setPower(gamepad2.right_stick_y);


      robot.fl.setPower(frontLeftPower);
      robot.bl.setPower(backLeftPower);
      robot.fr.setPower(frontRightPower);
      robot.br.setPower(backRightPower);


      // INTAKE

      //out -= gamepad2.right_stick_y;
      //out = Math.max(.005, Math.min(x,.4));

      //robot.slideOUT.setPosition(out);

      //telemetry.addData("out: ", out);

      if (gamepad2.dpad_up){
        robot.slideOUT.setPosition(0.01);
      }
      if (gamepad2.dpad_down){
        robot.slideOUT.setPosition(0.3);
      }

      //mathew joseph sneyers
      if (gamepad2.right_bumper){
        robot.intake.setPosition(1);
      }
      if (gamepad2.left_bumper){
        robot.intake.setPosition(0);
      }
      if (!gamepad2.left_bumper && !gamepad2.right_bumper){
        robot.intake.setPosition(0.5);
      }



      telemetry.update();
    }
  }
}