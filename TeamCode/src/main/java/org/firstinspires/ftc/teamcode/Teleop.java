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
  boolean timerDone = false;

  boolean timerDone2 = false;

  boolean outtaking = false;

  int timer = 60;
  int timer2 = 60;

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

      if (gamepad2.dpad_down) {
        //robot.slideOUT.setPosition(0.03);
        slideOut = false;

        if (!slideOut){
          timer2 = 0;
        }
        timerDone2 = false;
      }

      if (!timerDone2){
        timer2++;
      }
      if (timer2 > 50){
        timerDone2 = true;
      }
      if (timerDone2 && !slideOut){
        robot.slideOUT.setPosition(0.03);
      }


      if (gamepad2.dpad_up){;
        if (robot.slideOUT.getPosition() != 0.4) {
          timer = 0;
        }
        robot.slideOUT.setPosition(0.4);
        timerDone = false;
        slideOut = true;
      }

      if (!timerDone){
        timer++;
      }
      if (timer > 50) {
        timerDone = true;
      }

      /*
      telemetry.addData("slide pos", robot.slideOUT.getPosition());
      telemetry.addData("timer", timer);
      telemetry.addData("flipIn", slideOut);

       */

      if (slideOut && timerDone){
        if (gamepad2.a){
          robot.flipper.setPosition(0.85);
          flipIn = false;
        }
        else{
          robot.flipper.setPosition(0.75);
          flipIn = true;
        }
      }
      else{
          robot.flipper.setPosition(0.01);
      }




      robot.slideUP.setPower(-gamepad2.right_stick_y);
      telemetry.addData("stick", gamepad2.right_stick_y);


      if (gamepad1.a){
        outtaking = false;
      }

      if(outtaking){
        robot.flipOut();
      }
      else{
        robot.flipIn();
      }


      robot.fl.setPower(frontLeftPower);
      robot.bl.setPower(backLeftPower);
      robot.fr.setPower(frontRightPower);
      robot.br.setPower(backRightPower);


      // INTAKE

      //out -= gamepad2.right_stick_y;
      //out = Math.max(.005, Math.min(x,.4));

      //robot.slideOUT.setPosition(out);

      //telemetry.addData("out: ", out);


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