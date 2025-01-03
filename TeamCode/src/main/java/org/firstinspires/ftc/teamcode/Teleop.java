package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

@Config
@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {

  Robot robot;

  boolean slideOut = false;
  boolean outtaking = false;

  int timer = 60;
  int timer2 = 60;

  boolean timerDone = false;
  boolean timerDone2 = false;

  double horizontalPos = 0;

  float slideOutPos = 0;

  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);

    waitForStart();
    // START

    // LOOP
    while (opModeIsActive()) {
      //int hangPos = robot.hang.getCurrentPosition();
      int vSlideLPos = robot.slideLeft.getCurrentPosition();
      int vSlideRPos = robot.slideRight.getCurrentPosition();

      if (gamepad1.left_bumper) {
        robot.drive.lazyImu.get().resetYaw();
      }

      // === FIELD CENTRIC ===

      boolean slideGoOut;

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

      robot.setDriveTrainPower(frontRightPower, frontLeftPower, backRightPower, backLeftPower);

      /*
      robot.fl.setPower(frontLeftPower);
      robot.bl.setPower(backLeftPower);
      robot.fr.setPower(frontRightPower);
      robot.br.setPower(backRightPower);

       */

      // Hor slide w/ stick instead of toggle
      //horizontalPos -= gamepad2.left_stick_y/500;
      //horizontalPos = Math.max(0, Math.min(1, slideOutPos));

      if (gamepad2.dpad_down) {
        slideOut = false;
        timer2 = 0;
        timerDone2 = false;
      }

      if (!timerDone2) {
        timer2++;
      }
      if (timer2 > 35) {
        timerDone2 = true;
      }

      if (timerDone2 && !slideOut) {
        horizontalPos = Robot.HORIZONTAL_SLIDE_TRANSFER;
      }

      // TODO: direct control horizontal slides (increment/decrement servo position)
      if (gamepad2.dpad_up) {
        if (robot.slideOUT.getPosition() != Robot.HORIZONTAL_SLIDE_OUT) {
          timer = 0;
        }
        horizontalPos = Robot.HORIZONTAL_SLIDE_OUT;
        timerDone = false;
        slideOut = true;
      }

      if (gamepad2.right_stick_y != 0) {
        slideGoOut = true;
        horizontalPos = Robot.HORIZONTAL_SLIDE_OUT / 1.2;
      } else {
        slideGoOut = false;
      }

      robot.setHorizontalSlidePos(horizontalPos);

      if (!timerDone) {
        timer++;
      }
      if (timer > 35) {
        timerDone = true;
      }

      if (slideOut && timerDone) {
        if (gamepad2.a) {
          robot.rotateIntakeOut();
        } else {
          robot.rotateIntakeFlat();
        }
      } else {
        robot.rotateIntakeUp();
      }

      //robot.setVerticalSlidePower(-gamepad2.right_stick_y);

      if (gamepad2.dpad_left) {
        robot.setSlideUpPos(robot.VERTICAL_SLIDE_DEFAULT, 0.8);
      }

      if (gamepad2.dpad_right) {
        robot.setSlideUpPos(robot.VERTICAL_SLIDE_UP, 0.8);
      }

      outtaking = gamepad1.b;

            /*
            if (outtaking && (vSlideLPos + vSlideRPos) / 2 > 10) {
                robot.claw.setPlace();
            }

            if (gamepad1.a){
                robot.claw.setDefault();
            }

            if (gamepad1.x){
                robot.claw.setBucket();
            }

            if (gamepad1.y){
                robot.claw.setWall();
            }

             */

      robot.intake.setPower(gamepad2.left_trigger - gamepad2.right_trigger); //    /3?

      /*
      if (gamepad2.right_bumper) {
        robot.intake.setPower(1);
      } else if (gamepad2.left_bumper) {
        robot.intake.setPower(-1);
      } else {
        robot.intake.setPower(0);
      }

       */

            /*
            if (gamepad1.dpad_left) {
                robot.hang.setPower(1);
            } else if (gamepad1.dpad_right) {
                robot.hang.setPower(-1);
            } else {
                robot.hang.setPower(0);
            }

             */

      telemetry.addData("INTAKE ROTATE POS", robot.flipper.getPosition());
      telemetry.addData("INTAKE POW", robot.intake.getPower());
      //telemetry.addData("OUTTAKE POS", robot.outtake.getPosition());
      telemetry.addData("H SLIDE POS", robot.slideOUT.getPosition());
      telemetry.addData("V SLIDE L ENC", vSlideLPos);
      telemetry.addData("V SLIDE R ENC", vSlideRPos);
      //telemetry.addData("HANG ENC", hangPos);
      telemetry.addData("Slide go out?: ", slideGoOut);
      telemetry.update();
    }
  }
}