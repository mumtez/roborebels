package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "Claw Test", group = "TESTING")
public class ClawTesting extends LinearOpMode {

  Robot robot;
  public static double position = 0;

  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);
    waitForStart();

    // LOOP
    while (opModeIsActive()) {

      if (gamepad1.right_bumper) {
        robot.claw.clawOpen();
      }
      if (gamepad1.left_bumper) {
        robot.claw.clawClose();
      }

      if (gamepad1.circle) {
        robot.claw.setPlace();
      }

      if (gamepad1.square) {
        robot.claw.setUnder();
      }

      if (gamepad1.cross) {
        robot.claw.setBucket();
      }

      if (gamepad1.triangle) {
        robot.claw.setWall();
      }

      if (gamepad1.left_trigger > 0.5) {
        robot.claw.setInit();
      }

      if (gamepad1.right_trigger > 0.5) {
        robot.claw.setTransfer();
      }

      if (gamepad1.dpad_down) {
        robot.startSlideUpPos(Robot.VERTICAL_SLIDE_DEFAULT, 0.8);
      }

      if (gamepad1.dpad_up) {
        robot.startSlideUpPos(Robot.VERTICAL_SLIDE_UP, 0.8);
      }
      if (gamepad2.square) {
        robot.intake.rotateDown();
      }
      if (gamepad2.cross) {
        robot.intake.rotateFlat();
      }

      robot.intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

      if (gamepad2.dpad_up) {
        robot.intake.setHorizontalSlidePos(position);
      }

    }
  }
}