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
    // START

    // LOOP
    while (opModeIsActive()) {

      if (gamepad1.right_bumper) {
        robot.claw.clawOpen();
      }
      if (gamepad1.left_bumper) {
        robot.claw.clawClose();
      }

      if (gamepad1.b) {
        robot.claw.setPlace();
      }

      if (gamepad1.a) {
        robot.claw.setUnder();
      }

      if (gamepad1.x) {
        robot.claw.setBucket();
      }

      if (gamepad1.y) {
        robot.claw.setWall();
      }

      if (gamepad1.dpad_down) {
        robot.startSlideUpPos(Robot.VERTICAL_SLIDE_DEFAULT, 0.8);
      }

      if (gamepad1.dpad_up) {
        robot.startSlideUpPos(Robot.VERTICAL_SLIDE_UP, 0.8);
      }
      if (gamepad2.a) {
        robot.rotateIntakeDown();
      }
      if (gamepad2.b) {
        robot.rotateIntakeFlat();
      }

      robot.intake.setPower(gamepad2.left_trigger - gamepad2.right_trigger);

      if (gamepad2.dpad_up) {
        robot.setHorizontalSlidePos(position);
      }

    }
  }
}