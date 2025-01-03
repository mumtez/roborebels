package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

@Config
@TeleOp(name = "Claw Test", group = "TESTING")
public class ClawTesting extends LinearOpMode {

  Robot robot;
  Claw claw;

  public static double flipPos = 0.5;

  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);
    claw = new Claw(this);

    waitForStart();
    // START

    // LOOP
    while (opModeIsActive()) {

      if (gamepad1.right_bumper) {
        //robot.clawOpen();
      }
      if (gamepad1.left_bumper) {
        //robot.clawClose();
      }

      if (gamepad1.b) {
        robot.claw.setPlace();
      }

      if (gamepad1.a) {
        robot.claw.setDefault();
      }

      if (gamepad1.x) {
        robot.claw.setBucket();
      }

      if (gamepad1.y) {
        robot.claw.setWall();
      }

      if (gamepad1.dpad_down) {
        robot.setSlideUpPos(robot.VERTICAL_SLIDE_DEFAULT, 0.8);
      }

      if (gamepad1.dpad_up) {
        robot.setSlideUpPos(robot.VERTICAL_SLIDE_UP, 0.8);
      }

    }
  }
}