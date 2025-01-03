package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "ServoTesting", group = "TESTING")
public class ServoTesting extends LinearOpMode {

  Robot robot;
  
  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);

    waitForStart();
    // START

    // LOOP
    while (opModeIsActive()) {
      if (gamepad1.a) {
        robot.slideOUT.setPosition(0.5);
      } else {
        robot.slideOUT.setPosition(0);
      }

      if (gamepad1.b) {
        robot.flipper.setPosition((0.5));
      } else {
        robot.flipper.setPosition(0);
      }

      if (gamepad1.x) {
        robot.intake.setPower(1);
      } else {
        robot.intake.setPower(0);
      }

      if (gamepad1.y) {
        //robot.outtake.setPosition((0.5));
      } else {
        //robot.outtake.setPosition(0);
      }
    }
  }
}