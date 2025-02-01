package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
@TeleOp(name = "Intake Test", group = "TESTING")
public class IntakeTesting extends LinearOpMode {

  Robot robot;

  public static double rotatePos = Intake.INTAKE_FLAT;
  public static double hSlidePos = Intake.SLIDE_TRANSFER;

  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);

    waitForStart();
    // START

    // LOOP
    while (opModeIsActive()) {
      robot.intake.setHorizontalSlidePos(hSlidePos);
      robot.intake.rotate.setPosition(rotatePos);
      robot.intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

    }
  }
}