package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NewRobot;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlides;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
@TeleOp(name = "Intake Test", group = "TESTING")
public class IntakeTesting extends LinearOpMode {

  NewRobot robot;

  public static double rotatePos = Intake.INTAKE_FLAT;
  public static double hSlidePos = HorizontalSlides.TRANSFER_POS;

  @Override
  public void runOpMode() throws InterruptedException {
    robot = new NewRobot(this);

    waitForStart();

    // START

    // LOOP
    while (opModeIsActive()) {
      if (gamepad1.cross) {
        robot.horSlide.setTarget(hSlidePos);
      }
      robot.intake.rotate.setPosition(rotatePos);
      robot.intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

      double curPow = robot.horSlide.updatePIDControl();

      telemetry.addData("HSLIDE TARGET", hSlidePos);
      telemetry.addData("REFERENCE", robot.horSlide.position);
      telemetry.addData("error:", robot.horSlide.position - hSlidePos);
      telemetry.addData("cur pow", curPow);
      telemetry.update();
    }
  }
}