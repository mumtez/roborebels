package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import org.firstinspires.ftc.teamcode.NewRobot;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides;

@Config
@TeleOp(name = "Claw Test", group = "TESTING")
public class ClawTesting extends LinearOpMode {

  NewRobot robot;
  public static int position = 0;

  @Override
  public void runOpMode() throws InterruptedException {
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    robot = new NewRobot(this);

    robot.slides.setMode(RunMode.STOP_AND_RESET_ENCODER);
    waitForStart();
    robot.slides.setMode(RunMode.RUN_WITHOUT_ENCODER);

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

      if (gamepad1.dpad_left) {
        robot.slides.setTarget(VerticalSlides.TRANSFER);
      }
      if (gamepad1.dpad_up) {
        robot.slides.setTarget(VerticalSlides.UP);
      }

      if (gamepad2.square) {
        robot.intake.rotateDown();
      }
      if (gamepad2.cross) {
        robot.intake.rotateFlat();
      }

      robot.intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

      if (gamepad2.dpad_down) {
        robot.intake.horSlide.setTarget(position);
      }

      robot.slides.updatePIDControl();

      telemetry.addData("SLIDE REFERENCE", robot.slides.position);
      telemetry.update();
    }
  }
}