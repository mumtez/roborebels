package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import org.firstinspires.ftc.teamcode.NewRobot;

@Config
@TeleOp(name = "HORIZ SLIDE PID TESTING", group = "TESTING")
public class HorizSlidePIDTesting extends LinearOpMode {

  public static int TARGET = 0;

  @Override
  public void runOpMode() throws InterruptedException {
    // use dashboard telemetry
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    NewRobot robot = new NewRobot(this);
    robot.intake.horSlide.setMode(RunMode.STOP_AND_RESET_ENCODER);

    waitForStart();
    robot.intake.horSlide.setMode(RunMode.RUN_WITHOUT_ENCODER);

    while (opModeIsActive()) {
      if (gamepad1.square) {
        robot.intake.setHorizontalSlidePos(TARGET);
      }

      double curPow = robot.intake.horSlide.updatePIDControl();

      telemetry.addData("TARGET", TARGET);
      telemetry.addData("REFERENCE", robot.intake.horSlide.position);
      telemetry.addData("error:", robot.intake.horSlide.position - TARGET);
      telemetry.addData("cur pow", curPow);
      telemetry.update();
    }

  }

}
