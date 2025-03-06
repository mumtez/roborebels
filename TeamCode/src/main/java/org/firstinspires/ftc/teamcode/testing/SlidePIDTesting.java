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
@TeleOp(name = "SLIDE PID TESTING", group = "TESTING")
public class SlidePIDTesting extends LinearOpMode {

  public static int TARGET = VerticalSlides.UP;

  @Override
  public void runOpMode() throws InterruptedException {
    // use dashboard telemetry
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    NewRobot robot = new NewRobot(this);
    robot.slides.setMode(RunMode.STOP_AND_RESET_ENCODER);

    waitForStart();
    robot.slides.setMode(RunMode.RUN_WITHOUT_ENCODER);

    while (opModeIsActive()) {
      if (gamepad1.square) {
        robot.slides.setTarget(TARGET);
      }

      double pow = robot.slides.updatePIDControl();

      telemetry.addData("TARGET", TARGET);
      telemetry.addData("REFERENCE", robot.slides.position);
      telemetry.addData("POWER", pow);
      telemetry.update();
    }

  }

}
