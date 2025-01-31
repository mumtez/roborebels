package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "SLIDE PID TESTING", group = "TESTING")
public class SlidePIDTesting extends LinearOpMode {

  public static int TARGET = 1000;

  @Override
  public void runOpMode() throws InterruptedException {
    // use dashboard telemetry
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    Robot robot = new Robot(this);
    robot.slides.setMode(RunMode.STOP_AND_RESET_ENCODER);

    waitForStart();
    robot.slides.setMode(RunMode.RUN_WITHOUT_ENCODER);

    while (opModeIsActive()) {
      if (gamepad1.square) {
        robot.slides.setTarget(TARGET);
      }

      robot.slides.updatePIDControl();

      telemetry.addData("TARGET", TARGET);
      telemetry.addData("REFERENCE", robot.slides.position);
      telemetry.update();
    }

  }

}
