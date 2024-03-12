package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;

//@Disabled
@TeleOp(name = "Tune Distance Sensor Drive", group = "Testing")
public class TuneDistanceSensorDrive extends LinearOpMode {

  Robot robot;

  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    waitForStart();

    while (opModeIsActive()) {
      if (gamepad1.a) {
        robot.driveToBackdrop();
      } else if (gamepad1.b) {
        robot.driveToStack();
      } else {
        telemetry.addLine("PRESS A TO DRIVE TO BACKDROP");
        telemetry.addLine("PRESS B TO DRIVE TO STACK");
        telemetry.update();
      }
    }
  }
}