package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Robot;

public class TuneStackDrive extends LinearOpMode {

  Robot robot;

  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    waitForStart();

    while (opModeIsActive()) {
      robot.driveToStack();
    }
  }
}
