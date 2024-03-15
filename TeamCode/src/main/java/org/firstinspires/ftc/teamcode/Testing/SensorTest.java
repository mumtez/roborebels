package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Sensor Test", group = "TESTING")
public class SensorTest extends LinearOpMode {

  Robot robot;

  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    waitForStart();

    while (opModeIsActive()) {
      telemetry.addData("Backdrop CM", robot.backdropSensor.getDistance(DistanceUnit.CM));
      telemetry.addData("Stack US CM", robot.stackSensor.getDistance(DistanceUnit.CM));
      telemetry.addData("Left US CM", robot.ultrasonicRight.getDistance(DistanceUnit.CM));
      telemetry.addData("Right US CM", robot.ultrasonicLeft.getDistance(DistanceUnit.CM));
      telemetry.update();
    }
  }
}