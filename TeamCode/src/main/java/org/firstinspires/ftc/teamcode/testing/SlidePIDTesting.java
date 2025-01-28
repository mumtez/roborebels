package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "SLIDE PID TESTING", group = "TESTING")
public class SlidePIDTesting extends LinearOpMode {

  //TODO: TUNING (https://www.robotsforroboticists.com/pid-control/)
  public static double kp = 0;
  public static double ki = 0;
  public static double kd = 0;
  public static double TARGET = 100;

  private final ElapsedTime timer = new ElapsedTime();
  private double lastError = 0;
  private double integralSum = 0;


  @Override
  public void runOpMode() throws InterruptedException {
    // use dashboard telemetry
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    Robot robot = new Robot(this);

    waitForStart();
    timer.reset();

    while (opModeIsActive()) {
      int reference = robot.slideLeft.getCurrentPosition();

      double power = PIDControl(TARGET, reference);
      robot.setVerticalSlidePower(power);

      if (gamepad1.a) {
        reset();
      }

      telemetry.addData("TARGET", TARGET);
      telemetry.addData("REFERENCE", reference);
      telemetry.update();
    }

  }

  public void reset() {
    timer.reset();
    lastError = 0;
    integralSum = 0;
  }

  public double PIDControl(double reference, double state) {
    double error = reference - state;
    integralSum += error * timer.seconds();
    double derivative = (error - lastError) / timer.seconds();
    lastError = error;

    return (error * kp) + (derivative * kd) + (integralSum * ki);
  }
}
