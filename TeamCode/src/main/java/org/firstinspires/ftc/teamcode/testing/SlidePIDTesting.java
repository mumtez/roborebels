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

  //TODO: TUNING (https://www.robotsforroboticists.com/pid-control/)
  // Tuned but need a mag lim switch due to encoder drift / belt skipping
  public static double kp = 0.01;
  public static double ki = 0;
  public static double kd = 0.0001;
  public static double KG = 0.07;
  public static double TARGET = 1000;

  private final ElapsedTime timer = new ElapsedTime();
  private double lastError = 0;
  private double integralSum = 0;


  @Override
  public void runOpMode() throws InterruptedException {
    // use dashboard telemetry
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    Robot robot = new Robot(this);
    robot.setVerticalSlideMode(RunMode.STOP_AND_RESET_ENCODER);

    waitForStart();
    robot.setVerticalSlideMode(RunMode.RUN_WITHOUT_ENCODER);
    timer.reset();

    while (opModeIsActive()) {
      int leftRef = robot.slideLeft.getCurrentPosition();
      int rightRef = robot.slideRight.getCurrentPosition();

      //int reference = (leftRef + rightRef) / 2; // does not seem to improve performance
      int reference = leftRef;
      double power = PIDControl(TARGET, reference);
      robot.setVerticalSlidePower(power + KG);

      if (gamepad1.square) {
        reset();
      }

      telemetry.addData("TARGET", TARGET);
      telemetry.addData("REFERENCE", reference);
      telemetry.addData("OUTPUT", power);

      telemetry.addData("left slide ticks", leftRef);
      telemetry.addData("right slide ticks", rightRef);
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
    double dt = timer.seconds();

    integralSum += error * dt;
    double derivative = (error - lastError) / dt;

    lastError = error;

    double output = (error * kp) + (derivative * kd) + (integralSum * ki);
    timer.reset(); // TODO: Rob added this, see if you can understand why it's necessary
    return output;
  }
}
