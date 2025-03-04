package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class HorizontalSlides {

  public static int TRANSFER_POS = 0;
  public static int OUT_POS = 700;

  public static double MAX_POW = 1.0;

  public static double kf = 0.07;
  public static double kp = 0.0029;
  public static double ki = 0;
  public static double kd = 0.00006;

  private final ElapsedTime timer = new ElapsedTime();
  private double lastError = 0;
  private double integralSum = 0;
  private int targetPos = 0;

  public int position = 0;

  public final DcMotor hSlide;


  public HorizontalSlides(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;

    hSlide = hardwareMap.dcMotor.get("hs");

    hSlide.setDirection(Direction.FORWARD);
    hSlide.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    hSlide.setMode(RunMode.RUN_WITHOUT_ENCODER);
  }

  public void setMode(RunMode mode) {
    hSlide.setMode(mode);
  }

  public void setPower(double pow) {
    hSlide.setPower(pow);
  }

  public void setTarget(int targetPos) {
    timer.reset();
    lastError = 0;
    integralSum = 0;
    this.targetPos = Range.clip(targetPos, TRANSFER_POS, OUT_POS);
  }

  public int getTarget() {
    return this.targetPos;
  }

  public void updatePosition() {
    this.position = this.hSlide.getCurrentPosition();
  }

  public boolean atTarget() {
    return atTarget(20);
  }

  public boolean atTarget(int threshold) {
    return Math.abs(this.position - this.targetPos) < threshold;
  }

  // ALWAYS CALL UPDATE POSITION FIRST
  public double updatePIDControl() {
    if (this.position < 10 && this.targetPos < 10) {
      this.setPower(0);
      return 0;
    }

    double error = this.targetPos - this.position;
    double dt = timer.seconds();

    integralSum += error * dt;
    double derivative = (error - lastError) / dt;

    lastError = error;

    double pow = (error * kp) + (derivative * kd) + (integralSum * ki) + Math.signum(error) * kf;
    timer.reset();

    pow = Range.clip(pow, -MAX_POW, MAX_POW);
    this.setPower(pow);
    return pow;
  }

}
