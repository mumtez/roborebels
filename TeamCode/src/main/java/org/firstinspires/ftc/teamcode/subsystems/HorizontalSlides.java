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

  public static int MAX_POS = 900;  //TODO: Should probably tune this so we dont break the slides

  public static double MAX_POW = 0.2;

  // TODO: setup actual positions for horizontal slide (just out and in?)
  public static int TRANSFER = 430;
  public static int DEFAULT = 600;
  public static int UP = 2000;
  public static int SPECIMEN = 330;
  public static int PRE_TRANSFER = 900;

  public static double kp = 0.006;
  public static double ki = 0;
  public static double kd = 0.0007;

  private final ElapsedTime timer = new ElapsedTime();
  private double lastError = 0;
  private double integralSum = 0;
  private int targetPos = DEFAULT;
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
    this.targetPos = Math.max(0, Math.min(MAX_POS, targetPos));
  }

  public int getTarget() {
    return this.targetPos;
  }

  private void updatePosition() {
    this.position = this.hSlide.getCurrentPosition();
  }

  // TODO: anywhere you depend on hslide position you should be using this now, not getcurrent position
  //  Also need to add pid/pos updates to your loops in teleop & auto
  public boolean atTarget(int threshold) {
    return Math.abs(this.position - this.targetPos) < threshold;
  }

  public double updatePIDControl() {
    this.updatePosition();
    if (this.position < 10 && this.targetPos < 10) {
      this.setPower(0);
      return 0;
    }

    double error = this.targetPos - this.position;
    double dt = timer.seconds();

    integralSum += error * dt;
    double derivative = (error - lastError) / dt;

    lastError = error;

    double pow = (error * kp) + (derivative * kd) + (integralSum * ki);
    timer.reset();

    pow = Range.clip(pow, -MAX_POW, MAX_POW);

    this.setPower(pow);
    return pow;
  }

}
