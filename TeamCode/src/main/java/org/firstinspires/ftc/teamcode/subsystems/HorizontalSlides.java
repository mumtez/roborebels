package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class HorizontalSlides {

  public static int TRANSFER_POS = 0;
  public static int OUT_POS = 45;

  public static double MAX_POW = 1.0;

  public static double kf = 0.0;
  public static double kp = 0.00;
  public static double ki = 0;
  public static double kd = 0.0000;

  private final ElapsedTime timer = new ElapsedTime();
  private double lastError = 0;
  private double integralSum = 0;
  private int targetPos = TRANSFER_POS;
  public int position = 0;
  private int offset = 0;

  public final TouchSensor magLim;
  public AnalogInput horServoAnalog;
  public CRServo horServoTop;

  public CRServo horServoBottom;


  public HorizontalSlides(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;
    horServoAnalog = hardwareMap.get(AnalogInput.class, "horservoanalog");

    horServoTop = hardwareMap.get(CRServo.class, "horservotop");
    horServoBottom = hardwareMap.get(CRServo.class, "horservobottom");

    horServoTop.setDirection(Direction.REVERSE);
    horServoBottom.setDirection(Direction.FORWARD);

    magLim = hardwareMap.touchSensor.get("magh");
  }

  /*public void setMode(RunMode mode) {
    hSlide.setMode(mode);
  }*/

  public void setPower(double pow) {
    horServoTop.setPower(pow);
    horServoBottom.setPower(pow);
  }

  public void setTarget(int target) {
    if (target != this.targetPos) {
      timer.reset();
      lastError = 0;
      integralSum = 0;
      this.targetPos = target;
    }
  }

  public int getTarget() {
    return this.targetPos;
  }

  public void updatePosition() {
    int curPos = (int) (horServoAnalog.getVoltage() / 3.3 * 360);
    if (this.magLim.isPressed()) {
      this.offset = curPos - HorizontalSlides.TRANSFER_POS;
    }
    this.position = curPos - this.offset;
  }

  public boolean atTarget() {
    return atTarget(3);
  }

  public boolean atTarget(int threshold) {
    boolean inRange = Math.abs(this.position - this.targetPos) < threshold;
    if (this.targetPos == TRANSFER_POS) {
      inRange = inRange || this.magLim.isPressed();
    }
    return inRange;
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
