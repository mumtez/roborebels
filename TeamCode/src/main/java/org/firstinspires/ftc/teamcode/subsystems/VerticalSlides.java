package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class VerticalSlides {

  public static int TRANSFER = 430;
  public static int DEFAULT = 750;
  public static int UP = 2800;

  public static double kp = 0.01;
  public static double ki = 0;
  public static double kd = 0.0001;
  public static double KG = 0.07;

  private final ElapsedTime timer = new ElapsedTime();
  private double lastError = 0;
  private double integralSum = 0;
  private int targetPos = DEFAULT;
  private int offset = 0;
  public int position = 0;

  public final DcMotor slideLeft;
  public final DcMotor slideRight;

  public final TouchSensor magLim;

  public VerticalSlides(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;

    slideLeft = hardwareMap.dcMotor.get("lu");
    slideRight = hardwareMap.dcMotor.get("ru");
    slideLeft.setDirection(Direction.FORWARD);
    slideRight.setDirection(Direction.REVERSE);
    slideLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    slideRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    slideLeft.setMode(RunMode.RUN_WITHOUT_ENCODER);
    slideRight.setMode(RunMode.RUN_WITHOUT_ENCODER);

    magLim = hardwareMap.touchSensor.get("mag");
  }

  public void setMode(RunMode mode) {
    slideLeft.setMode(mode);
    slideRight.setMode(mode);
  }

  public void setPower(double pow) {
    slideRight.setPower(pow);
    slideLeft.setPower(pow);
  }

  public void setTarget(int targetPos) {
    timer.reset();
    lastError = 0;
    integralSum = 0;
    this.targetPos = targetPos;
  }

  public int getTarget() {
    return this.targetPos;
  }

  private void updatePosition() {
    int curPos = this.slideLeft.getCurrentPosition();
    if (magLim.isPressed()) {
      this.offset = curPos - VerticalSlides.TRANSFER;
    }
    this.position = curPos - this.offset;
  }

  public boolean atTarget(int threshold) {
    return Math.abs(this.position - this.targetPos) < threshold;
  }

  public void updatePIDControl() {
    this.updatePosition();
    if (this.position < 10 && this.targetPos < 10) {
      this.setPower(0);
      return;
    }

    double error = this.targetPos - this.position;
    double dt = timer.seconds();

    integralSum += error * dt;
    double derivative = (error - lastError) / dt;

    lastError = error;

    double pow = (error * kp) + (derivative * kd) + (integralSum * ki) + KG;
    timer.reset();

    this.setPower(pow);
  }

}
