package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.SwitchableLight;

@Config
public class Intake {

  public static double SLIDE_IN = 0.405;
  public static double SLIDE_TRANSFER = 0.405;
  public static double SLIDE_OUT = 0.68;

  public static double INTAKE_DOWN = 0.16;
  public static double INTAKE_FLAT = 0.05;

  public static float COLOR_GAIN = 2;
  public static float COLOR_THRESHOLD = 10;

  private final DcMotor intake;
  public final ServoImplEx rotate, hSlide;

  public final NormalizedColorSensor color;

  // TODO: utilize color sensor + tune GAIN for environment
  //    NormalizedRGBA colors = intakeColor.getNormalizedColors();
  //    Access colors:
  //    colors.red; colors.green; colors.blue; colors.alpha;
  //    Convert to HSV if desired:
  //    final float[] hsvValues = new float[3];
  //    Color.colorToHSV(colors.toColor(), hsvValues);
  //    hsvValues[0] // hue
  //    hsvValues[1] // sat
  //    hsvValues[2] // value


  public Intake(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;

    intake = hardwareMap.dcMotor.get("int");
    intake.setMode(RunMode.RUN_WITHOUT_ENCODER);
    intake.setDirection(DcMotorSimple.Direction.REVERSE);
    intake.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

    rotate = (ServoImplEx) hardwareMap.servo.get("flip");
    rotate.setDirection(Direction.REVERSE);

    hSlide = (ServoImplEx) hardwareMap.servo.get("so");
    hSlide.setDirection(Direction.FORWARD);

    color = hardwareMap.get(NormalizedColorSensor.class, "ins");
    color.setGain(COLOR_GAIN);
    if (color instanceof SwitchableLight) {
      ((SwitchableLight) color).enableLight(true);
      // Turn the light ON to observe objects that don't emit their own light
    }
  }

  public NormalizedRGBA senseColor() {
    return this.color.getNormalizedColors();
  }

  public double senseDistance() {
    return ((OpticalDistanceSensor) this.color).getLightDetected();
  }

  public void setHorizontalSlidePos(double pos) {
    this.hSlide.setPosition(pos);
  }

  public void setPower(double pow) {
    this.intake.setPower(pow);
  }

  public void rotateFlat() {
    this.rotate.setPosition(INTAKE_FLAT);
  }

  public void rotateDown() {
    this.rotate.setPosition(INTAKE_DOWN);
  }
}
