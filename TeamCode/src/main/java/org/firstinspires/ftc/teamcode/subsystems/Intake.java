package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

@Config
public class Intake {

  public static double SLIDE_TRANSFER = 0.39;
  public static double SLIDE_OUT = .67;

  public static double INTAKE_DOWN = 0.23;
  public static double INTAKE_FLAT = 0.1;

  private final DcMotor intake;
  public final ServoImplEx rotate, hSlide;
  // TODO: add color sensor
  // public static float INTAKE_COLOR_GAIN = 2;
  // public final NormalizedColorSensor color;

  /*
    intakeColor = hardwareMap.get(NormalizedColorSensor.class, "ins");
    intakeColor.setGain(INTAKE_COLOR_GAIN);
    if (intakeColor instanceof SwitchableLight) {
      ((SwitchableLight) intakeColor).enableLight(
          true); // Turn the light ON to observe objects that dont emit their own light
    }

 */
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
  }


  public void setHorizontalSlidePos(double pos) {
    pos = Range.clip(pos, SLIDE_TRANSFER, SLIDE_OUT);
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
