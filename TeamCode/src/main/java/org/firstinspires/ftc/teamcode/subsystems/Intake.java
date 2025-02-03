package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.AllianceColor;

@Config
public class Intake {

  public enum SampleColor {
    RED, BLUE, YELLOW, NONE
  }

  public static double SLIDE_TRANSFER = 0.6;
  public static double SLIDE_OUT = 0.96;

  public static double INTAKE_DOWN = 0.17;
  public static double INTAKE_FLAT = 0.05;

  public static float COLOR_GAIN = 2;
  public static double RED_THRESHOLD = 0.02;
  public static double BLUE_THRESHOLD = 0.02;
  public static double COLOR_THRESHOLD = 0.01;
  public static double DIST_THRESHOLD_CM = 2;

  private final DcMotor intake;
  public final ServoImplEx rotate, hSlide;

  public final NormalizedColorSensor colorSensor;
  public final Servo rgb;

  private final ElapsedTime spitTimer = new ElapsedTime();

  private SampleColor sampleColor = SampleColor.NONE;
  private NormalizedRGBA colors;
  private double dist;

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

    colorSensor = hardwareMap.get(NormalizedColorSensor.class, "ins");
    colorSensor.setGain(COLOR_GAIN);
    if (colorSensor instanceof SwitchableLight) {
      ((SwitchableLight) colorSensor).enableLight(true);
      // Turn the light ON to observe objects that don't emit their own light
    }

    rgb = hardwareMap.servo.get("rgb");
    rgb.setPosition(0);
  }

  public void senseColor() {
    this.colors = colorSensor.getNormalizedColors();
  }

  public void senseDistance() {
    this.dist = ((RevColorSensorV3) this.colorSensor).getDistance(DistanceUnit.CM);
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

  public SampleColor getSampleColor() {
    return sampleColor;
  }

  public boolean validSampleIn(AllianceColor allianceColor) {
    if (allianceColor == AllianceColor.RED) {
      return this.sampleColor == SampleColor.RED || this.sampleColor == SampleColor.YELLOW;
    } else {
      return this.sampleColor == SampleColor.BLUE || this.sampleColor == SampleColor.YELLOW;
    }
  }

  public void update(double power, boolean flat, double hSlidePos, AllianceColor allianceColor) {
    this.senseDistance();
    this.senseColor();

    // Update sample color
    if (this.dist < DIST_THRESHOLD_CM) {
      this.sampleColor = SampleColor.YELLOW;

      if (this.colors.red >= Intake.RED_THRESHOLD && this.colors.blue < Intake.COLOR_THRESHOLD) {
        this.sampleColor = SampleColor.RED;
      } else if (this.colors.blue >= Intake.BLUE_THRESHOLD && this.colors.red < Intake.COLOR_THRESHOLD) {
        this.sampleColor = SampleColor.BLUE;
      }
    } else {
      this.sampleColor = SampleColor.NONE;
    }

    // If spitting, finish spit (400ms)
    if (this.spitTimer.milliseconds() > 400) {
      // actions based on collected sample color
      switch (this.sampleColor) {
        case RED:
          this.rgb.setPosition(0.28);
          if (allianceColor == AllianceColor.BLUE) {
            spit();
          } else {
            manualControl(power, flat);
          }
          break;
        case BLUE:
          this.rgb.setPosition(0.63);
          if (allianceColor == AllianceColor.RED) {
            spit();
          } else {
            manualControl(power, flat);
          }
          break;
        case YELLOW:
          this.rgb.setPosition(0.388);
          manualControl(power, flat);
          break;
        case NONE:
          this.rgb.setPosition(0);
          manualControl(power, flat);
          break;
      }
    }

    this.setHorizontalSlidePos(hSlidePos);
  }

  public void spit() {
    this.rotateFlat();
    this.setPower(-1);
    this.spitTimer.reset();
  }

  public double getDist() {
    return dist;
  }

  public NormalizedRGBA getColors() {
    return colors;
  }

  public void manualControl(double power, boolean flat) {
    this.intake.setPower(power);
    if (flat) {
      this.rotateFlat();
    } else {
      this.rotateDown();
    }
  }
}
