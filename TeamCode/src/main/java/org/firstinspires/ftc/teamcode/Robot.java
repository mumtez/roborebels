package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.odom.MecanumDrive;

// CONFIG

@Config
public class Robot {

  public static double HORIZONTAL_SLIDE_IN = 0.3;
  public static double HORIZONTAL_SLIDE_OUT = 0.5;

  public static double INTAKE_OUT = 0.61;
  public static double INTAKE_UP = 0.03;
  public static double INTAKE_FLAT = 0.5;
  public static double INTAKE_VERTICAL = 0.2; // TODO: tune

  public static double OUTTAKE_IN = 0.05;
  public static double OUTTAKE_OUT = 0.8;

  public static int VERTICAL_SLIDE_UP = 2800;
  public static int VERTICAL_SLIDE_DOWN = 0;

  public static double GYRO_TURN_P = .055;
  public static double KG = 0.07;
  public static double HEADING_THRESHOLD = 1;

  public static double SLIDEOUT_THRESHOLD = 0.4;

  // TODO: tune color sensor gain
  public static float INTAKE_COLOR_GAIN = 2;

  //public final IMU imu;
  public final MecanumDrive drive;
  //public final DcMotor fl, fr, bl, br;
  public final DcMotor slideLeft;
  public final DcMotor slideRight;

  public final DcMotor hang;
  public final ServoImplEx slideOUT;
  public final CRServoImplEx intake;
  public final ServoImplEx flipper, outtake;

  public final NormalizedColorSensor intakeColor;

  private final LinearOpMode opMode;

  public Robot(LinearOpMode opMode) {
    this.opMode = opMode;
    HardwareMap hardwareMap = opMode.hardwareMap;

    this.drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

    // Hang
    hang = hardwareMap.dcMotor.get("hang");
    hang.setMode(RunMode.RUN_WITHOUT_ENCODER);
    hang.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

    // Slides
    slideLeft = hardwareMap.dcMotor.get("lu");
    slideRight = hardwareMap.dcMotor.get("ru");

    slideLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
    slideRight.setMode(RunMode.STOP_AND_RESET_ENCODER);

    slideLeft.setDirection(Direction.REVERSE);
    slideRight.setDirection(Direction.FORWARD);

    slideLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    slideRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

    slideLeft.setMode(RunMode.RUN_WITHOUT_ENCODER);
    slideRight.setMode(RunMode.RUN_WITHOUT_ENCODER);

    slideOUT = (ServoImplEx) hardwareMap.servo.get("so");

    // Intake

    intake = (CRServoImplEx) hardwareMap.crservo.get("in");
    outtake = (ServoImplEx) hardwareMap.servo.get("out");
    flipper = (ServoImplEx) hardwareMap.servo.get("flip");

    flipper.setDirection(Servo.Direction.REVERSE);

    // Sensor
    intakeColor = hardwareMap.get(NormalizedColorSensor.class, "ins");
    intakeColor.setGain(INTAKE_COLOR_GAIN);
    if (intakeColor instanceof SwitchableLight) {
      ((SwitchableLight) intakeColor).enableLight(
          true); // Turn the light ON to observe objects that dont emit their own light
    }
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

  }

  public void initAuton() {
    drive.lazyImu.get().resetYaw();
    this.rotateIntakeVertical();
    this.outtakeIn();
    this.setHorizontalSlidePos(HORIZONTAL_SLIDE_IN);
  }

  public void setHorizontalSlidePos(double pos) {
    pos = Range.clip(pos, HORIZONTAL_SLIDE_IN, HORIZONTAL_SLIDE_OUT);
    this.slideOUT.setPosition(pos);
  }

  public void outtakeOut() {
    outtake.setPosition(OUTTAKE_OUT);
  }

  public void outtakeIn() {
    outtake.setPosition(OUTTAKE_IN);
  }

  public void rotateIntakeFlat() {
    this.flipper.setPosition(INTAKE_FLAT);
  }

  public void rotateIntakeUp() {
    this.flipper.setPosition(INTAKE_UP);
  }

  public void rotateIntakeOut() {
    this.flipper.setPosition(INTAKE_OUT);
  }

  public void rotateIntakeVertical() {
    this.flipper.setPosition(INTAKE_VERTICAL);
  }

  public void setVerticalSlidePower(double pow) {
    slideRight.setPower((pow + KG));
    slideLeft.setPower((pow + KG));
  }

  public void setSlideUpPos(int pos, double pow) {
    setVerticalSlidePower(0);

    slideLeft.setTargetPosition(pos);
    slideRight.setTargetPosition(pos);

    slideLeft.setMode(RunMode.RUN_TO_POSITION);
    slideRight.setMode(RunMode.RUN_TO_POSITION);

    setVerticalSlidePower(pow);

    while (this.opMode.opModeIsActive() && Math.abs(slideLeft.getCurrentPosition() - pos) > 30) {
      // Wait for slide to end
    }

    setVerticalSlidePower(0);
    slideLeft.setMode(RunMode.RUN_WITHOUT_ENCODER);
    slideRight.setMode(RunMode.RUN_WITHOUT_ENCODER);

  }

  public void pickUp() {
    setHorizontalSlidePos(Robot.HORIZONTAL_SLIDE_OUT / 2);
    waitTime(500);

    rotateIntakeOut();
    intake.setPower(1);
    waitTime(500);

    setHorizontalSlidePos(Robot.HORIZONTAL_SLIDE_OUT);
    waitTime(500);

    intake.setPower(0);
    rotateIntakeUp();
    waitTime(500);

    setHorizontalSlidePos(Robot.HORIZONTAL_SLIDE_IN);
    waitTime(500);

    intake.setPower(-1);
    waitTime(500);

    rotateIntakeUp();
  }

  public void deposit() {

    intake.setPower(-.5);
    waitTime(300);

    intake.setPower(0);
    waitTime(300);

    intake.setPower(-.5);
    waitTime(300);

    intake.setPower(0);
    waitTime(300);

    intake.setPower(-.5);
    waitTime(300);

    intake.setPower(0);


    waitTime(500);

    setHorizontalSlidePos(HORIZONTAL_SLIDE_IN + 0.08);
    waitTime(1000);

    setSlideUpPos(Robot.VERTICAL_SLIDE_UP, .8);
    outtakeOut();
    waitTime(1000);

    outtakeIn();
    setSlideUpPos(Robot.VERTICAL_SLIDE_DOWN, 1);

    waitTime(500);
    setHorizontalSlidePos(HORIZONTAL_SLIDE_IN);
  }

  public double getHeading() {
    return drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
  }

  public void setDriveTrainPower(double frPow, double flPow, double brPow, double blPow) {
    drive.rightFront.setPower(frPow);
    drive.leftFront.setPower(flPow);
    drive.rightBack.setPower(brPow);
    drive.leftBack.setPower(blPow);
  }

  public void turnByGyro(double targetDegrees) {

    double headingError = targetDegrees - getHeading();
    // Normalize the error to be within +/- 180 degrees
    while (headingError > 180) {
      headingError -= 360;
    }
    while (headingError <= -180) {
      headingError += 360;
    }

    int x = 0;
    ElapsedTime timer = new ElapsedTime();
    // keep looping while we are still active, and not on heading.
    // Max time: 1/2 second
    while (this.opMode.opModeIsActive() && x < 5 && timer.milliseconds() < 500) {

      headingError = targetDegrees - getHeading();

      // Normalize the error to be within +/- 180 degrees
      while (headingError > 180) {
        headingError -= 360;
      }
      while (headingError <= -180) {
        headingError += 360;
      }

      double turnSpeed = Range.clip(headingError * GYRO_TURN_P, -0.6, 0.6);
      this.setDriveTrainPower(turnSpeed, -turnSpeed, turnSpeed, -turnSpeed);

      if (Math.abs(headingError) <= HEADING_THRESHOLD) {
        x++;
      } else {
        x = 0;
      }

      opMode.telemetry.addData("target", targetDegrees);
      opMode.telemetry.addData("cur", getHeading());
      opMode.telemetry.addData("error", headingError);
      opMode.telemetry.addData("speed", turnSpeed);
      opMode.telemetry.update();
    }

    this.setDriveTrainPower(0, 0, 0, 0);
  }

  public void waitTime(double ms) {
    double startTime = System.currentTimeMillis();
    while (opMode.opModeIsActive() && System.currentTimeMillis() - startTime < ms) {
    }
  }

  public int distanceToEncoderTicks(double distanceMM) {
    double circumference = Math.PI * 96;
    double cpr = 537.7;
    double ticksPerMM = cpr / circumference;
    return (int) (ticksPerMM * distanceMM);
  }

}