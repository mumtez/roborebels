package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Robot {

  private final LinearOpMode opMode;
  private int counter;
  //public final RevColorSensorV3 pixelSensor;
  //public final DistanceSensor distanceSensor;
  public final IMU imu;
  public final DcMotor fl, fr, bl, br;
  public final DcMotor slideL, slideR;
  public final DcMotor intake;
  public final Servo plane, gateFlip, pixelPull, pixelPullFront;

  public static double DIST_P = .1;
  public static double DIST_THRESH = 1.5; // cm
  public static double DIST_TARG = 20; // cm

  public static double GYRO_TURN_P_GAIN = .04;
  public static double HEADING_THRESHOLD = 1;

  public Robot(LinearOpMode opMode) {
    this.opMode = opMode;
    HardwareMap hardwareMap = opMode.hardwareMap;

    // BULK CACHING
//    List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//    for (LynxModule hub : allHubs) {
//      hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//    }

    // IMU
    imu = hardwareMap.get(IMU.class, "imu");
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        LogoFacingDirection.RIGHT,
        RevHubOrientationOnRobot.UsbFacingDirection.UP));
    imu.initialize(parameters);
    imu.resetYaw();

    // Sensors
    //distanceSensor = hardwareMap.get(DistanceSensor.class, "dist");
    //RevLED led = new RevLED(hardwareMap, "redled", "greenled");
    //led.off();

    counter = 0;

    // Drivetrain
    fl = hardwareMap.dcMotor.get("fl");
    fr = hardwareMap.dcMotor.get("fr");
    bl = hardwareMap.dcMotor.get("bl");
    br = hardwareMap.dcMotor.get("br");

    fl.setMode(RunMode.RUN_WITHOUT_ENCODER);
    fr.setMode(RunMode.RUN_WITHOUT_ENCODER);
    bl.setMode(RunMode.RUN_WITHOUT_ENCODER);
    br.setMode(RunMode.RUN_WITHOUT_ENCODER);

    fl.setDirection(Direction.REVERSE);
    fr.setDirection(Direction.FORWARD);
    bl.setDirection(Direction.REVERSE);
    br.setDirection(Direction.FORWARD);

    fl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    fr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    bl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    br.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

    // Slides
    slideL = hardwareMap.dcMotor.get("sl");
    slideR = hardwareMap.dcMotor.get("sr");

    slideL.setMode(RunMode.STOP_AND_RESET_ENCODER);
    slideR.setMode(RunMode.STOP_AND_RESET_ENCODER);

    slideL.setDirection(Direction.REVERSE);
    slideR.setDirection(Direction.FORWARD);

    slideL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    slideR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

    // Intake
    intake = hardwareMap.dcMotor.get("intake");

    intake.setMode(RunMode.STOP_AND_RESET_ENCODER);
    intake.setDirection(Direction.FORWARD);
    intake.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

    // Servos
    plane = hardwareMap.servo.get("plane");
    gateFlip = hardwareMap.servo.get("gate");
    pixelPull = hardwareMap.servo.get("pixel");
    pixelPullFront = hardwareMap.servo.get("front");

    gateFlip.setPosition(0);
    pixelPullFront.setPosition(0.78);
    pixelPull.setPosition(0.22);

    // pixelSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColour");
    // pixelSensor.enableLed(true);
  }

  public void setSlidePower(double pow) {
    slideL.setPower(pow);
    slideR.setPower(pow);
  }

  public void setSlidePos(int pos, double pow) {
    setSlidePower(0);

    slideL.setTargetPosition(pos);
    slideR.setTargetPosition(pos);

    slideL.setMode(RunMode.RUN_TO_POSITION);
    slideR.setMode(RunMode.RUN_TO_POSITION);

    setSlidePower(pow);

    while (this.opMode.opModeIsActive()
        && Math.abs(slideL.getCurrentPosition() - pos) > 30
        && Math.abs(slideR.getCurrentPosition() - pos) > 30) {
      // Wait for slide to end
    }
  }

  public double getHeading() {
    return this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
  }

  public void holdPosition(double pow) {
    fr.setTargetPosition(fr.getCurrentPosition());
    fl.setTargetPosition(fl.getCurrentPosition());
    br.setTargetPosition(br.getCurrentPosition());
    bl.setTargetPosition(bl.getCurrentPosition());
    fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.setDriveTrainPower(pow, pow, pow, pow);
  }

  public void encodeDriveForward(double disto, double y) {
    int targetTicks = distanceToEncoderTicks(disto);
    fr.setTargetPosition(targetTicks + fr.getCurrentPosition());
    fl.setTargetPosition(targetTicks + fl.getCurrentPosition());
    br.setTargetPosition(targetTicks + br.getCurrentPosition());
    bl.setTargetPosition(targetTicks + bl.getCurrentPosition());

    fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    setDriveTrainPower(y, y, y, y);
    opMode.telemetry.addData("fr pos", fr.getCurrentPosition());
    opMode.telemetry.update();
  }

  public void setDriveTrainPower(double frPow, double flPow, double brPow, double blPow) {
    fr.setPower(frPow);
    fl.setPower(flPow);
    br.setPower(brPow);
    bl.setPower(blPow);
  }

  public void encodeDriveStrafe(double disto, double x) {
    int targetTicks = distanceToEncoderTicks(disto);
    fr.setTargetPosition(-targetTicks + fr.getCurrentPosition());
    fl.setTargetPosition(targetTicks + fl.getCurrentPosition());
    br.setTargetPosition(targetTicks + br.getCurrentPosition());
    bl.setTargetPosition(-targetTicks + bl.getCurrentPosition());

    fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    setDriveTrainPower(x, x, x, x);

    while (opMode.opModeIsActive() && fr.isBusy() && fl.isBusy() && br.isBusy() && bl.isBusy()) {
      // Wait for drive to end
    }

    setDriveTrainPower(0, 0, 0, 0);
  }

  public void turnByGyro(double targetDegrees) {
    this.fr.setMode(RunMode.RUN_WITHOUT_ENCODER);
    this.fl.setMode(RunMode.RUN_WITHOUT_ENCODER);
    this.br.setMode(RunMode.RUN_WITHOUT_ENCODER);
    this.bl.setMode(RunMode.RUN_WITHOUT_ENCODER);

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
    while (this.opMode.opModeIsActive() && x < 10 && timer.milliseconds() < 3000) {

      headingError = targetDegrees - getHeading();

      // Normalize the error to be within +/- 180 degrees
      while (headingError > 180) {
        headingError -= 360;
      }
      while (headingError <= -180) {
        headingError += 360;
      }

      double turnSpeed = Range.clip(headingError * GYRO_TURN_P_GAIN, -1, 1);

      // Clip the speed to the maximum permitted value.
      turnSpeed = Range.clip(turnSpeed, -.6, .6);
      this.setDriveTrainPower(turnSpeed, -turnSpeed, turnSpeed, -turnSpeed);

      if (Math.abs(headingError) < HEADING_THRESHOLD) {
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

  public void spitPixel() {
    this.flipperControl(true);
    this.setIntakePos(-100, .1);
    this.waitTime(100);
  }

  public void setIntakePos(int pos, double pow) {
    intake.setMode(RunMode.STOP_AND_RESET_ENCODER);
    intake.setTargetPosition(pos);

    intake.setMode(RunMode.RUN_TO_POSITION);

    intake.setPower(pow);

    while (this.opMode.opModeIsActive() && intake.isBusy()) {
      // Wait for slide to end
    }
  }

  public void toggleDoor(boolean open) {
    if (open) {
      gateFlip.setPosition(.6);
    } else {
      gateFlip.setPosition(0);
    }
  }

  /*public int getPixel() {
    if (pixelSensor.getDistance(DistanceUnit.CM) < 5) {
      counter++;
    }
    return counter;
  }*/

  /*public void driveToBoard() {
    double error = Math.abs(distanceSensor.getDistance(DistanceUnit.CM) - DIST_TARG);
    if ((Math.abs(distanceSensor.getDistance(DistanceUnit.CM) - DIST_TARG)) > DIST_THRESH) {
      this.setDriveTrainPower(DIST_P * error, DIST_P * error, DIST_P * error, DIST_P * error);
    } else {
      this.setDriveTrainPower(0, 0, 0, 0);
    }


  }*/

  public void flipperControl(boolean x) {
    if (!x) {
      pixelPullFront.setPosition(0.78);
      pixelPull.setPosition(0.22);
    } else {
      pixelPull.setPosition(0.7);
      pixelPullFront.setPosition(0.3);
    }
  }

  public void fly() {
    plane.setPosition(1);
  }
}
