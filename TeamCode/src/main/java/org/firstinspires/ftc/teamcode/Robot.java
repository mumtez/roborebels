package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.sensors.LVMaxSonarEZ;
import org.firstinspires.ftc.teamcode.sensors.RevLED;

@Config
public class Robot {

  public static double BACKDROP_DIST_P = 0.03;
  public static double BACKDROP_DIST_THRESH = 1; // cm
  public static double BACKDROP_DIST = 15; // cm
  public static double STACK_DIST_P = 0.03;
  public static double STACK_DIST_THRESH = 1; // cm
  public static double STACK_DIST = 16; // cm
  public static double ULTRASONIC_DIST_P = 0.05;
  public static double ULTRASONIC_DIST_THRESH = 1.5; // cm
  public static double ULTRASONIC_DIST = 20; // cm
  public static double GYRO_TURN_P = .055;
  public static double HEADING_THRESHOLD = 1;

  public static double LEFT_CLAW_CLOSED = 0;
  public static double LEFT_CLAW_OPEN = 1;

  public static double RIGHT_CLAW_CLOSED = 1;
  public static double RIGHT_CLAW_OPEN = 0;

  public static double GATE_CLOSED = 0;
  public static double GATE_OPEN = 1;

  public static double SWEEP_OUT = 0;
  public static double SWEEP_IN = 1;

  public final IMU imu;
  public final DcMotor fl, fr, bl, br;
  public final DcMotor slideL, slideR;
  public final DcMotor intake;
  public final ServoImplEx plane, gate, pixelClawLeft, pixelClawRight, sweeper;
  public final DistanceSensor backdropSensor;
  public final LVMaxSonarEZ ultrasonicLeft, ultrasonicRight, stackSensor;
  public final RevLED led;

  private final LinearOpMode opMode;

  public Robot(LinearOpMode opMode) {
    this.opMode = opMode;
    HardwareMap hardwareMap = opMode.hardwareMap;

    // BULK CACHING
    List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule hub : allHubs) {
      hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }

    // IMU
    imu = hardwareMap.get(IMU.class, "imu");
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        LogoFacingDirection.RIGHT,
        RevHubOrientationOnRobot.UsbFacingDirection.UP));
    imu.initialize(parameters);
    imu.resetYaw();

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
    intake.setMode(RunMode.RUN_WITHOUT_ENCODER);
    intake.setDirection(Direction.FORWARD);
    intake.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

    // Servos
    plane = (ServoImplEx) hardwareMap.servo.get("plane");

    gate = (ServoImplEx) hardwareMap.servo.get("gate");
    gate.setPwmRange(new PwmRange(500, 2500));

    pixelClawLeft = (ServoImplEx) hardwareMap.servo.get("left claw");
    pixelClawLeft.setPwmRange(new PwmRange(500, 2500));

    pixelClawRight = (ServoImplEx) hardwareMap.servo.get("right claw");
    pixelClawRight.setPwmRange(new PwmRange(500, 2500));

    sweeper = (ServoImplEx) hardwareMap.servo.get("sweeper");
    sweeper.setPwmRange(new PwmRange(500, 2500));

    // TODO: set a starting plane position
    //plane.setPosition(0);
    gate.setPosition(GATE_OPEN);
    pixelClawRight.setPosition(RIGHT_CLAW_CLOSED);
    pixelClawLeft.setPosition(LEFT_CLAW_CLOSED);
    sweeper.setPosition(SWEEP_IN);

    // Sensors
    backdropSensor = hardwareMap.get(DistanceSensor.class, "backdrop");
    stackSensor = new LVMaxSonarEZ(hardwareMap.analogInput.get("stack"));
    ultrasonicLeft = new LVMaxSonarEZ(hardwareMap.analogInput.get("usl"));
    ultrasonicRight = new LVMaxSonarEZ(hardwareMap.analogInput.get("usr"));

    led = new RevLED(hardwareMap, "red", "green");
    led.green();

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

    setSlidePower(0);
  }

  public double getHeading() {
    return this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
  }

  public void setDriveTrainPower(double frPow, double flPow, double brPow, double blPow) {
    fr.setPower(frPow);
    fl.setPower(flPow);
    br.setPower(brPow);
    bl.setPower(blPow);
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

  public void setGate(boolean open) {
    if (open) {
      gate.setPosition(GATE_OPEN);
    } else {
      gate.setPosition(GATE_CLOSED);
    }
  }

  public void setSweepOut(boolean out) {
    if (out) {
      sweeper.setPosition(SWEEP_OUT);
    } else {
      sweeper.setPosition(SWEEP_IN);
    }
  }

  public void driveToStack() {
    this.driveByDistanceSensor(stackSensor, STACK_DIST_P, STACK_DIST, STACK_DIST_THRESH,
        DIRECTION.FORWARD);
  }

  public void driveToBackdrop() {
    this.driveByDistanceSensor(backdropSensor, BACKDROP_DIST_P, BACKDROP_DIST, BACKDROP_DIST_THRESH,
        DIRECTION.BACKWARD);
  }

  public void driveToLeftWall() {
    this.driveByDistanceSensor(ultrasonicLeft, ULTRASONIC_DIST_P, ULTRASONIC_DIST,
        ULTRASONIC_DIST_THRESH,
        DIRECTION.LEFT);
  }

  public void driveToRightWall() {
    this.driveByDistanceSensor(ultrasonicRight, ULTRASONIC_DIST_P, ULTRASONIC_DIST,
        ULTRASONIC_DIST_THRESH,
        DIRECTION.RIGHT);
  }

  public void driveByDistanceSensor(DistanceSensor distanceSensor, double pVal, double targetDistCm,
      double thresholdCm, DIRECTION direction) {
    this.fr.setMode(RunMode.RUN_WITHOUT_ENCODER);
    this.fl.setMode(RunMode.RUN_WITHOUT_ENCODER);
    this.br.setMode(RunMode.RUN_WITHOUT_ENCODER);
    this.bl.setMode(RunMode.RUN_WITHOUT_ENCODER);

    double error;
    ElapsedTime timeout = new ElapsedTime();
    int x = 0;
    do {
      double distance = distanceSensor.getDistance(DistanceUnit.CM);
      error = distance - targetDistCm;
      double p = Range.clip(pVal * error, -0.6, 0.6);
      switch (direction) {
        case LEFT:
          this.setDriveTrainPower(p, -p, -p, p);
          break;
        case RIGHT:
          this.setDriveTrainPower(-p, p, p, -p);
          break;
        case FORWARD:
          this.setDriveTrainPower(p, p, p, p);
          break;
        case BACKWARD:
          this.setDriveTrainPower(-p, -p, -p, -p);
          break;
      }

      if (Math.abs(error) < STACK_DIST_THRESH) {
        x++;
      } else {
        x = 0;
      }

      this.opMode.telemetry.addData("distance cm:", distance);
      this.opMode.telemetry.addData("error:", error);
      this.opMode.telemetry.addData("power:", p);
      this.opMode.telemetry.update();
    } while (this.opMode.opModeIsActive() && Math.abs(error) > thresholdCm && x < 5
        && timeout.milliseconds() < 1000);

    this.setDriveTrainPower(0, 0, 0, 0);
  }

  public void flipperControl(boolean open) {
    if (open) {
      pixelClawLeft.setPosition(LEFT_CLAW_OPEN);
      pixelClawRight.setPosition(RIGHT_CLAW_OPEN);
    } else {
      pixelClawLeft.setPosition(LEFT_CLAW_CLOSED);
      pixelClawRight.setPosition(RIGHT_CLAW_CLOSED);
    }
  }

  public void fly() {
    plane.setPosition(1);
  }

  public enum DIRECTION {
    LEFT, RIGHT, FORWARD, BACKWARD
  }
}