package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.sensors.LVMaxSonarEZ;
import org.firstinspires.ftc.teamcode.sensors.RevLED;


@Config
public class Robot {

  public static double slideOutDist = 0.8;
  public static double slideInDist = 0.8;

  public static double outtakeIn = 0;
  public static double outtakeOut = 0.8;

  public static double GYRO_TURN_P = .055;
  public static double HEADING_THRESHOLD = 1;

  public final IMU imu;
  public final DcMotor fl, fr, bl, br;
  public final DcMotor slideUP;
  public final ServoImplEx slideOUT;
  public final ServoImplEx intake;
  public final ServoImplEx flipper; //, outtake;

  //public final DistanceSensor  intakeSense;


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
        LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
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
    slideUP = hardwareMap.dcMotor.get("su");

    slideOUT = (ServoImplEx) hardwareMap.servo.get("so");
    //slideOUT

    slideUP.setMode(RunMode.STOP_AND_RESET_ENCODER);
    //slideOUT.setMode(RunMode.STOP_AND_RESET_ENCODER);

    slideUP.setDirection(Direction.REVERSE);
    //slideOUT.setDirection(Direction.FORWARD);

    slideUP.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    //slideOUT.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

    slideUP.setMode(RunMode.RUN_WITHOUT_ENCODER);



    // Intake

    intake = (ServoImplEx) hardwareMap.servo.get("in");
    //outtake = (ServoImplEx) hardwareMap.servo.get("out");
    flipper = (ServoImplEx) hardwareMap.servo.get("flip");

    flipper.setDirection(Servo.Direction.REVERSE);

    // Sensor
    //intakeSense = hardwareMap.get(DistanceSensor.class, "ins");

  }

  public void flipOut(){
    //outtake.setPosition(outtakeOut);
  }

  public void flipIn(){
    //flipper.setPosition(outtakeIn);
  }



  public void setSlidePower(double pow) {
    slideUP.setPower(pow);
  }


  public void slideOutPush(double dir){
    slideOUT.setPosition(slideOUT.getPosition() + (dir/100));
  }


  public void setSlideUpPos(int pos, double pow) {
    setSlidePower(0);

    slideUP.setTargetPosition(pos);

    slideUP.setMode(RunMode.RUN_TO_POSITION);

    setSlidePower(pow);

    while (this.opMode.opModeIsActive() && Math.abs(slideUP.getCurrentPosition() - pos) > 30)
    {
      // Wait for slide to end
    }

    slideUP.setMode(RunMode.RUN_WITHOUT_ENCODER);

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

    while (opMode.opModeIsActive() && fr.isBusy() && fl.isBusy() && br.isBusy() && bl.isBusy()) {
      // Do Nothing
    }

    setDriveTrainPower(0, 0, 0, 0);

  }

  public int distanceToEncoderTicks(double distanceMM) {
    double circumference = Math.PI * 96;
    double cpr = 537.7;
    double ticksPerMM = cpr / circumference;
    return (int) (ticksPerMM * distanceMM);
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

  public enum DIRECTION {
    LEFT, RIGHT, FORWARD, BACKWARD
  }
}