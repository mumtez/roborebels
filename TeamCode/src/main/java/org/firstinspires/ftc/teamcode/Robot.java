package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import java.util.List;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
public class Robot {

  public static int VERTICAL_SLIDE_UP = 2800;
  public static int VERTICAL_SLIDE_DEFAULT = 400;
  public static int VERTICAL_SLIDE_PRE_TRANSFER = 800;
  public static double KG = 0.07;

  private final LinearOpMode opMode;

  public final Follower follower;
  public final Claw claw;
  public final Intake intake;

  public final DcMotor slideLeft;
  public final DcMotor slideRight;
  public final DcMotor hang;

  public final TouchSensor maglim;

  public final Servo rgb;

  public int team_color = 0;  //0 red 1 blue

  public Robot(LinearOpMode opMode) {
    this.opMode = opMode;
    HardwareMap hardwareMap = opMode.hardwareMap;
    Constants.setConstants(FConstants.class, LConstants.class);

    // From https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
    List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule hub : allHubs) {
      hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }

    // FOLLOWER (Pedro Pathing)
    follower = new Follower(hardwareMap);

    // CLAW / INTAKE
    claw = new Claw(opMode);
    intake = new Intake(opMode);

    // HANG
    hang = hardwareMap.dcMotor.get("hang");
    hang.setMode(RunMode.RUN_WITHOUT_ENCODER);
    hang.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

    // VERTICAL SLIDES
    slideLeft = hardwareMap.dcMotor.get("lu");
    slideRight = hardwareMap.dcMotor.get("ru");
    slideLeft.setDirection(Direction.FORWARD);
    slideRight.setDirection(Direction.REVERSE);
    slideLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    slideRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    slideLeft.setMode(RunMode.RUN_WITHOUT_ENCODER);
    slideRight.setMode(RunMode.RUN_WITHOUT_ENCODER);

    // TODO: wire + use magnetic limit switch to offset slide encoder vals
    maglim = hardwareMap.touchSensor.get("mag");

    // TODO: use color beacon to show which color is in the intake
    rgb = hardwareMap.servo.get("rgb");
  }

  public void initAuton() {
    slideLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
    slideRight.setMode(RunMode.STOP_AND_RESET_ENCODER);

    claw.clawClose();
    claw.setInit();

    this.intake.rotateDown();
    this.intake.setHorizontalSlidePos(Intake.SLIDE_TRANSFER);
  }

  public void setVerticalSlideMode(RunMode mode) {
    slideLeft.setMode(mode);
    slideRight.setMode(mode);
  }

  public void setVerticalSlidePower(double pow) {
    slideRight.setPower(pow);
    slideLeft.setPower(pow);
  }

  // TODO: use a custom PID for improved performance
  public void startSlideUpPos(int pos, double pow) {
    setVerticalSlidePower(0);

    slideLeft.setTargetPosition(pos);
    slideRight.setTargetPosition(pos);

    slideLeft.setMode(RunMode.RUN_TO_POSITION);
    slideRight.setMode(RunMode.RUN_TO_POSITION);
    setVerticalSlidePower(pow);
  }

  // TODO: add method to return true if slides are within a threshold of a position

  public void waitTime(double ms) {
    double startTime = System.currentTimeMillis();
    while (opMode.opModeIsActive() && System.currentTimeMillis() - startTime < ms) {
    }
  }
}