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
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

// CONFIG

@Config
public class Robot {


  public static int VERTICAL_SLIDE_UP = 2800;
  public static int VERTICAL_SLIDE_DEFAULT = 400;
  public static int VERTICAL_SLIDE_PRE_TRANSFER = 800;
  public static double KG = 0.07;

  // TODO: tune color sensor gain
  // public static float INTAKE_COLOR_GAIN = 2;

  public final Follower follower;

  public final Claw claw;
  public final Intake intake;

  public final DcMotor slideLeft;
  public final DcMotor slideRight;

  public final DcMotor hang;

  //public final NormalizedColorSensor intakeColor;

  public final LinearOpMode opMode;

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
    // TODO: add magnetic limit switch to bottom out encoder vals
    slideLeft = hardwareMap.dcMotor.get("lu");
    slideRight = hardwareMap.dcMotor.get("ru");
    slideLeft.setDirection(Direction.REVERSE);
    slideRight.setDirection(Direction.FORWARD);
    slideLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    slideRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    slideLeft.setMode(RunMode.RUN_WITHOUT_ENCODER);
    slideRight.setMode(RunMode.RUN_WITHOUT_ENCODER);


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

  }

  public void initAuton() {
    slideLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
    slideRight.setMode(RunMode.STOP_AND_RESET_ENCODER);

    claw.clawClose();
    claw.setInit();

    this.intake.rotateDown();
    this.intake.setHorizontalSlidePos(Intake.SLIDE_TRANSFER);
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

  public void endSlideUpPos(int pos) {

    while (this.opMode.opModeIsActive() && Math.abs(slideLeft.getCurrentPosition() - pos) > 30) {
      // Wait for slide to end
    }

    setVerticalSlidePower(0);

    slideLeft.setMode(RunMode.RUN_WITHOUT_ENCODER);
    slideRight.setMode(RunMode.RUN_WITHOUT_ENCODER);
  }

  public void waitTime(double ms) {
    double startTime = System.currentTimeMillis();
    while (opMode.opModeIsActive() && System.currentTimeMillis() - startTime < ms) {
    }
  }
}