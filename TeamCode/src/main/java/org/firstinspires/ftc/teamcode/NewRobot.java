package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import java.util.List;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlides;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class NewRobot {

  public enum AllianceColor {
    RED, BLUE
  }

  private final LinearOpMode opMode;

  public Follower follower;
  public DcMotor fr, fl, br, bl;
  public IMU imu;

  public final Claw claw;
  public final Intake intake;
  public final VerticalSlides slides;
  public final HorizontalSlides horSlide;

  private final AllianceColor allianceColor;  //0 red 1 blue

  public NewRobot(LinearOpMode opMode) {
    this(opMode, AllianceColor.RED);
  }

  public NewRobot(LinearOpMode opMode, AllianceColor allianceColor) {
    this.opMode = opMode;
    this.allianceColor = allianceColor;
    HardwareMap hardwareMap = opMode.hardwareMap;

    // From https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
    List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule hub : allHubs) {
      hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }

    follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

    fl = hardwareMap.dcMotor.get("fl");
    fr = hardwareMap.dcMotor.get("fr");
    bl = hardwareMap.dcMotor.get("bl");
    br = hardwareMap.dcMotor.get("br");

    fl.setDirection(Direction.REVERSE);
    fr.setDirection(Direction.FORWARD);
    bl.setDirection(Direction.REVERSE);
    br.setDirection(Direction.FORWARD);

    fl.setMode(RunMode.RUN_WITHOUT_ENCODER);
    fr.setMode(RunMode.RUN_WITHOUT_ENCODER);
    bl.setMode(RunMode.RUN_WITHOUT_ENCODER);
    br.setMode(RunMode.RUN_WITHOUT_ENCODER);

    fl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    fr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    bl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    br.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

    imu = hardwareMap.get(IMU.class, "imu");
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        LogoFacingDirection.RIGHT,
        UsbFacingDirection.UP));
    imu.initialize(parameters);
    opMode.telemetry.addData("IMU Initialized", true);
    opMode.telemetry.update();

    // CLAW / INTAKE / SLIDES
    claw = new Claw(opMode);
    intake = new Intake(opMode);
    slides = new VerticalSlides(opMode);
    horSlide = new HorizontalSlides(opMode);
  }

  public void initAuton() {
    slides.setMode(RunMode.STOP_AND_RESET_ENCODER);
    horSlide.setMode(RunMode.STOP_AND_RESET_ENCODER);

    horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
    slides.setTarget(VerticalSlides.TRANSFER);

    claw.clawClose();
    claw.setInit();

    this.intake.rotateFlat();
  }

  public void initAutonSpec() {
    slides.setMode(RunMode.STOP_AND_RESET_ENCODER);
    horSlide.setMode(RunMode.STOP_AND_RESET_ENCODER);

    horSlide.setTarget(HorizontalSlides.TRANSFER_POS);
    slides.setTarget(VerticalSlides.TRANSFER);

    claw.clawClose();
    claw.setInitSpec();

    this.intake.rotateFlat();
  }

  public AllianceColor getAllianceColor() {
    return this.allianceColor;
  }

  public void waitTime(long ms) {
    long startTime = System.currentTimeMillis();

    while (this.opMode.opModeIsActive() && System.currentTimeMillis() - startTime < ms) {
      follower.update();
      slides.updatePIDControl();
      horSlide.updatePosition();
      horSlide.updatePIDControl();
    }
  }
}