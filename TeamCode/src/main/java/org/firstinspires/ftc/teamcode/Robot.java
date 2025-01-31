package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class Robot {

  public enum AllianceColor {
    RED, BLUE
  }

  private final LinearOpMode opMode;

  public final Follower follower;
  public final Claw claw;
  public final Intake intake;
  public final VerticalSlides slides;

  public final DcMotor hang;

  private AllianceColor allianceColor;  //0 red 1 blue

  public Robot(LinearOpMode opMode) {
    this(opMode, AllianceColor.RED);
  }

  public Robot(LinearOpMode opMode, AllianceColor allianceColor) {
    this.opMode = opMode;
    this.allianceColor = allianceColor;
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
    slides = new VerticalSlides(opMode);

    // HANG
    hang = hardwareMap.dcMotor.get("hang");
    hang.setMode(RunMode.RUN_WITHOUT_ENCODER);
    hang.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
  }

  public void initAuton() {
    slides.setMode(RunMode.STOP_AND_RESET_ENCODER);

    claw.clawClose();
    claw.setInit();

    this.intake.rotateDown();
    this.intake.setHorizontalSlidePos(Intake.SLIDE_TRANSFER);
  }

  public void setAllianceColor(AllianceColor allianceColor) {
    this.allianceColor = allianceColor;
  }

  public AllianceColor getAllianceColor() {
    return this.allianceColor;
  }

  // TODO: add method to return true if slides are within a threshold of a position
  public void waitTime(double ms) {
    double startTime = System.currentTimeMillis();
    while (opMode.opModeIsActive() && System.currentTimeMillis() - startTime < ms) {
    }
  }
}