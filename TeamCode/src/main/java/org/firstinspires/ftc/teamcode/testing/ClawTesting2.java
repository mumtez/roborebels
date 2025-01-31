package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides;

@Config
@TeleOp(name = "Claw Test 2.0", group = "TESTING")
public class ClawTesting2 extends LinearOpMode {

  Robot robot;
  public static double INTAKE_ROT = Intake.INTAKE_FLAT;
  public static double HSLIDE_POS = Intake.SLIDE_TRANSFER;
  public static double CLAW_POS = Claw.clawOpenPos;
  public static double CLAW_UP_ARM_POS = Claw.upArmInit;
  public static double CLAW_DOWN_ARM_POS = Claw.downArmInit;
  public static int VSLIDE_TARGET = VerticalSlides.DEFAULT;

  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);
    robot.slides.setMode(RunMode.STOP_AND_RESET_ENCODER);
    waitForStart();
    robot.slides.setMode(RunMode.RUN_WITHOUT_ENCODER);

    // LOOP
    while (opModeIsActive()) {
      robot.intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
      robot.intake.setHorizontalSlidePos(HSLIDE_POS);
      robot.intake.rotate.setPosition(INTAKE_ROT);

      robot.claw.claw.setPosition(CLAW_POS);
      robot.claw.clawUpArm.setPosition(CLAW_UP_ARM_POS);
      robot.claw.clawDownArm.setPosition(CLAW_DOWN_ARM_POS);

      robot.slides.setTarget(VSLIDE_TARGET);
      robot.slides.updatePIDControl();
    }
  }
}