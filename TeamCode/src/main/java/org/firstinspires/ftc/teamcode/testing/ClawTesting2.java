package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NewRobot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides;

@Config
@TeleOp(name = "Claw Test 2.0", group = "TESTING")
public class ClawTesting2 extends LinearOpMode {

  NewRobot robot;
  public static double INTAKE_ROT = Intake.INTAKE_FLAT;
  public static double CLAW_POS = Claw.clawOpenPos;
  public static double CLAW_UP_ARM_POS = Claw.upArmInit;
  public static double CLAW_DOWN_ARM_POS = Claw.downArmInit;
  public static double WRIST_POS = Claw.wristDefault;
  public static double SWEEP_POS = Intake.SWEEP_IN;


  public static int SLIDE_POS = VerticalSlides.TRANSFER;

  @Override
  public void runOpMode() throws InterruptedException {
    robot = new NewRobot(this);

    waitForStart();
    // LOOP
    while (opModeIsActive()) {
      robot.intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
      robot.intake.rotate.setPosition(INTAKE_ROT);
      robot.slides.setTarget(SLIDE_POS);
      robot.slides.updatePIDControl();

      robot.claw.claw.setPosition(CLAW_POS);
      robot.claw.clawUpArm.setPosition(CLAW_UP_ARM_POS);
      robot.claw.clawDownArm.setPosition(CLAW_DOWN_ARM_POS);

      robot.intake.sweep.setPosition(SWEEP_POS);

      //robot.intake.sweepOut(gamepad2.circle);
    }
  }
}