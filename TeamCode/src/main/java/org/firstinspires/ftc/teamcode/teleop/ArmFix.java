package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NewRobot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

@TeleOp(name = "Arm Fix")
public class ArmFix extends LinearOpMode {


  @Override
  public void runOpMode() throws InterruptedException {
    NewRobot robot = new NewRobot(this);

    waitForStart();

    // LOOP
    while (opModeIsActive()) {
      robot.claw.clawUpArm.setPosition(Claw.upArmTransfer);
    }
  }
}