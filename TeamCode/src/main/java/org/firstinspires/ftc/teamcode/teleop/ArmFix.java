package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NewRobot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

@Config
@TeleOp(name = "Arm Fix")
public class ArmFix extends LinearOpMode {


  @Override
  public void runOpMode() throws InterruptedException {
    NewRobot robot = new NewRobot(this);

    waitForStart();

    // LOOP
    while (opModeIsActive()) {
      if (gamepad1.cross) {
        robot.claw.clawUpArm.setPosition(Claw.upArmTransfer);
      }
    }
  }
}