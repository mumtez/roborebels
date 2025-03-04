package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.NewRobot;
import org.firstinspires.ftc.teamcode.NewRobot.AllianceColor;
import org.firstinspires.ftc.teamcode.auton.baseAutons.BaseSpecAuton;

@Autonomous(name = "BLUE SPEC", group = "PEDRO")
public class BlueSpecAuton extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    NewRobot robot = new NewRobot(this, AllianceColor.BLUE, true);
    new BaseSpecAuton(this, robot).run();
  }
}