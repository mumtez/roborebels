package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.NewRobot;
import org.firstinspires.ftc.teamcode.NewRobot.AllianceColor;
import org.firstinspires.ftc.teamcode.auton.baseAutons.BaseSpecAuton;

@Autonomous(name = "RED SPEC", group = "PEDRO")
public class RedSpecAuton extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    NewRobot robot = new NewRobot(this, AllianceColor.RED, true);
    new BaseSpecAuton(this, robot).run();
  }

}
