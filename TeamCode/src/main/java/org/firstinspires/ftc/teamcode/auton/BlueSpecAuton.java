package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Robot.AllianceColor;
import org.firstinspires.ftc.teamcode.auton.baseAutons.BaseSpecAuton;

@Autonomous(name = "BLUE SPEC", group = "PEDRO")
public class BlueSpecAuton extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this, AllianceColor.BLUE, true);
    new BaseSpecAuton(this, robot).run();
  }

}
