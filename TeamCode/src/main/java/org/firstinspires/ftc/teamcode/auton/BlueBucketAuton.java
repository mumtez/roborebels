package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Robot.AllianceColor;

@Autonomous(name = "BLUE BUCKET", group = "PEDRO")
public class BlueBucketAuton extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this, AllianceColor.BLUE, true);
    new BaseBucketAuton(this, robot).run();
  }
}
