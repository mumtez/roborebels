package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Robot.AllianceColor;

@Autonomous(name = "RED BUCKET", group = "PEDRO")
public class RedBucketAuton extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    Robot robot = new Robot(this, AllianceColor.RED, true);
    new BaseBucketAuton(this, robot).run();
  }

}
