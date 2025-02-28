package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.NewRobot;
import org.firstinspires.ftc.teamcode.NewRobot.AllianceColor;
import org.firstinspires.ftc.teamcode.auton.baseAutons.BaseBucketAuton;
//import org.firstinspires.ftc.teamcode.auton.baseAutons.BaseBucketAuton;

@Autonomous(name = "BLUE BUCKET", group = "PEDRO")
public class BlueBucketAuton extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    NewRobot robot = new NewRobot(this, AllianceColor.BLUE, true);
    new BaseBucketAuton(this, robot).run();
  }
}
