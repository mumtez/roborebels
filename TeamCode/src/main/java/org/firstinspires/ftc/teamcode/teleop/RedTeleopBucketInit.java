package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NewRobot;
import org.firstinspires.ftc.teamcode.NewRobot.AllianceColor;

@TeleOp(name = "RED TELEOP BUCKET INIT", group = "MAIN")
public class RedTeleopBucketInit extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    NewRobot robot = new NewRobot(this, AllianceColor.RED);
    new BaseTeleop(this, robot, 90).run();
  }
}
