package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NewRobot;
import org.firstinspires.ftc.teamcode.NewRobot.AllianceColor;

@TeleOp(name = "BLUE TELEOP", group = "MAIN")
public class BlueTeleop extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    NewRobot robot = new NewRobot(this, AllianceColor.BLUE);
    new BaseTeleop(this, robot).run();
  }
}
