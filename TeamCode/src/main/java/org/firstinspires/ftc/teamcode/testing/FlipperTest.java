package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "Flip Test", group = "TESTING")
public class FlipperTest extends LinearOpMode {

  Robot robot;

  public static double flipPos = 0.5;

  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);

    waitForStart();
    // START

    // LOOP
    while (opModeIsActive()) {

      robot.flipper.setPosition(flipPos);

    }
  }
}