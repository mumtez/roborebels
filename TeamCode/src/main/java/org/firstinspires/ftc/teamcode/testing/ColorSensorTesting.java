package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.Timer;

@Config

@TeleOp(name = "Color Test", group = "TESTING")
public class ColorSensorTesting extends LinearOpMode {

  Robot robot;
  Timer out;

  public static double COLOR_THRESHOLD = 0.02;
  public static double YELLOW_THRESHOLD = 0.01;


  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);
    out = new Timer();


    waitForStart();
    // START

    // LOOP
    while (opModeIsActive()) {

      if (!robot.intake.validSampleIn(Robot.AllianceColor.BLUE)){
        robot.intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

      }
      /*
      else if(Timer){

      }

       */

      robot.intake.senseColor(); // Important: only make 1 i2c call per loop
      NormalizedRGBA colors = robot.intake.getColors();
      if (colors.blue > COLOR_THRESHOLD) {
        gamepad1.rumble(5000);
        gamepad2.rumble(5000);
      }

      telemetry.addData("Blue in bot", colors.blue > COLOR_THRESHOLD);
      telemetry.addData("Red in bot", colors.red > COLOR_THRESHOLD);
      telemetry.addData("Yellow in bot",
          colors.red > YELLOW_THRESHOLD || colors.blue > YELLOW_THRESHOLD);

      telemetry.addData("Red", colors.red);
      telemetry.addData("Blue", colors.blue);
      telemetry.update();


    }
  }
}