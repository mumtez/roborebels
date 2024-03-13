package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Motor Encoder Test", group = "TESTING")
public class MotorEncoderTest extends LinearOpMode {

  Robot robot;

  @Override
  public void runOpMode() throws InterruptedException {
    robot = new Robot(this);
    waitForStart();
    while (opModeIsActive()) {
//      telemetry.addData("FR", robot.fr.getCurrentPosition());
//      telemetry.addData("FL", robot.fl.getCurrentPosition());
//      telemetry.addData("BR", robot.br.getCurrentPosition());
//      telemetry.addData("BL", robot.bl.getCurrentPosition());
      telemetry.addData("SLIDE L", robot.slideL.getCurrentPosition());
      telemetry.addData("SLIDE R", robot.slideR.getCurrentPosition());
      telemetry.addData("INTAKE", robot.intake.getCurrentPosition());
      telemetry.update();

    }
  }

}