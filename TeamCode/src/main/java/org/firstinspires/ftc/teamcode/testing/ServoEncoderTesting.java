package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.NewRobot;

@Disabled
@TeleOp(name = "Servo Encoder Test", group = "TESTING")
public class ServoEncoderTesting extends LinearOpMode {

  //get our analog input from the hardwareMap
  AnalogInput analogInput;


  double position;


  NewRobot robot;


  @Override
  public void runOpMode() throws InterruptedException {
    robot = new NewRobot(this);
    analogInput = hardwareMap.get(AnalogInput.class, "myanaloginput");

    while (opModeIsActive()) {

      // get the voltage of our analog line
      // divide by 3.3 (the max voltage) to get a value between 0 and 1
      // multiply by 360 to convert it to 0 to 360 degrees
      position = analogInput.getVoltage() / 3.3 * 360;

      telemetry.addData("Position", position);
    }
  }
}