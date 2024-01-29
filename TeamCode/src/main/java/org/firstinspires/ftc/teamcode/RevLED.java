package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannel.Mode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RevLED {

  private final DigitalChannel redLED;
  private final DigitalChannel greenLED;

  public RevLED(HardwareMap hardwareMap, String redConfig, String greenConfig) {
    this.redLED = hardwareMap.get(DigitalChannel.class, redConfig);
    this.greenLED = hardwareMap.get(DigitalChannel.class, greenConfig);

    this.redLED.setMode(Mode.OUTPUT);
    this.greenLED.setMode(Mode.OUTPUT);
  }

  public void off() {
    this.redLED.setState(false);
    this.greenLED.setState(false);
  }

  public void red() {
    this.redLED.setState(true);
    this.greenLED.setState(false);
  }

  public void orange() {
    this.redLED.setState(true);
    this.greenLED.setState(true);
  }

  public void green() {
    this.redLED.setState(false);
    this.greenLED.setState(true);
  }
}
