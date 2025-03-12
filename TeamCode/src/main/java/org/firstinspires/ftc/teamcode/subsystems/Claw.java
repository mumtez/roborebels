package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

// CONFIG

@Config
public class Claw {

  // might be updated
  public static double clawOpenPos = 0.5;
  public static double clawClosePos = 0.62;

  public static double upArmPlaceTeleOP = .5;

  public static double upArmPlaceAuto = 0.7;

  public static double upArmWallAuto = 0.17;
  public static double upArmBucket = .65;

  public static double upArmTransfer = 0.19;
  public static double upArmInit = 0.19;


  public static double downArmPlaceTeleOP = .98;

  public static double downArmPlaceAuto = 0.98;

  public static double downArmWallAuto = 0.48;

  public static double downArmSpecInit = 0.7;
  public static double downArmBucket = 0.95;
  public static double downArmTransfer = 0.14;
  public static double downArmInit = 0.12;


  public static double wristDefault = 0.89;
  public static double wristPlace = 0.89;

  public static double wristPlaceAuto = 0.33;

  public final ServoImplEx claw, clawUpArm, clawDownArm, wrist;
  // TODO: add color/touch/limit sensor (states?)

  private boolean clawClosed = false;

  public Claw(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;

    claw = (ServoImplEx) hardwareMap.servo.get("c");
    clawUpArm = (ServoImplEx) hardwareMap.servo.get("cu");
    clawDownArm = (ServoImplEx) hardwareMap.servo.get("cd");
    wrist = (ServoImplEx) hardwareMap.servo.get("wrist");

  }

  public void clawClose() {
    this.clawClosed = true;
    claw.setPosition(clawClosePos);
  }

  public void clawOpen() {
    this.clawClosed = false;
    claw.setPosition(clawOpenPos);
  }


  public void setPlace() { // needs to be tuned
    clawDownArm.setPosition(downArmPlaceTeleOP);
    clawUpArm.setPosition(upArmPlaceTeleOP);
    wrist.setPosition(wristPlace);
  }

  public void setPlaceAuto() {
    clawDownArm.setPosition(downArmPlaceAuto);
    clawUpArm.setPosition(upArmPlaceAuto);
    wrist.setPosition(wristPlaceAuto);
  }

  public void setWall() {
    clawDownArm.setPosition(downArmWallAuto);
    clawUpArm.setPosition(upArmWallAuto);
    wrist.setPosition(wristDefault);
  }

  public void setInitSpec() {
    clawDownArm.setPosition(downArmSpecInit);
    clawUpArm.setPosition(upArmWallAuto);
    wrist.setPosition(wristPlaceAuto);
  }


  public void setTransfer() {
    clawDownArm.setPosition(downArmTransfer);
    clawUpArm.setPosition(upArmTransfer);
    wrist.setPosition(wristDefault);
  }


  public void setBucket() {
    clawDownArm.setPosition(downArmBucket);
    clawUpArm.setPosition(upArmBucket);
    wrist.setPosition(wristDefault);
  }


  public void setInit() {
    clawDownArm.setPosition(downArmInit);
    clawUpArm.setPosition(upArmInit);
    wrist.setPosition(wristDefault);
  }

  public boolean isClawClosed() {
    return clawClosed;
  }
}