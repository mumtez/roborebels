package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

// CONFIG

@Config
public class Claw {

  public static double clawWallOpenPos = 0.45;
  public static double clawOpenPos = 0.5;
  public static double clawClosePos = 0.6;

  public static double upArmPlace = 0.88;


  public static double upArmWall = 0.9;
  public static double upArmBucket = .65;

  public static double upArmTransfer = 0.19;
  public static double upArmInit = 0.19;
//0.9

  public static double downArmPlace = 0.47;


  public static double downArmWall = 0.95;
  public static double downArmBucket = 0.95;
  public static double downArmTransfer = 0.12;
  public static double downArmInit = 0.12;


  public static double wristDefault = 0.88;
  public static double wristPlace = 0.33;

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

  public void clawOpenWall() {
    this.clawClosed = false;
    claw.setPosition(clawWallOpenPos);
    wrist.setPosition(wristDefault);
  }

  public void setPlace() {
    clawDownArm.setPosition(downArmPlace);
    clawUpArm.setPosition(upArmPlace);
    wrist.setPosition(wristPlace);
  }


  public void setTransfer() {
    clawDownArm.setPosition(downArmTransfer);
    clawUpArm.setPosition(upArmTransfer);
    wrist.setPosition(wristDefault);
  }

  public void setWall() {
    clawDownArm.setPosition(downArmWall);
    clawUpArm.setPosition(upArmWall);
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