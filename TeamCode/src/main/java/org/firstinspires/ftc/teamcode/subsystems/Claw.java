package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

// CONFIG

@Config
public class Claw {

  public static double clawWallOpenPos = 0.5;
  public static double clawOpenPos = 0.5;
  public static double clawClosePos = 0.6;

  public static double upArmPlace = 0.7;

  public static double upArmPlace1 = 0.75;
  public static double upArmPlace2 = 0.65;  //maybe 85


  public static double upArmWall = 0.13;
  public static double upArmBucket = .4;
  public static double upArmUnder = 0.6;
  public static double upArmTransfer = 0.88;
  public static double upArmInit = 0.82;
  public static double upArmPostTransfer = 0.88;
//0.9

  public static double downArmPlace = 0.60;
  public static double downArmPlace1 = 0.9;
  public static double downArmPlace2 = 0.60;


  public static double downArmWall = 0.85;
  public static double downArmBucket = 0.8;
  public static double downArmUnder = 0.4;
  public static double downArmTransfer = 0;
  public static double downArmInit = 0.85;

  public static double downArmPostTransfer = 0;

  public final ServoImplEx claw, clawUpArm, clawDownArm;
  // TODO: add color/touch/limit sensor (states?)

  private boolean clawClosed = false;

  public Claw(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;

    claw = (ServoImplEx) hardwareMap.servo.get("c");
    clawUpArm = (ServoImplEx) hardwareMap.servo.get("cu");
    clawDownArm = (ServoImplEx) hardwareMap.servo.get("cd");
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
  }

  public void setPlace() {
    clawDownArm.setPosition(downArmPlace);
    clawUpArm.setPosition(upArmPlace);
  }

  public void setPlaceOne() {
    clawDownArm.setPosition(downArmPlace1);
    clawUpArm.setPosition(upArmPlace1);
  }

  public void setPlaceTwo() {
    clawDownArm.setPosition(downArmPlace2);
    clawUpArm.setPosition(upArmPlace2);
  }

  public void setTransfer() {
    clawDownArm.setPosition(downArmTransfer);
    clawUpArm.setPosition(upArmTransfer);
  }

  public void setTransferClear() {
    clawDownArm.setPosition(downArmPostTransfer);
    clawUpArm.setPosition(upArmPostTransfer);
  }

  public void setWall() {
    clawDownArm.setPosition(downArmWall);
    clawUpArm.setPosition(upArmWall);
  }

  public void setBucket() {
    clawDownArm.setPosition(downArmBucket);
    clawUpArm.setPosition(upArmBucket);
  }

  public void setUnder() {
    clawDownArm.setPosition(downArmUnder);
    clawUpArm.setPosition(upArmUnder);
  }

  public void setInit() {
    clawDownArm.setPosition(downArmInit);
    clawUpArm.setPosition(upArmInit);
  }

  public boolean isClawClosed() {
    return clawClosed;
  }
}