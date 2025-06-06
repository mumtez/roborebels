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

  public static double clawOpenWallPos = 0.45;

  public static double clawClosePos = 0.62;

  public static double upArmPlaceTeleOP = .5;


  public static double upArmWallAuto = 0.20;

  public static double upArmSpecInit = 0.07;

  public static double upArmBucket = 0.5;

  public static double upArmTransfer = 0.12;
  public static double upArmInit = 0.12;


  public static double downArmPlaceTeleOP = .98;


  public static double downArmWallAuto = 0.44;

  public static double downArmSpecInit = 0.7;
  public static double downArmBucket = 0.8;
  public static double downArmTransfer = 0;
  public static double downArmInit = 0;

  
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
    claw.setPosition(clawOpenWallPos);
  }


  public void setPlace() { // needs to be tuned
    clawDownArm.setPosition(downArmPlaceTeleOP);
    clawUpArm.setPosition(upArmPlaceTeleOP);
  }


  public void setWall() {
    clawDownArm.setPosition(downArmWallAuto);
    clawUpArm.setPosition(upArmWallAuto);
  }

  public void setInitSpec() {
    clawDownArm.setPosition(downArmSpecInit);
    clawUpArm.setPosition(upArmSpecInit);
  }


  public void setTransfer() {
    clawDownArm.setPosition(downArmTransfer);
    clawUpArm.setPosition(upArmTransfer);
  }


  public void setBucket() {
    clawDownArm.setPosition(downArmBucket);
    clawUpArm.setPosition(upArmBucket);
  }


  public void setInit() {
    clawDownArm.setPosition(downArmInit);
    clawUpArm.setPosition(upArmInit);
  }

  public boolean isClawClosed() {
    return clawClosed;
  }
}