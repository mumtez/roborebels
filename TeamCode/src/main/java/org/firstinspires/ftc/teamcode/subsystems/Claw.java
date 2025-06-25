package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

// CONFIG

@Config
public class Claw {

  // might be updated
  public static double clawOpenPos = 0.87;

  public static double clawOpenWallPos = 0.6;

  public static double clawClosePos = 1;

  public static double downArmPlace = 0.84;

  public static double upArmPlace = 0.44;

  public static double downArmWall = 0.35;
  public static double upArmWall = 0.12;

  public static double upArmSpecInit = 0.14;

  public static double upArmBucket = 0.53;

  public static double upArmTransfer = 0.09;
  public static double upArmInit = 0.09;

  public static double downArmSpecInit = .5;
  public static double downArmBucket = 0.84;
  public static double downArmTransfer = 0.01;
  public static double downArmInit = 0.01;


  public final ServoImplEx claw, clawUpArm, clawDownArm;
  // TODO: add color/touch/limit sensor (states?)

  private boolean clawClosed = false;

  public Claw(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;

    claw = (ServoImplEx) hardwareMap.servo.get("c");
    clawUpArm = (ServoImplEx) hardwareMap.servo.get("cu");
    clawDownArm = (ServoImplEx) hardwareMap.servo.get("cd");

  }

  public void disableClaw () {
    claw.setPwmDisable();
    clawUpArm.setPwmDisable();
    clawDownArm.setPwmDisable();
  }

  public void enableClaw () {
    claw.setPwmEnable();
    clawUpArm.setPwmEnable();
    clawDownArm.setPwmEnable();
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


  public void setPlace() { // TODO: needs to be tuned
    clawDownArm.setPosition(downArmPlace);
    clawUpArm.setPosition(upArmPlace);
  }


  public void setWall() {
    clawDownArm.setPosition(downArmWall);
    clawUpArm.setPosition(upArmWall);
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