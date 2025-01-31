package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

// CONFIG

@Config
public class Claw {

  public static double clawOpenPos = 0.57;
  public static double clawClosePos = 0.4;

  public static double upArmPlace = .47;
  public static double upArmWall = 0.34;
  public static double upArmBucket = 0.78;
  public static double upArmUnder = 0.60;
  public static double upArmTransfer = 0.00;
  public static double upArmInit = 0.60;

  public static double downArmPlace = 0.2;
  public static double downArmWall = 0.84;
  public static double downArmBucket = 0.35;
  public static double downArmUnder = 0.16;
  public static double downArmTransfer = 0.92;
  public static double downArmInit = 0.85;

  public final ServoImplEx claw, clawUpArm, clawDownArm;
  // TODO: add color/touch/limit sensor (states?)

  public Claw(LinearOpMode opMode) {
    HardwareMap hardwareMap = opMode.hardwareMap;

    claw = (ServoImplEx) hardwareMap.servo.get("c");
    clawUpArm = (ServoImplEx) hardwareMap.servo.get("cu");
    clawDownArm = (ServoImplEx) hardwareMap.servo.get("cd");
  }

  public void clawClose() {
    claw.setPosition(clawClosePos);
  }

  public void clawOpen() {
    claw.setPosition(clawOpenPos);
  }

  public void setPlace() {
    clawDownArm.setPosition(downArmPlace);
    clawUpArm.setPosition(upArmPlace);
  }

  public void setTransfer() {
    clawDownArm.setPosition(downArmTransfer);
    clawUpArm.setPosition(upArmTransfer);
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

}