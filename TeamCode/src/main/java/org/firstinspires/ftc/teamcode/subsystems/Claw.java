package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.acmerobotics.dashboard.FtcDashboard;

// CONFIG

@Config

public class Claw {

    public final ServoImplEx claw, clawUpArm, clawDownArm;


    public static float clawOpenPos;
    public static float clawClosePos;


    public static float upArmDefault;
    public static float upArmPlace;
    public static float upArmWall;
    public static float upArmBucket;


    public static float downArmDefault;
    public static float downArmPlace;
    public static float downArmWall;
    public static float downArmBucket;


    private final LinearOpMode opMode;

    public Claw(LinearOpMode opMode){

        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;

        claw = (ServoImplEx) hardwareMap.servo.get("c");
        clawUpArm = (ServoImplEx) hardwareMap.servo.get("cu");
        clawDownArm = (ServoImplEx) hardwareMap.servo.get("cd");


    }

    public void clawClose(){
        claw.setPosition(clawOpenPos);
    }

    public void clawOpen(){
        claw.setPosition(clawClosePos);
    }


    public void setDefault(){
        clawDownArm.setPosition(downArmDefault);
        waitTime(100);
        clawUpArm.setPosition(upArmDefault);
    }

    public void setPlace(){
        clawDownArm.setPosition(downArmPlace);
        clawUpArm.setPosition(upArmPlace);
    }

    public void setWall(){
        clawDownArm.setPosition(downArmWall);
        clawUpArm.setPosition(upArmWall);
    }

    public void setBucket(){
        clawDownArm.setPosition(downArmBucket);
        clawUpArm.setPosition(upArmBucket);
    }

    public void waitTime(double ms) {
        double startTime = System.currentTimeMillis();
        while (opMode.opModeIsActive() && System.currentTimeMillis() - startTime < ms) {
        }
    }




}
