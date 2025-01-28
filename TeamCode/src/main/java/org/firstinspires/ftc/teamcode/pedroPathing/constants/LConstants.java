package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = 0.00297;
        TwoWheelConstants.strafeTicksToInches = 0.006599;
        //https://pedropathing.com/localization/twoWheel.html  steps 2 a & b

        TwoWheelConstants.forwardY  = 8.08;
        TwoWheelConstants.strafeX = 0;
        //Re measure the pods

        TwoWheelConstants.forwardEncoder_HardwareMapName  = "bl";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "br";
        //Check that the ports are correct

        TwoWheelConstants.forwardEncoderDirection  = Encoder.FORWARD;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;

        /*
            Run Localization Test
            If the x value ticks down when the robot moves forward, reverse the direction of the forward pod.
            If the y value ticks down when the robot moves left, reverse the direction of the strafe pod.
        */

        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
    }
}




