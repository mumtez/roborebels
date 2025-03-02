package pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.ThreeWheelConstants;

public class LConstants {

  static {

    ThreeWheelConstants.forwardTicksToInches = 0.002010224152; // 0.0020124626327022157 0.002006875944816942 0.002011333879639193
    ThreeWheelConstants.strafeTicksToInches = 0.002007984783; // 0.002004745501020975 0.0020117399715245893 0.0020074688777171343
    ThreeWheelConstants.turnTicksToInches = 0.001959233;

    //TODO: Measure the pods (Robot Grid: https://pedropathing.com/localization/setup.html#robot-coordinate-grid)
    ThreeWheelConstants.leftY = 7.5;
    ThreeWheelConstants.rightY = -7.5;
    ThreeWheelConstants.strafeX = 2.75;

    //TODO: Check the names
    ThreeWheelConstants.leftEncoder_HardwareMapName = "fr";
    ThreeWheelConstants.rightEncoder_HardwareMapName = "bl";
    ThreeWheelConstants.strafeEncoder_HardwareMapName = "br";

    //TODO:Check these
    /*
    Run the Localization Test and observe the encoder values
    If the x value ticks down when the robot moves forward, reverse the direction of both of the parallel pods (left and right).
    If the x value stays relatively constant when the robot drives forward, that means that one of the parallel pods (left and right) need to be reversed.
    If the y value ticks down when the robot strafe left, reverse the direction of the strafe pod.
     */
    ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
    ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
    ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
  }
}









