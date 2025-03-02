package pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.ThreeWheelConstants;

public class LConstants {

  static {

    ThreeWheelConstants.forwardTicksToInches = .001989436789;
    ThreeWheelConstants.strafeTicksToInches = .001989436789;
    ThreeWheelConstants.turnTicksToInches = .001989436789;

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
    ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
    ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
    ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
  }
}









