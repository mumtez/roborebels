package pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.ThreeWheelConstants;

public class LConstants {

  static {
    ThreeWheelConstants.forwardTicksToInches = 0.002010224152;
    ThreeWheelConstants.strafeTicksToInches = 0.002007984783;
    ThreeWheelConstants.turnTicksToInches = 0.001959233;
    ThreeWheelConstants.leftY = 7.5;
    ThreeWheelConstants.rightY = -7.5;
    ThreeWheelConstants.strafeX = 2.75;
    ThreeWheelConstants.leftEncoder_HardwareMapName = "fr";
    ThreeWheelConstants.rightEncoder_HardwareMapName = "bl";
    ThreeWheelConstants.strafeEncoder_HardwareMapName = "br";
    ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
    ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
    ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
  }
}




