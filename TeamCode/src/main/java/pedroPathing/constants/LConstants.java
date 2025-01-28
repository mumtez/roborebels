package pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.ThreeWheelConstants;

// TODO: swap to pinpoint localizer
public class LConstants {

  static {
    ThreeWheelConstants.forwardTicksToInches = 0.00297;
    ThreeWheelConstants.strafeTicksToInches = 0.006599;
    ThreeWheelConstants.turnTicksToInches = -0.00287;
    ThreeWheelConstants.leftY = 8.08;
    ThreeWheelConstants.rightY = -7.08;
    ThreeWheelConstants.strafeX = 0;
    ThreeWheelConstants.leftEncoder_HardwareMapName = "OdomPod";
    ThreeWheelConstants.rightEncoder_HardwareMapName = "bl";
    ThreeWheelConstants.strafeEncoder_HardwareMapName = "br";
    ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
    ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
    ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
  }
}




