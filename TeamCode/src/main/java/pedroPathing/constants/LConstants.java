package pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.GoBildaPinpointDriver.EncoderDirection;
import com.pedropathing.localization.constants.PinpointConstants;
import com.pedropathing.localization.constants.ThreeWheelConstants;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {

  static {
    /*
    PinpointConstants.forwardY  = 3.5;
    PinpointConstants.strafeX = 0;

    PinpointConstants.distanceUnit = DistanceUnit.INCH;
    PinpointConstants.hardwareMapName = "pinpoint";  // change name


    PinpointConstants.useYawScalar = false;
    PinpointConstants.yawScalar = 1.0;

    PinpointConstants.useCustomEncoderResolution = false;
    PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    PinpointConstants.customEncoderResolution = 13.26291192; // Useless ?

    PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

     */

    //TODO: Tune these
    /*
        a) Forward Localizer Tuner
        Position a ruler alongside your robot.

        Push the robot forward by the desired distance (default is 48 inches).

        The tuner will display two numbers:

        First number: Distance the robot thinks it has traveled.

        Second number (multiplier)

        (Optional) Run multiple tests and average the multipliers for better accuracy.

        Input this value in LConstants as ThreeWheelConstants.forwardTicksToInches = [multiplier], where [multiplier] is the value you obtained from the tuner.

        b) Lateral Localizer Tuner
        Position a ruler alongside your robot.

        Push the robot sideways (strafing) by the desired distance (default is 48 inches).

        The tuner will display two numbers:

        First number: Distance the robot thinks it has traveled laterally.

        Second number (multiplier)

        (Optional) Run multiple tests and average the multipliers for better accuracy.

        Input this value in LConstants as ThreeWheelConstants.strafeTicksToInches = [multiplier], where [multiplier] is the value you obtained from the tuner.

        c) Turn Localizer Tuner
        Position your robot facing a recognizable landmark, like a field tile edge.

        Spin the robot counterclockwise for one full rotation (or your desired angle).

        The tuner will display two numbers:

        First number: Distance the robot thinks it has spun.

        Second number (multiplier)

        (Optional) Run multiple tests and average the multipliers for better accuracy.

        Input this value in LConstants as ThreeWheelConstants.turnTicksToInches = [multiplier], where [multiplier] is the value you obtained from the tuner.

     */
    ThreeWheelConstants.forwardTicksToInches = .001989436789;
    ThreeWheelConstants.strafeTicksToInches = .001989436789;
    ThreeWheelConstants.turnTicksToInches = .001989436789;

    //TODO: Measure the pods (Robot Grid: https://pedropathing.com/localization/setup.html#robot-coordinate-grid)
    ThreeWheelConstants.leftY = 1;
    ThreeWheelConstants.rightY = -1;
    ThreeWheelConstants.strafeX = -2.5;

    //TODO: Check the names
    ThreeWheelConstants.leftEncoder_HardwareMapName = "leftFront";
    ThreeWheelConstants.rightEncoder_HardwareMapName = "rightRear";
    ThreeWheelConstants.strafeEncoder_HardwareMapName = "rightFront";

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









