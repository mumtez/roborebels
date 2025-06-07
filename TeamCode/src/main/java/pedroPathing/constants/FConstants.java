package pedroPathing.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {

  static {
    FollowerConstants.localizers = Localizers.THREE_WHEEL;

    FollowerConstants.leftFrontMotorName = "fl";
    FollowerConstants.leftRearMotorName = "bl";
    FollowerConstants.rightFrontMotorName = "fr";
    FollowerConstants.rightRearMotorName = "br";

    FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
    FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
    FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

    FollowerConstants.mass = 13.6;

    FollowerConstants.xMovement = 80.60488553;
    FollowerConstants.yMovement = 62.66679621;

    FollowerConstants.forwardZeroPowerAcceleration = -33.59430605;
    FollowerConstants.lateralZeroPowerAcceleration = -79.55918221;

    FollowerConstants.translationalPIDFCoefficients.setCoefficients(.1, 0, .01, 0);
    FollowerConstants.useSecondaryTranslationalPID = false;

    FollowerConstants.headingPIDFCoefficients.setCoefficients(2, 0, 0.1, 0);
    FollowerConstants.useSecondaryHeadingPID = false;

    FollowerConstants.drivePIDFCoefficients.setCoefficients(0.015, 0, 0.0001, 0.6, 0);
    FollowerConstants.useSecondaryDrivePID = false;

    FollowerConstants.zeroPowerAccelerationMultiplier = 4;
    FollowerConstants.centripetalScaling = 0.0004;

    FollowerConstants.pathEndTimeoutConstraint = 500;
    FollowerConstants.pathEndTValueConstraint = 0.995;
    FollowerConstants.pathEndVelocityConstraint = 0.1;
    FollowerConstants.pathEndTranslationalConstraint = 0.1;
    FollowerConstants.pathEndHeadingConstraint = 0.007;

    FollowerConstants.turnHeadingErrorThreshold = 0.01; // default is 0.01
  }
}
