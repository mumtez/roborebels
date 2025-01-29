package pedroPathing.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {

  static {
    FollowerConstants.localizers = Localizers.PINPOINT;

    FollowerConstants.leftFrontMotorName = "fl";
    FollowerConstants.leftRearMotorName = "bl";
    FollowerConstants.rightFrontMotorName = "fr";
    FollowerConstants.rightRearMotorName = "br";

    FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
    FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
    FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

    FollowerConstants.mass = 14.2;

    FollowerConstants.xMovement = 75.2449;
    FollowerConstants.yMovement = 109.03;

    FollowerConstants.forwardZeroPowerAcceleration = -29.0683;
    FollowerConstants.lateralZeroPowerAcceleration = -116.4933;

    FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.028, 0, 0.0005, 0.0001);
    FollowerConstants.useSecondaryTranslationalPID = false;
    FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1, 0, 0.01, 0);

    FollowerConstants.headingPIDFCoefficients.setCoefficients(2, 0, 0.15, 0);
    FollowerConstants.useSecondaryHeadingPID = false;
    FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2, 0, 0.1, 0);

    FollowerConstants.drivePIDFCoefficients.setCoefficients(0.008, 0, 0.0001, 0.6, 0);
    FollowerConstants.useSecondaryDrivePID = false;
    FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1, 0, 0, 0.6, 0);

    FollowerConstants.zeroPowerAccelerationMultiplier = 4;
    FollowerConstants.centripetalScaling = 0.0003;

    FollowerConstants.pathEndTimeoutConstraint = 500;
    FollowerConstants.pathEndTValueConstraint = 0.995;
    FollowerConstants.pathEndVelocityConstraint = 0.1;
    FollowerConstants.pathEndTranslationalConstraint = 0.1;
    FollowerConstants.pathEndHeadingConstraint = 0.007;

    FollowerConstants.automaticHoldEnd = true;
    FollowerConstants.useBrakeModeInTeleOp = true;
  }
}
