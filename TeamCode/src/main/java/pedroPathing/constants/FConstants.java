package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftBack";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightBack";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 14;

        FollowerConstants.xMovement = 85;
        FollowerConstants.yMovement = 65;

        FollowerConstants.forwardZeroPowerAcceleration = -27;
//        FollowerConstants.lateralZeroPowerAcceleration = -70;
        FollowerConstants.lateralZeroPowerAcceleration = -27;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.3,0,0.02,0);
//        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.14,0,0.0,0);
//        FollowerConstants.useSecondaryTranslationalPID = true;
//        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.25,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID
//        FollowerConstants.translationalPIDFSwitch = 4;

        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.12,0);
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(1.4,0.005,0.09,0); // Not being used, @see useSecondaryHeadingPID
        FollowerConstants.headingPIDFSwitch = 0.120;

//        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.02,0,0.000,0.45,0);
        FollowerConstants.useSecondaryDrivePID = true; // d was 0.0004
        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.025,0,0.00001,0.6,0);
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.005,0,0.000,0.45,0); // Not being used, @see useSecondaryDrivePID
//        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.025,0,0.000017,0.6,0);
        FollowerConstants.drivePIDFSwitch = 4;

        FollowerConstants.zeroPowerAccelerationMultiplier = 4.5;
        FollowerConstants.centripetalScaling = 0.00031;

        FollowerConstants.useVoltageCompensationInAuto = true;
        FollowerConstants.useBrakeModeInTeleOp = true;
        FollowerConstants.turnHeadingErrorThreshold = 0.15;
//
//        FollowerConstants.pathEndTimeoutConstraint = 500;
    }
}
