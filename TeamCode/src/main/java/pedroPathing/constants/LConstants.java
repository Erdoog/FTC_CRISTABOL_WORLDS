package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {
//        ThreeWheelConstants.forwardTicksToInches = .001989436789;
//        ThreeWheelConstants.strafeTicksToInches = .001989436789;
//        ThreeWheelConstants.turnTicksToInches = .001989436789;
//        ThreeWheelConstants.leftY = 1;
//        ThreeWheelConstants.rightY = -1;
//        ThreeWheelConstants.strafeX = -2.5;
//        ThreeWheelConstants.leftEncoder_HardwareMapName = "leftFront";
//        ThreeWheelConstants.rightEncoder_HardwareMapName = "rightRear";
//        ThreeWheelConstants.strafeEncoder_HardwareMapName = "rightFront";//        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
//        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
//        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;

        PinpointConstants.forwardY = -140;
        PinpointConstants.strafeX = -10;
        PinpointConstants.distanceUnit = DistanceUnit.MM;
        PinpointConstants.hardwareMapName = "pinpoint";
        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    }
}




