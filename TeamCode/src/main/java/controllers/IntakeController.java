package controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import opmodes.ServoAndControllersCalibrate;

@Config
public class IntakeController {
    public static double ARM_OFFSET = 0.0;

    public enum ClawStates{
        OPEN(0.32),
        CLOSED(0.64),
        TRANSFER(0.63)
        ;
        public final double position;
        ClawStates(double position){
            this.position = position;
        }
    }

    public enum TwisterStates {
        LEFT_FULL     (0.77),
        LEFT_DIAGONAL (0.6),
        STRAIGHT      (0.43),
        RIGHT_DIAGONAL(0.26),
        RIGHT_FULL    (0.09),
        ;
        public final double position;
        TwisterStates(double position){
            this.position = position;
        }
    }

    public enum IntakeArmStates {
        START         (0.81, 0.90),
        CAMERA        (0.37, 0.50),
        FLOOR_PICK    (0.37, 0.89),
    FLOOR_AIM     (0.450, 0.92),
        FLOOR_HOVER   (0.53, 0.92),
        //        TRANSFER      (.56 , 0.25)
        TRANSFER      (0.63, 0.55),
        TRANSFER_READY(0.55, 0.55),
        HANG          (0.72, 0.44)
        ;
        public final double armPosition, wristPosition;
        IntakeArmStates(double armPosition, double wristPosition){
            this.armPosition = armPosition;
            this.wristPosition = wristPosition;
        }
    }

    DawgServo clawServo;
    DawgServo twisterServo;
    DawgServo wristServo;
    DawgServo arm1Servo;
    DawgServo arm2Servo;

    DawgTimer actionTimer = new DawgTimer();
    public IntakeController(HardwareMap hardwareMap) {
        clawServo =    new DawgServo(hardwareMap, "intakeClaw");
        twisterServo = new DawgServo(hardwareMap, "intakeTwister");
        wristServo =   new DawgServo(hardwareMap, "intakeWrist");
        arm1Servo =    new DawgServo(hardwareMap, "intakeArm1");
        arm2Servo =    new DawgServo(hardwareMap, "intakeArm2");
        arm2Servo.setDirection(Servo.Direction.REVERSE);

        arm1Servo.setSpeedRangePerSecond(1.4);
        arm2Servo.setSpeedRangePerSecond(1.4);
        wristServo.setSpeedRangePerSecond(1.45);
        clawServo.setSpeedRangePerSecond(1.5);
        twisterServo.setSpeedRangePerSecond(1.5);
    }

    public boolean isBusy(){
        boolean busy = false;
        busy |= arm1Servo.getBusy();
        busy |= arm2Servo.getBusy();
        busy |= wristServo.getBusy();
        busy |= clawServo.getBusy();
        busy |= twisterServo.getBusy();
        return busy;
    }

    ClawStates clawState = ClawStates.OPEN;
    ClawStates prevClawState = ClawStates.OPEN;
//    private boolean clawClosed = false;
//    private boolean prevClawClosed = false;

    public  boolean getClawClosed(){
        return (clawState != ClawStates.OPEN);
    }

    public void setClawState(ClawStates clawState){
        if (this.clawState == clawState)
            return;
        actionTimer.reset();
        this.clawState = clawState;
    }

    public void setClawClosed(boolean state){
        clawState = (state ? ClawStates.CLOSED : ClawStates.OPEN);
        if (prevClawState == clawState)
            return;
        actionTimer.reset();
        prevClawState = clawState;
//        this.clawClosed = state;
//        if (prevClawClosed != clawClosed)
//            actionTimer.reset();
//        prevClawClosed = clawClosed;
    }

    TwisterStates currentTwisterState = TwisterStates.STRAIGHT;
    public void straightenTwist(){
        twisterCustomized = false;
        currentTwisterState = TwisterStates.STRAIGHT;
    }

    public void setTwist(TwisterStates twisterState){
        twisterCustomized = false;
        currentTwisterState = twisterState;
    }

    boolean twisterCustomized = false;
    double twisterCustomPosition;
    public void setTwistAngle(double degrees){
        while (degrees > 90)
            degrees -= 180;
        while (degrees < -90)
            degrees += 180;
        twisterCustomized = true;
        twisterCustomPosition = 0.5 + (0.34 * -degrees / 90.0);
    }

    public void deviateTwist(boolean right){
        twisterCustomized = false;
        int direction = right ? 1 : -1;
        int targetIndex = currentTwisterState.ordinal() + direction;
//        if (targetIndex < 0)
//            targetIndex = TwisterStates.values().length - 2;
//        if (TwisterStates.values().length <= targetIndex)
//            targetIndex = 1;
        if (targetIndex < 0 || TwisterStates.values().length <= targetIndex)
            return;
        currentTwisterState = TwisterStates.values()[targetIndex];
    }

    public TwisterStates getTwist(){
        return currentTwisterState;
    }

    boolean securityHide = false;
    private IntakeArmStates currentState = IntakeArmStates.FLOOR_AIM;
    private IntakeArmStates lastIteraionState = IntakeArmStates.FLOOR_AIM;
    public IntakeArmStates prevState = IntakeArmStates.FLOOR_AIM;

    public IntakeArmStates getState(){
        return currentState;
    }
    public void setState(IntakeArmStates state){
        if (state == null)
            return;
        if (state != lastIteraionState)
            prevState = lastIteraionState;
        lastIteraionState = currentState;
        currentState = state;
        actionTimer.reset();
    }

    public void update(){
        double armPosition = currentState.armPosition + ARM_OFFSET;
        arm1Servo.setPosition(armPosition, currentState == IntakeArmStates.FLOOR_PICK);
        arm2Servo.setPosition(armPosition + ServoAndControllersCalibrate.INTAKE_ARM2_OFFSET, currentState == IntakeArmStates.FLOOR_PICK);
        wristServo.setPosition(currentState.wristPosition);
        if (!twisterCustomized)
            twisterServo.setPosition(currentTwisterState.position);
        else
            twisterServo.setPosition(twisterCustomPosition);
        clawServo.setPosition(clawState.position);
    }

    public void pwmDisable(){
        clawServo.pwmDisable();
        twisterServo.pwmDisable();
        arm1Servo.pwmDisable();
        arm1Servo.pwmDisable();
        wristServo.pwmDisable();
    }
}
