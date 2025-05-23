package controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import opmodes.ServoAndControllersCalibrate;

@Config
public class DepositController {
    public static double CLOSED_POSITION = 0.65;
    public static double OPENED_POSITION = 0.33;
    public static double TRANSFER_READY_OPEN_POSITION = 0.38;

    public static double ARM_INPUT_SPEED = 1.8;

    public enum DepositArmStates {
        START               (0.75, 0.58),
        SPECIMEN_GRAB_AIM   (0.1425, 0.91),
        TRANSFER_READY      (0.755, 0.48),
        TRANSFER            (0.755, 0.48),
        CHAMBER             (0.83, 0.55),
        BASKET              (0.26, 0.77),
        BASKET_PREDROP      (0.42, 0.79),
        UPPER_BASKET        (0.36, 0.79),
        HUMAN_DROP          (0.1 , 0.73),
        HANG                (0.4 , 0.93),
        AUTO_PARK(0.83, 0.54),
        ;
        public final double armPosition, wristPosition;
        DepositArmStates(double armPosition, double wristPosition){
            this.armPosition = armPosition;
            this.wristPosition = wristPosition;
        }
    }

    DawgServo clawServo;
    DawgServo wristServo;
    DawgServo arm1Servo;
    DawgServo arm2Servo;
    DawgTimer actionTimer = new DawgTimer();

    public DepositController(HardwareMap hardwareMap) {
        clawServo =  new DawgServo(hardwareMap, "depositClaw");
        wristServo = new DawgServo(hardwareMap, "depositWrist");
        arm1Servo =  new DawgServo(hardwareMap, "depositArm1");
        arm2Servo =  new DawgServo(hardwareMap, "depositArm2");
        arm2Servo.setDirection(Servo.Direction.REVERSE);

        arm1Servo.setSpeedRangePerSecond(1.4);
        arm2Servo.setSpeedRangePerSecond(1.4);
        wristServo.setSpeedRangePerSecond(1.4);
        clawServo.setSpeedRangePerSecond(1.4);
    }

    public boolean isBusy(){
        boolean busy = false;
        busy |= arm1Servo.getBusy();
        busy |= arm2Servo.getBusy();
        busy |= wristServo.getBusy();
        busy |= clawServo.getBusy();
        return busy;
    }

    private boolean clawClosed = false;
    private boolean prevClawClosed = false;

    public boolean getClawClosed(){
        return clawClosed;
    }

    public void setClawClosed(boolean state){
        this.clawClosed = state;
        if (prevClawClosed != clawClosed)
            actionTimer.reset();
        prevClawClosed = clawClosed;
    }

    private DepositArmStates currentState = DepositArmStates.TRANSFER_READY;

    public DepositArmStates getCurrentState(){
        return currentState;
    }
    public void setState(DepositArmStates state){
        if (state == null)
            return;
        currentState = state;
        actionTimer.reset();
    }

    public void switchModes(){
        switchModes(false);
    }

    public void switchModes(boolean zeroOutHeading){
        setClawClosed(false);
//        HeadingController.resetTarget();
        if (zeroOutHeading){
            HeadingController.setTargetHeading(0);
        } else {
            HeadingController.setTargetHeading(HeadingController.getCappedCurrentHeading());
        }
        if (getCurrentState() == DepositArmStates.SPECIMEN_GRAB_AIM || getCurrentState() == DepositArmStates.CHAMBER)
            setState(DepositArmStates.TRANSFER_READY);
        else if (getCurrentState() == DepositArmStates.TRANSFER_READY || getCurrentState() == DepositArmStates.HUMAN_DROP || getCurrentState() == DepositArmStates.BASKET)
            setState(DepositArmStates.SPECIMEN_GRAB_AIM);
        LiftController.setFinalTargetPosition(0);
    }

    public void setSampleMode(){
        setClawClosed(false);
        if (getCurrentState() == DepositArmStates.SPECIMEN_GRAB_AIM)
            setState(DepositArmStates.TRANSFER_READY);
    }

    double armHeightT = 0;

    public void deviateArm(double input, double deltaTime){
        armHeightT += input * deltaTime * ARM_INPUT_SPEED;
        if (armHeightT < 0)
            armHeightT = 0;
        if (armHeightT > 1)
            armHeightT = 1;
    }

    public double getArmHeightT(){
        return armHeightT;
    }

    public void update(){
        if (LiftController.getFinalTargetPosition() <= 0 && getCurrentState() == DepositArmStates.BASKET)
            setState(DepositArmStates.HUMAN_DROP);
        if (LiftController.getFinalTargetPosition() > 0 && getCurrentState() == DepositArmStates.HUMAN_DROP)
            setState(DepositArmStates.BASKET);

        if (getCurrentState() == DepositArmStates.BASKET){
            double armPosition = currentState.armPosition + (DepositArmStates.UPPER_BASKET.armPosition - DepositArmStates.BASKET.armPosition) * armHeightT;
            double wristPosition = currentState.wristPosition + (DepositArmStates.UPPER_BASKET.wristPosition - DepositArmStates.BASKET.wristPosition) * armHeightT;
            arm1Servo.setPosition(armPosition);
            arm2Servo.setPosition(armPosition + ServoAndControllersCalibrate.DEPOSIT_ARM2_OFFSET);
            wristServo.setPosition(wristPosition);
        } else {
            arm1Servo.setPosition(currentState.armPosition);
            arm2Servo.setPosition(currentState.armPosition + ServoAndControllersCalibrate.DEPOSIT_ARM2_OFFSET);
            wristServo.setPosition(currentState.wristPosition);
            armHeightT = 0;
        }

        clawServo.setPosition(clawClosed ? CLOSED_POSITION : (currentState == DepositArmStates.TRANSFER_READY ? TRANSFER_READY_OPEN_POSITION : OPENED_POSITION));
    }

    public void debug(Telemetry telemetry){
        telemetry.addData("deposit state", getCurrentState());
    }
    public void pwmDisable(){
        clawServo.pwmDisable();
        arm1Servo.pwmDisable();
        arm1Servo.pwmDisable();
        wristServo.pwmDisable();
    }
}