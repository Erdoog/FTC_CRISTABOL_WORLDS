package controllers;

import static java.lang.Double.NaN;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StateMachineController {
    DawgTimer actionTimer = new DawgTimer();
    DawgTimer grabCooldownTimer = new DawgTimer();
    LiftController liftController;
    ExtendoController extendoController;
//    ColorController colorController;
    public DepositController depositController;
    public IntakeController intakeController;

    private ActionStates actionState = ActionStates.FREE;
    public ActionStates getActionState(){
        return actionState;
    }

    public double extendoBeforeTransfer = 0;
    public boolean successfulGrabEvent = false;


    public StateMachineController(HardwareMap hardwareMap, ExtendoController extendoController, LiftController liftController, ColorController.COLORS allianceColor){
        this.liftController = liftController;
        this.extendoController = extendoController;
        this.depositController = new DepositController(hardwareMap);
        this.intakeController = new IntakeController(hardwareMap);
//        this.colorController = new ColorController(hardwareMap);
    }

    public enum ActionStates {
        FREE(null, null, NaN, NaN),

        FLOOR_PICK          (DepositController.DepositArmStates.TRANSFER_READY, IntakeController.IntakeArmStates.FLOOR_PICK, NaN, NaN),
        FLOOR_CLAWING       (DepositController.DepositArmStates.TRANSFER_READY, IntakeController.IntakeArmStates.FLOOR_PICK, NaN, NaN),
        RELEASE_AFTER_FAIL  (DepositController.DepositArmStates.TRANSFER_READY, IntakeController.IntakeArmStates.FLOOR_AIM , NaN, NaN),

        TRANSFER_ENGAGE(DepositController.DepositArmStates.TRANSFER_READY, IntakeController.IntakeArmStates.TRANSFER_READY, LiftController.bottomLimit, ExtendoController.transferPosition),
        TRANSFER_AWAIT_SAMPLE(DepositController.DepositArmStates.TRANSFER_READY, IntakeController.IntakeArmStates.TRANSFER_READY, LiftController.bottomLimit, ExtendoController.transferPosition),
        TRANSFER_AWAIT_ARMS(DepositController.DepositArmStates.TRANSFER, IntakeController.IntakeArmStates.TRANSFER, LiftController.bottomLimit, ExtendoController.transferPosition),
        TRANSFER_BOTH_GRABBING(DepositController.DepositArmStates.TRANSFER, IntakeController.IntakeArmStates.TRANSFER, LiftController.bottomLimit, ExtendoController.transferPosition),
        TRANSFER_DEPOSIT_GRABBING(DepositController.DepositArmStates.TRANSFER, IntakeController.IntakeArmStates.TRANSFER, LiftController.bottomLimit, ExtendoController.transferPosition),
        CANCEL_TRANSFER(DepositController.DepositArmStates.TRANSFER_READY, IntakeController.IntakeArmStates.FLOOR_AIM, LiftController.bottomLimit, NaN),

        BASKET_RELEASING(null, null, NaN, NaN),
        HUMAN_RELEASING(null, null, NaN, NaN),

        CHAMBER_RELEASING(DepositController.DepositArmStates.CHAMBER, null, NaN, NaN),
        CHAMBER_ARM_RETRACTING(DepositController.DepositArmStates.SPECIMEN_GRAB_AIM, null, NaN, NaN),
        SPECIMEN_GRABBING(null, null, NaN, NaN)
        ;
        final IntakeController.IntakeArmStates intakeState;
        final DepositController.DepositArmStates depositArmStates;
        final double liftHeight;
        final double extendoPosition;
        ActionStates(DepositController.DepositArmStates depositArmStates, IntakeController.IntakeArmStates intakeState, double liftHeight, double extendoPosition) {
            this.depositArmStates = depositArmStates;
            this.intakeState = intakeState;
            this.liftHeight = liftHeight;
            this.extendoPosition = extendoPosition;
        }
    }

    public void setHang(boolean hang){
        if (hang){
            depositController.setState(DepositController.DepositArmStates.HANG);
            intakeController.setState(IntakeController.IntakeArmStates.HANG);
        } else {
            depositController.setState(DepositController.DepositArmStates.TRANSFER_READY);
            intakeController.setState(IntakeController.IntakeArmStates.FLOOR_AIM);
        }
    }
    public void switchHang(){
        if (depositController.getCurrentState() == DepositController.DepositArmStates.HANG){
            depositController.setState(DepositController.DepositArmStates.TRANSFER_READY);
            intakeController.setState(IntakeController.IntakeArmStates.FLOOR_AIM);
        } else {
            depositController.setState(DepositController.DepositArmStates.HANG);
            intakeController.setState(IntakeController.IntakeArmStates.HANG);
        }
    }

    public void setActionState(ActionStates state){
        actionState = state;
        actionTimer.reset();
    }


    public void grab(){
        if (getActionState() != ActionStates.FREE)
            return;
        if (depositController.getCurrentState() == DepositController.DepositArmStates.CHAMBER)
            depositController.switchModes();
        if (depositController.getCurrentState() == DepositController.DepositArmStates.HUMAN_DROP || depositController.getCurrentState() == DepositController.DepositArmStates.BASKET)
            release();
        if (depositController.getCurrentState() == DepositController.DepositArmStates.TRANSFER_READY){
//            if (grabCooldownTimer.milliseconds() < 700)
//                return;
            if (!intakeController.getClawClosed() && !intakeController.isBusy())
                setActionState(ActionStates.FLOOR_PICK);
            else
                setActionState(ActionStates.RELEASE_AFTER_FAIL);
        }


        else if (depositController.getCurrentState() == DepositController.DepositArmStates.SPECIMEN_GRAB_AIM)
            setActionState(StateMachineController.ActionStates.SPECIMEN_GRABBING);
    }

    public void release() {
        release(false);
    }

    boolean zeroHeadingOnDrop = false;

    public void release(boolean zeroHeading){
        zeroHeadingOnDrop = zeroHeading;
        if (getActionState() != ActionStates.FREE)
            return;
        grabCooldownTimer.unreset();
        if (!depositController.isBusy() && depositController.getCurrentState() == DepositController.DepositArmStates.HUMAN_DROP)
            setActionState(StateMachineController.ActionStates.HUMAN_RELEASING);
        if (!depositController.isBusy() && depositController.getCurrentState() == DepositController.DepositArmStates.BASKET)
            setActionState(StateMachineController.ActionStates.BASKET_RELEASING);
        if (!depositController.isBusy() && depositController.getCurrentState() == DepositController.DepositArmStates.CHAMBER)
            setActionState(StateMachineController.ActionStates.CHAMBER_RELEASING);
        intakeController.setClawClosed(false);
    }

    public void transfer(){
        if ((getActionState() == ActionStates.FREE) && depositController.getCurrentState() != DepositController.DepositArmStates.CHAMBER){
            setActionState(ActionStates.TRANSFER_ENGAGE);
            extendoBeforeTransfer = ExtendoController.getFinalTargetPosition();
        }
    }

    public void cancelTransfer(){
        if (getActionState() == ActionStates.TRANSFER_ENGAGE || getActionState() == ActionStates.TRANSFER_BOTH_GRABBING || getActionState() == ActionStates.TRANSFER_DEPOSIT_GRABBING)
            setActionState(ActionStates.CANCEL_TRANSFER);
    }

    public boolean isAnythingBusy(){
        boolean busy = false;
        busy |= intakeController.isBusy();
        busy |= depositController.isBusy();
        busy |= liftController.isBusy();
        busy |= extendoController.isBusy();
        return busy;
    }

    public void update(){
        IntakeController.IntakeArmStates targetIntakeState = actionState.intakeState;
        DepositController.DepositArmStates targetDepositState = actionState.depositArmStates;
        double targetLiftHeight = actionState.liftHeight;
        double targetExtendoPosition = actionState.extendoPosition;
        successfulGrabEvent = false;

        switch (actionState){
            case CANCEL_TRANSFER:
                targetExtendoPosition = extendoBeforeTransfer;
                setActionState(ActionStates.FREE);
                break;
            case BASKET_RELEASING:
                depositController.setClawClosed(false);
                if (actionTimer.milliseconds() > 250){
                    setActionState(ActionStates.FREE);
                    depositController.setState(DepositController.DepositArmStates.TRANSFER_READY);
                }
                break;
            case HUMAN_RELEASING:
                depositController.setClawClosed(false);
                if (actionTimer.milliseconds() > 250){
                    setActionState(ActionStates.FREE);
                    depositController.switchModes(zeroHeadingOnDrop);
                }
                break;
            case TRANSFER_ENGAGE:
                intakeController.setClawState(IntakeController.ClawStates.TRANSFER);
                intakeController.straightenTwist();
                depositController.setClawClosed(false);
                if (!isAnythingBusy() && actionTimer.milliseconds() > 50)
                    setActionState(ActionStates.TRANSFER_AWAIT_SAMPLE);
                break;
            case TRANSFER_AWAIT_SAMPLE:
                intakeController.setClawState(IntakeController.ClawStates.TRANSFER);
                intakeController.straightenTwist();
                depositController.setClawClosed(false);
//                if (actionTimer.milliseconds() > 80)
                    setActionState(ActionStates.TRANSFER_AWAIT_ARMS);
                break;
            case TRANSFER_AWAIT_ARMS:
                intakeController.setClawState(IntakeController.ClawStates.CLOSED);
                intakeController.straightenTwist();
                depositController.setClawClosed(false);
                if (actionTimer.milliseconds() > 80)
                    setActionState(ActionStates.TRANSFER_BOTH_GRABBING);
                break;
            case TRANSFER_BOTH_GRABBING:
                intakeController.setClawClosed(true);
                intakeController.straightenTwist();
                depositController.setClawClosed(true);
                if (actionTimer.milliseconds() > 160)
                    setActionState(ActionStates.TRANSFER_DEPOSIT_GRABBING);
                break;
            case TRANSFER_DEPOSIT_GRABBING:
                intakeController.setClawClosed(false);
                intakeController.straightenTwist();
                depositController.setClawClosed(true);
                if (actionTimer.milliseconds() > 80){
                    setActionState(ActionStates.FREE);
                    targetIntakeState = IntakeController.IntakeArmStates.FLOOR_AIM;
                    targetDepositState = DepositController.DepositArmStates.BASKET;
                }
                break;
            case SPECIMEN_GRABBING:
                depositController.setClawClosed(true);
                if (actionTimer.milliseconds() > 250){
                    setActionState(ActionStates.FREE);
                    targetDepositState = DepositController.DepositArmStates.CHAMBER;
//                    targetLiftHeight = LiftController.CHAMBER_HEIGHT;
                    targetLiftHeight = LiftController.CHAMBER_HEIGHT;
                }
                break;
            case CHAMBER_RELEASING:
                depositController.setClawClosed(false);
                if (!depositController.isBusy() && actionTimer.milliseconds() > 150){
                    setActionState(ActionStates.CHAMBER_ARM_RETRACTING);
                    targetLiftHeight = LiftController.bottomLimit;
                }
                break;
            case CHAMBER_ARM_RETRACTING:
                if (actionTimer.milliseconds() > 300){
                    setActionState(ActionStates.FREE);
                }
                break;
            case FLOOR_PICK:
                if (actionTimer.milliseconds() > 200){
                    setActionState(ActionStates.FLOOR_CLAWING);
                }
                break;
            case FLOOR_CLAWING:
                intakeController.setClawClosed(true);
                if (actionTimer.milliseconds() > 170){
                    targetIntakeState = IntakeController.IntakeArmStates.FLOOR_AIM;
                    intakeController.setState(IntakeController.IntakeArmStates.FLOOR_AIM);
                    setActionState(ActionStates.FREE);
                }
                break;
            case RELEASE_AFTER_FAIL:
                intakeController.setClawClosed(false);
                if (actionTimer.milliseconds() > 180){
                    setActionState(ActionStates.FLOOR_PICK);
                }
            case FREE:
                if (LiftController.getFinalTargetPosition() < LiftController.bottomLimit)
                    targetLiftHeight = LiftController.bottomLimit;
                if (ExtendoController.getFinalTargetPosition() < ExtendoController.bottomLimit)
                    targetExtendoPosition = ExtendoController.bottomLimit;
                if (ExtendoController.getFinalTargetPosition() > 0 && depositController.getCurrentState() == DepositController.DepositArmStates.SPECIMEN_GRAB_AIM)
                    depositController.setSampleMode();
                break;
        }

//        if (extendoController.isMoving() && (intakeController.getState() == IntakeController.IntakeArmStates.FLOOR_AIM || intakeController.getState() == IntakeController.IntakeArmStates.PICKED)){
//            targetIntakeState = IntakeController.IntakeArmStates.HIDDEN;
//        }
//        if (!extendoController.isMoving() && intakeController.getState() == IntakeController.IntakeArmStates.HIDDEN){
//            targetIntakeState = intakeController.prevState;
//        }

        if ((targetIntakeState == IntakeController.IntakeArmStates.FLOOR_AIM ||
           (intakeController.getState() == IntakeController.IntakeArmStates.FLOOR_AIM && targetIntakeState == null)) &&
            extendoController.isBusy())
            targetIntakeState = IntakeController.IntakeArmStates.FLOOR_HOVER;
        if (!extendoController.isBusy() && (targetIntakeState == IntakeController.IntakeArmStates.FLOOR_HOVER || intakeController.getState() == IntakeController.IntakeArmStates.FLOOR_HOVER))
            targetIntakeState = IntakeController.IntakeArmStates.FLOOR_AIM;

        if (targetDepositState != null)
            depositController.setState(targetDepositState);
        if (targetIntakeState != null)
            intakeController.setState(targetIntakeState);
        if (!Double.isNaN(targetLiftHeight))
            LiftController.setFinalTargetPosition(targetLiftHeight);
        if (!Double.isNaN(targetExtendoPosition))
            ExtendoController.setFinalTargetPosition(targetExtendoPosition);
        intakeController.update();
        depositController.update();
    }

    public void degub(Telemetry telemetry){
        telemetry.addData("state", actionState);
    }
}
