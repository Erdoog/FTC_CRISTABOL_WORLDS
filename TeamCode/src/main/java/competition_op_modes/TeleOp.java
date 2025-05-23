package competition_op_modes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

import controllers.ColorController;
import controllers.DawgTimer;
import controllers.DepositController;
import controllers.ExtendoController;
import controllers.HeadingController;
import controllers.PitcherController;
import controllers.TouchSensorController;
import util.InputScaler;
import controllers.IntakeController;
import controllers.LiftController;
import controllers.StateMachineController;
import controllers.VoltageProcessor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "!competition")
public class TeleOp extends OpMode {
    LiftController liftController;
    ExtendoController extendoController;
    HeadingController headingController;
    StateMachineController stateMachineController;
    TouchSensorController touchSensorController;
    PitcherController pitcherController;

    IntakeController intakeController;
    DepositController depositController;

    GamepadEx driver1, driver2;
    DawgTimer elapsedTimer;
    DawgTimer runtimeTimer;

    HDrive train;
    MotorEx leftFront, leftBack, rightBack, rightFront;

    List<LynxModule> allHubs;
    boolean tankDriveMod = false;
    boolean arcadeMod = false;
    boolean headingPIDOn = true;
    double totalCurrentDraw = 0;
    double prevCurrentDraw = 0;

    public static double TURN_SLOW = 0.5;

    @Override
    public void init(){
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DawgTimer.updateAllTimers();

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        VoltageProcessor.initializeSensor(hardwareMap);
//        VoltageProcessor.updateReadings();

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        touchSensorController = new TouchSensorController(hardwareMap);
        extendoController = new ExtendoController(hardwareMap);
        liftController = new LiftController(hardwareMap);
        headingController = new HeadingController(hardwareMap);
        stateMachineController = new StateMachineController(hardwareMap, extendoController, liftController, null);
        depositController = stateMachineController.depositController;
        intakeController = stateMachineController.intakeController;
        pitcherController = new PitcherController(hardwareMap);
        pitcherController.setActivated(false);

        elapsedTimer = new DawgTimer();
        runtimeTimer = new DawgTimer();

        leftBack =   new MotorEx(hardwareMap, "leftBack");
        leftFront =  new MotorEx(hardwareMap, "leftFront");
        rightBack =  new MotorEx(hardwareMap, "rightBack");
        rightFront = new MotorEx(hardwareMap, "rightFront");

        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftFront.setRunMode(Motor.RunMode.VelocityControl);
        rightFront.setRunMode(Motor.RunMode.VelocityControl);
        leftBack.setRunMode(Motor.RunMode.VelocityControl);
        rightBack.setRunMode(Motor.RunMode.VelocityControl);

        train = new HDrive(leftFront, rightFront, leftBack, rightBack);

    }

    @Override
    public void start(){
        headingController.resetPinpoint();
        extendoController.reset();
        liftController.reset();
    }

    @Override
    public void loop(){
        for (LynxModule hub : allHubs)
            hub.clearBulkCache();

        DawgTimer.updateAllTimers();
//        VoltageProcessor.updateReadings();


        double deltaTime = elapsedTimer.milliseconds() / 1000.0;
        elapsedTimer.reset();

        double forwardInput = -driver1.getLeftY();
        double leftInput = -driver1.getLeftX();
        double turnInput = driver1.getRightX();

        double forwardOutput = 0;
        double leftOutput = 0;
        double turnDeviateOutput = 0;

//        if (driver1.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
//            forwardOutput = InputScaler.scaleInputLow(forwardInput);
//            leftOutput = InputScaler.scaleInputLow(leftInput);
//            turnDeviateOutput = InputScaler.scaleInputLow(turnInput);
//        } else
        if (driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0) {
            forwardOutput = InputScaler.scaleInputLow(forwardInput);
            leftOutput = InputScaler.scaleInputLow(leftInput);
            turnDeviateOutput = InputScaler.scaleInputLow(turnInput);
        } else if (driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0) {
//        } else if (driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0 || ExtendoController.getTargetPosition() > 250) {
            forwardOutput = InputScaler.scaleInputMedium(forwardInput);
            leftOutput = InputScaler.scaleInputMedium(leftInput);
            turnDeviateOutput = InputScaler.scaleInputMedium(turnInput);
        } else {
            forwardOutput = InputScaler.scaleInputHigh(forwardInput);
            leftOutput = InputScaler.scaleInputHigh(leftInput);
            turnDeviateOutput = InputScaler.scaleInputHigh(turnInput);
        }

        double secondDriverTurn = 0;

//        if (!tankDriveMod){
//            if (headingController.getCappedCurrentHeading() > 225 && headingController.getCappedCurrentHeading() < 315)
//                turnDeviateOutput += InputScaler.scaleInputLow(driver2.getRightY());
//            else if (headingController.getCappedCurrentHeading() < 135 && headingController.getCappedCurrentHeading() > 45)
//                turnDeviateOutput -= InputScaler.scaleInputLow(driver2.getRightY());
//            else if (headingController.getCappedCurrentHeading() > 315 || headingController.getCappedCurrentHeading() < 45)
//                turnDeviateOutput += InputScaler.scaleInputLow(driver2.getRightX());
//            else if (headingController.getCappedCurrentHeading() < 225 && headingController.getCappedCurrentHeading() > 135)
//                turnDeviateOutput -= InputScaler.scaleInputLow(driver2.getRightX());
//        } else {
            secondDriverTurn = InputScaler.scaleInputLow(driver2.getLeftX());
//        }

        if (depositController.getCurrentState() == DepositController.DepositArmStates.CHAMBER && touchSensorController.getBothPressed()){
            headingController.reset();
        }

//        if (driver1.getButton(GamepadKeys.Button.START)){
        if (driver1.getButton(GamepadKeys.Button.START) || driver1.gamepad.touchpad){
            headingController.reset();
        }

        if (!tankDriveMod){
            if ((depositController.getCurrentState() == DepositController.DepositArmStates.SPECIMEN_GRAB_AIM
             || depositController.getCurrentState() == DepositController.DepositArmStates.CHAMBER) && headingPIDOn){
                headingController.deviateTargetHeading(turnDeviateOutput, deltaTime);
                double turnPower = -headingController.calculateTurnPower();
                train.driveFieldCentric(leftOutput, forwardOutput, turnPower, headingController.getHeading());
            }
            else
                train.driveFieldCentric(leftOutput, forwardOutput, -turnDeviateOutput * 0.5 - secondDriverTurn, headingController.getHeading());
        }
//            train.driveFieldCentric(leftOutput, forwardOutput, turnPower, headingController.getHeading());
        else {
            if (arcadeMod){
                train.driveRobotCentric(leftOutput, forwardOutput, -turnDeviateOutput * 0.5 - secondDriverTurn);
            } else {
                double leftStickForward = -driver1.getLeftY();
                double rightStickForward = -driver1.getRightY();
//                double leftStickStrafe = driver1.getLeftX();
                double leftStickStrafe = 0;
                double rightStickStrafe = driver1.getRightX() * 1.4;
//                double rightStickStrafe = -driver1.getRightX();
                double strafePower = leftStickStrafe / 2.0 + rightStickStrafe / 2.0;

                if (driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0) {
                    leftStickForward = InputScaler.scaleInputLow(leftStickForward);
                    rightStickForward = InputScaler.scaleInputLow(rightStickForward);
                    strafePower = InputScaler.scaleInputLow(strafePower);
                } else if (driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0) {
                    leftStickForward =  InputScaler.scaleInputMedium(leftStickForward);
                    rightStickForward = InputScaler.scaleInputMedium(rightStickForward);
                    strafePower =       InputScaler.scaleInputMedium(strafePower);
                } else {
                    leftStickForward = InputScaler.scaleInputHigh(leftStickForward);
                    rightStickForward = InputScaler.scaleInputHigh(rightStickForward);
                    strafePower = InputScaler.scaleInputHigh(strafePower);
                }

                double stickDifference = Math.abs(leftStickForward + rightStickForward);

                leftStickForward *= (1.0 - (stickDifference / 2.0) * TURN_SLOW);
                rightStickForward *= (1.0 - (stickDifference / 2.0) * TURN_SLOW);

                double leftFrontPower =   leftStickForward - strafePower - secondDriverTurn;
                double leftBackPower =    leftStickForward + strafePower - secondDriverTurn;
                double rightFrontPower = rightStickForward - strafePower - secondDriverTurn;
                double rightBackPower =  rightStickForward + strafePower - secondDriverTurn;

                double maxPower = 1;
                maxPower = Math.max(maxPower, leftFrontPower);
                maxPower = Math.max(maxPower, leftBackPower);
                maxPower = Math.max(maxPower, rightFrontPower);
                maxPower = Math.max(maxPower, rightBackPower);

                leftFrontPower /= maxPower;
                leftBackPower /= maxPower;
                rightFrontPower /= maxPower;
                rightBackPower /= maxPower;

                leftFront.set(leftFrontPower);
                leftBack.set( leftBackPower);
                rightFront.set(rightFrontPower);
                rightBack.set( rightBackPower);
            }
        }

        telemetry.addData("pitcher activated", pitcherController.getActivated());

        if (driver1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
            pitcherController.switchActivated();
            if (pitcherController.getActivated()){
                LiftController.bottomLimit = LiftController.HANG_LIMIT;
                stateMachineController.setHang(true);
            }
            else{
                LiftController.bottomLimit = -5;
                stateMachineController.setHang(false);
            }
        }

        if (driver2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)){
            HeadingController.setTargetHeading(headingController.getCappedCurrentHeading());
            headingPIDOn = !headingPIDOn;
        }

        if (driver1.wasJustPressed(GamepadKeys.Button.B)){
            double targetHeading = headingController.getTargetHeading();
            double roundedTarget = Math.floor(targetHeading / 45.0) * 45.0;
            if (targetHeading == roundedTarget){
                targetHeading -= 45.0;
            } else {
                targetHeading = roundedTarget;
            }
            HeadingController.setTargetHeading(targetHeading);
        }

        if (driver1.wasJustPressed(GamepadKeys.Button.X)){
            double targetHeading = headingController.getTargetHeading();
            double roundedTarget = Math.ceil(targetHeading / 45.0) * 45.0;
            if (targetHeading == roundedTarget){
                targetHeading += 45.0;
            } else {
                targetHeading = roundedTarget;
            }
            HeadingController.setTargetHeading(targetHeading);
        }

        double liftInput = 0;
//        if (driver2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
//            liftController.setTargetPosition(LiftController.topLimit);
//        }
//        if (driver2.getButton(GamepadKeys.Button.RIGHT_BUMPER))
//            liftInput += 1.0;
//        if (driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0)
//            liftInput -= 1.0;
        double liftLevelDeviation = 0;
        if (driver2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER))
            liftLevelDeviation = 1;
        if (driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0)
            liftLevelDeviation = -1;
        if (liftLevelDeviation != 0 && depositController.getCurrentState() == DepositController.DepositArmStates.HUMAN_DROP || depositController.getCurrentState() == DepositController.DepositArmStates.BASKET || depositController.getCurrentState() == DepositController.DepositArmStates.TRANSFER_READY)
            LiftController.deviateBasketLevel(liftLevelDeviation);

//        else if (depositController.getCurrentState() == DepositController.DepositArmStates.CHAMBER){
//            LiftController.deviateChamberLevel(liftLevelDeviation);
//        }

//        if (driver2.getButton(GamepadKeys.Button.LEFT_BUMPER)){
        if (gamepad2.left_bumper){
            liftInput += 1.0;
        }
        if (driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0)
            liftInput -= 1.0;
//            liftController.forceMove(-1);

        double extendoInput = 0;
//        if (!tankDriveMod){
//            if (headingController.getCappedCurrentHeading() < 315 && headingController.getCappedCurrentHeading() > 225)
//                extendoInput = driver2.getRightX() * 0.3;
//            else if (headingController.getCappedCurrentHeading() < 135 && headingController.getCappedCurrentHeading() > 45)
//                extendoInput = -driver2.getRightX() * 0.3;
//            else if (headingController.getCappedCurrentHeading() < 225 && headingController.getCappedCurrentHeading() > 135)
//                extendoInput = -driver2.getRightY() * 0.3;
//            else if (headingController.getCappedCurrentHeading() < 45 || headingController.getCappedCurrentHeading() > 315)
//                extendoInput = driver2.getRightY() * 0.3;
//        } else {
            extendoInput = -driver2.getRightY() * 0.3;
//        }

        double extendoLevelDeviation = 0;
//        if (driver1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER))
//            extendoLevelDeviation = 1;
//        if (driver1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER))
//            extendoLevelDeviation = -1;
        ExtendoController.deviateLevels(extendoLevelDeviation);

//        if (driver1.wasJustPressed(GamepadKeys.Button.Y)){
        if ((driver1.wasJustPressed(GamepadKeys.Button.A) || driver1.wasJustPressed(GamepadKeys.Button.Y) || driver2.wasJustPressed(GamepadKeys.Button.X))){
            if (ExtendoController.getFinalTargetPosition() > 300)
                ExtendoController.setFinalTargetPosition(ExtendoController.bottomLimit, true);
//                stateMachineController.transfer();
            else
                ExtendoController.setFinalTargetPosition(ExtendoController.topLimit, true);
        }

//        if (runtimeTimer.milliseconds() > 90000){
//            if (extendoController.getCurrentPosition() > 300)
//                extendoController.setTargetPosition(300);
//        }

//        if (driver1.getButton(GamepadKeys.Button.BACK))
        if (driver1.getButton(GamepadKeys.Button.DPAD_DOWN))
            extendoController.forceMove(-0.4);
        if (driver2.getButton(GamepadKeys.Button.BACK))
            liftController.forceMove(-0.3);

        if (liftInput > 0){
            if (LiftController.getFinalTargetPosition() != LiftController.topLimit)
                liftController.deviateTargetPosition(liftInput, deltaTime);
            else{
                depositController.deviateArm(liftInput, deltaTime);
            }
        } else if (liftInput < 0) {
            if (depositController.getArmHeightT() > 0){
                depositController.deviateArm(liftInput, deltaTime);
            }
            else
                liftController.deviateTargetPosition(liftInput, deltaTime);
        }
        extendoController.deviateTargetPosition(extendoInput, deltaTime);


        if (driver2.wasJustPressed(GamepadKeys.Button.Y)){
            if (stateMachineController.getActionState() == StateMachineController.ActionStates.TRANSFER_ENGAGE || stateMachineController.getActionState() == StateMachineController.ActionStates.TRANSFER_BOTH_GRABBING || stateMachineController.getActionState() == StateMachineController.ActionStates.TRANSFER_DEPOSIT_GRABBING){
                stateMachineController.cancelTransfer();
            } else
                stateMachineController.transfer();
        }
        if (driver2.wasJustPressed(GamepadKeys.Button.A)){
            stateMachineController.grab();
        }
        if (driver2.getButton(GamepadKeys.Button.DPAD_DOWN)){
            liftController.forceMove(-1);
        }
        if (driver2.wasJustPressed(GamepadKeys.Button.B)){
            stateMachineController.release(true);
        }

        if (driver2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
            depositController.switchModes();
        }

        if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
            intakeController.deviateTwist(true);
        if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
            intakeController.deviateTwist(false);
        if (driver2.wasJustPressed(GamepadKeys.Button.DPAD_UP))
            intakeController.straightenTwist();

//        totalCurrentDraw = 0;
//        totalCurrentDraw += extendoController.getCurrentDraw();
//        totalCurrentDraw += liftController.getTotalCurrentDraw();
//        totalCurrentDraw += leftFront.motorEx.getCurrent(CurrentUnit.MILLIAMPS) / 1000.0;
//        totalCurrentDraw += leftBack.motorEx.getCurrent(CurrentUnit.MILLIAMPS) / 1000.0;
//        totalCurrentDraw += rightFront.motorEx.getCurrent(CurrentUnit.MILLIAMPS) / 1000.0;
//        totalCurrentDraw += rightBack.motorEx.getCurrent(CurrentUnit.MILLIAMPS) / 1000.0;
//
//        telemetry.addData("Total current draw", totalCurrentDraw);
//        telemetry.addData("Rate of change of total current draw", (totalCurrentDraw - prevCurrentDraw) / deltaTime);
//
//        prevCurrentDraw = totalCurrentDraw;

//        extendoController.debug(telemetry);
//        liftController.debug(telemetry);
//        headingController.debug(telemetry);
//
//        telemetry.addData("sample rate", 1.0 / deltaTime);
//        VoltageProcessor.debug(telemetry);
//        headingController.debug(telemetry);
//        telemetry.update();

        if (!tankDriveMod)
            headingController.readSensorValues();
        stateMachineController.update();
        extendoController.update();
        liftController.update();

        driver1.readButtons();
        driver2.readButtons();
    }

    @Override
    public void stop(){
        double currentTimeStamp = System.nanoTime() / 1e9;
        intakeController.pwmDisable();
        depositController.pwmDisable();
        pitcherController.pwmDisable();
        while (System.nanoTime() / 1e9 - currentTimeStamp < 0.2) {}
    }
}
