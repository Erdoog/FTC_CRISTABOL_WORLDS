package controllers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import util.LerpController;

@Config
public class LiftController {
	public static boolean AUTO_RAN = false;

	private final MotorEx leftMotor, rightMotor, midMotor;
	private final MotorEx.Encoder encoder;

	private final DawgPIDFController controller;
	public static double kP = 0.005, kI = 0.000,
			kD = 0.0001,
			kF = 0.00009;
//	    	kD = 0.0002;

	private static double finalTargetPosition = 0;
	public static double inputSpeed = 600;
	public static double topLimit = 1530;
	public static double bottomLimit = 0;

	public static double HANG_LIMIT = -450;
	public static double AUTO_CHAMBER_HEIGHT = 173;
	public static double CHAMBER_HEIGHT = 792;
	public static double UPPER_BASKET_HEIGHT = 1530;
	public static double LOWER_BASKET_HEIGHT = 590;

	private static double currentPosition = 0;
	private static double currentVelocity = 0;
	double currentTarget = 0;
	double power = 0;
	private static LerpController lerpController = new LerpController();

	public LiftController(HardwareMap hardwareMap) {
		leftMotor = new MotorEx(hardwareMap, "leftLift");
		rightMotor = new MotorEx(hardwareMap, "rightLift");
		midMotor = new MotorEx(hardwareMap, "midLift");
		encoder = midMotor.encoder;

		leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
		rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
		midMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

		leftMotor.setRunMode(Motor.RunMode.RawPower);
		rightMotor.setRunMode(Motor.RunMode.RawPower);
		midMotor.setRunMode(Motor.RunMode.RawPower);

		rightMotor.setInverted(true);
		midMotor.setInverted(true);
		finalTargetPosition = 0;
//		if (reset){
		encoder.reset();
//		}

		controller = new DawgPIDFController(kP, kI, kD, kF);
		controller.setIntegrationBounds(500, 500);
		controller.setTolerance(50, 10);
	}

	public void reset() {
		encoder.reset();
		controller.reset();
	}

	public static void deviateBasketLevel(double deviation){
		if (deviation > 0){
			if (getFinalTargetPosition() >= LOWER_BASKET_HEIGHT)
				setFinalTargetPosition(UPPER_BASKET_HEIGHT);
			else
				setFinalTargetPosition(LOWER_BASKET_HEIGHT);
		} else if (deviation < 0){
			setFinalTargetPosition(bottomLimit);
		}
	}
//
//	public static void deviateChamberLevel(double deviation){
//		if (deviation > 0){
//			if (getTargetPosition() == CHAMBER_AIM_HEIGHT)
//				setTargetPosition(CHAMBER_SCORE_HEIGHT);
//		} else if (deviation < 0){
//			setTargetPosition(CHAMBER_AIM_HEIGHT);
//		}
//	}


	public static void setFinalTargetPosition(double position) {
		setFinalTargetPosition(position, false);
	}
	public static void setFinalTargetPosition(double position, boolean applyLerp) {
		if (position < bottomLimit)
			position = bottomLimit;
		if (position > topLimit)
			position = topLimit;

		if (Double.isNaN(position))
			return;
		if (applyLerp && position != finalTargetPosition){
			lerpController.beginMovement(currentPosition, position, Math.abs(currentPosition - position) * 0.75, true);
		} else
			lerpController.interrupt();
		finalTargetPosition = position;
	}
	public static double getFinalTargetPosition() {
		return finalTargetPosition;
	}

	public double getCurrentPosition(){
		return currentPosition;
	}

	public void deviateTargetPosition(double deviation, double deltaTime) {
		if (deviation == 0)
			return;
		finalTargetPosition += deviation * inputSpeed * deltaTime;
		if (finalTargetPosition < bottomLimit)
			finalTargetPosition = bottomLimit;
		if (finalTargetPosition > topLimit)
			finalTargetPosition = topLimit;
	}

	public boolean forced = false;
	public void forceMove(double power){
		finalTargetPosition = 0;
		encoder.reset();
		rightMotor.set(power);
		leftMotor.set(power);
		midMotor.set(power);
		forced = true;
	}

	public boolean isBusy(){
		return Math.abs(currentPosition - finalTargetPosition) > 60 || currentVelocity > 50;
	}

	public boolean isAlmostThere(){
		return Math.abs(currentPosition - finalTargetPosition) < 400;
	}

	public void update() {
		controller.setPIDF(kP, kI, kD, kF);

		currentPosition = encoder.getPosition();
		currentVelocity = encoder.getCorrectedVelocity();
		currentTarget = finalTargetPosition;
		double lerpTarget = lerpController.getPosition();
		if (lerpTarget == finalTargetPosition || finalTargetPosition != lerpController.getEndPos())
			lerpController.interrupt();
		if (lerpController.isBusy())
			currentTarget = lerpTarget;
		power = controller.calculate(currentPosition, currentTarget);

		if (Double.isInfinite(currentPosition))
			throw new RuntimeException("infinite current");
		if (Double.isInfinite(power))
			throw new RuntimeException("infinite power");
		if (Double.isInfinite(currentTarget))
			throw new RuntimeException("infinite target");
		if (!forced){
			leftMotor.set(power);
			rightMotor.set(power);
			midMotor.set(power);
		}
		forced = false;
	}

	public double getTotalCurrentDraw(){
		return leftMotor.motorEx.getCurrent(CurrentUnit.MILLIAMPS) / 1000.0
		     +  midMotor.motorEx.getCurrent(CurrentUnit.MILLIAMPS) / 1000.0
  		    + rightMotor.motorEx.getCurrent(CurrentUnit.MILLIAMPS) / 1000.0;
	}

	public void debug(Telemetry telemetry) {
		telemetry.addData("lift target", finalTargetPosition);
		telemetry.addData("lift current", currentPosition);
		telemetry.addData("lift power", power);
	}
}
