package controllers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import util.LerpController;

@Config
public class ExtendoController {
	private final MotorEx motor;

	DawgTimer stallTimer = new DawgTimer();
	private final DawgPIDController controller;
	public static double kP = 0.015, kI = 0.000, kD = 0.0003;

	private static double finalTargetPosition = -10;
	private static double currentPosition = 0;
	public static double inputSpeed = 1100;
	public static double topLimit = 670;
	public static double bottomLimit = -10;
	public static double transferPosition = -50;
	private static double currentTarget = 0;
	private static double power = 0;
	static LerpController lerpController = new LerpController();

	public ExtendoController(HardwareMap hardwareMap) {
		motor = new MotorEx(hardwareMap, "extendo");
		motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
//		motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
		motor.setRunMode(Motor.RunMode.RawPower);
//		motor.setInverted(true);

		motor.resetEncoder();
		finalTargetPosition = bottomLimit;

		controller = new DawgPIDController(kP, kI, kD);
		controller.setIntegrationBounds(500, 500);
		controller.setTolerance(50, 10);
	}

	public static void deviateLevels(double deviation){
		if (deviation > 0){
			if (ExtendoController.getFinalTargetPosition() > 500){}
			else if (ExtendoController.getFinalTargetPosition() > 300) {
				ExtendoController.setFinalTargetPosition(670);
			}
			else if (ExtendoController.getFinalTargetPosition() > 100) {
				ExtendoController.setFinalTargetPosition(400);
			}
			else {
				ExtendoController.setFinalTargetPosition(200);
			}
		} else if (deviation < 0) {
			if (ExtendoController.getFinalTargetPosition() > 500){
				ExtendoController.setFinalTargetPosition(400);
			}
			else if (ExtendoController.getFinalTargetPosition() > 300) {
				ExtendoController.setFinalTargetPosition(200);
			}
			else {
				ExtendoController.setFinalTargetPosition(ExtendoController.bottomLimit);
			}
		}
	}

	public void reset() {
		motor.resetEncoder();
		controller.reset();
	}

	public static void setFinalTargetPosition(double position){
		setFinalTargetPosition(position, false);
	}

	public static void setFinalTargetPosition(double inputPosition, boolean useLerp) {
		if (inputPosition < bottomLimit && inputPosition != transferPosition)
			inputPosition = bottomLimit;
		if (inputPosition > topLimit)
			inputPosition = topLimit;

		if (Double.isNaN(inputPosition))
			return;
		if (inputPosition != transferPosition && useLerp){
			lerpController.beginMovement(currentPosition, inputPosition, 500 * Math.pow(Math.abs(inputPosition - currentPosition) / topLimit, 0.55), true);
		} else
			lerpController.interrupt();
		finalTargetPosition = inputPosition;
	}
	public static double getFinalTargetPosition(){
		return finalTargetPosition;
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

	public double getCurrentPosition(){
		return currentPosition;
	}

	public boolean forced = false;
	public void forceMove(double power){
		finalTargetPosition = 0;
		motor.resetEncoder();
		motor.set(power);
		forced = true;
	}

	public boolean isBusy(){
		return Math.abs(currentPosition - finalTargetPosition) > 105;
	}

	public boolean atTarget(double tolerance){
		return Math.abs(currentPosition - finalTargetPosition) < tolerance;
	}

	public boolean isMoving(){
		return Math.abs(currentVelocity) > 30;
	}

	double currentVelocity = 0;
	double draw = 0;
	public void update() {
		controller.setPID(kP, kI, kD);

		currentPosition = motor.getCurrentPosition();
		currentVelocity = motor.getCorrectedVelocity();
		currentTarget = finalTargetPosition;
		double lerpTarget = lerpController.getPosition();
		if (lerpTarget == finalTargetPosition || finalTargetPosition != lerpController.getEndPos() || finalTargetPosition == transferPosition)
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
//		if (Math.abs(currentVelocity) > 150 || currentTarget - currentPosition < 75)
//			stallTimer.reset();
//		if (stallTimer.milliseconds() > 300)
//			setFinalTargetPosition(bottomLimit);

		if (!forced)
			motor.set(power);
		forced = false;
	}

	public double getCurrentDraw(){
		return motor.motorEx.getCurrent(CurrentUnit.MILLIAMPS) / 1000.0;
	}

	public void debug(Telemetry telemetry) {
		telemetry.addData("extendo target", finalTargetPosition);
		telemetry.addData("extendo current target", currentTarget);
		telemetry.addData("extendo power", power);
		telemetry.addData("extendo current", currentPosition);
	}
}
