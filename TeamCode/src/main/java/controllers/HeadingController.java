package controllers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV2;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

//import controllers.pinpoint.DawgGoBildaPinpointDriver;

@Config
public class HeadingController {
//	DawgGoBildaPinpointDriver pinpoint;
	GoBildaPinpointDriver pinpoint;

	private static double currentHeading = 0;
	private double currentRaw = 0;
	private static double targetHeading = 0;

	public static double incrementCoefficient = 180;

	public static double kP = 0.024, kI = 0.0, kD = 0.0003;
	private final PIDController controller;
	public double headingOffset;

	public double prevRawHeading = 0;

	public HeadingController(HardwareMap hardwareMap) {
		pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
		pinpoint.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1
		pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
		pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
		pinpoint.resetPosAndIMU();
//		pinpoint = hardwareMap.get(DawgGoBildaPinpointDriver.class, "pinpoint");
//		pinpoint.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1
//		pinpoint.setEncoderResolution(DawgGoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//		pinpoint.setEncoderDirections(DawgGoBildaPinpointDriver.EncoderDirection.REVERSED, DawgGoBildaPinpointDriver.EncoderDirection.FORWARD);
//		pinpoint.resetPosAndIMU();
		controller = new PIDController(kP, kI, kD);
	}

	DawgTimer resetTimer = new DawgTimer();

	public static void resetTarget(){
		targetHeading = currentHeading;
	}
	public static void setTargetHeading(double heading) {targetHeading = heading;}

	public void resetPinpoint(){
		currentHeading = 0;
		headingOffset = 0;
		targetHeading = 0;
		controller.reset();
		resetTimer.reset();
		pinpoint.resetPosAndIMU();
	}

	boolean wasNan = false;

	public void reset() {
		double rawHeading = pinpoint.getHeading();
		if (Double.isFinite(rawHeading))
			headingOffset = rawHeading / (Math.PI * 2) * 360.0;
		else
			wasNan = true;
		currentHeading = 0;
		targetHeading = 0;
		controller.reset();
	}

	public void readSensorValues() {
		pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
//		pinpoint.update(DawgGoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
		double rawHeading = (pinpoint.getHeading() / (Math.PI * 2)) * 360.0;
		currentRaw = rawHeading;

		double deltaHeading = currentRaw - prevRawHeading;

		prevRawHeading = currentRaw;

//		if (Double.isFinite(rawHeading))
//			currentHeading = rawHeading / (Math.PI * 2) * 360.0 - headingOffset;
//		else
//			wasNan = true;

//		if (Double.isFinite(deltaHeading) && deltaHeading < 15)
		if (Double.isFinite(deltaHeading))
			currentHeading += deltaHeading;

		while (currentHeading - targetHeading > 180)
			currentHeading -= 360;
		while (currentHeading - targetHeading < -180)
			currentHeading += 360;
	}

	public static double getCappedCurrentHeading(){
		double cappedCurrent = currentHeading;
		while (cappedCurrent < 0)
			cappedCurrent += 360;
		while (cappedCurrent > 360)
			cappedCurrent -= 360;

		return cappedCurrent;
	}

	public static double getTargetHeading() {
		return targetHeading;
	}


	public void deviateTargetHeading(double deviation, double deltaTime) {
		targetHeading -= deviation * incrementCoefficient * deltaTime;
	}

	public double calculateTurnPower() {
		controller.setPID(kP, kI, kD);
//		if (resetTimer.milliseconds() < 400 || isPinpointCooked())
//			return 0;
		return -controller.calculate(currentHeading, targetHeading);
	}

	public double getHeading() {
		return currentHeading;
	}

	public void debug(Telemetry telemetry) {
		telemetry.addData("target heading", targetHeading);
		telemetry.addData("current heading", currentHeading);
		telemetry.addData("current raw", currentRaw);
		telemetry.addData("prev raw", prevRawHeading);
		telemetry.addData("was Nan", wasNan);
		telemetry.addData("x", pinpoint.getPosX());
		telemetry.addData("y", pinpoint.getPosY());
	}
}
