package controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class VoltageProcessor {
    public static class ReadingsNotUpdatedException extends Exception {}
    public static class SensorNotInitializedException extends Exception {}
    public static double voltage = 0;
    static VoltageSensor sensor;
    public static void initializeSensor(HardwareMap hardwareMap){
        sensor = hardwareMap.voltageSensor.iterator().next();
    }

    public static void updateReadings() {
        if (sensor == null){
            try {
                throw new SensorNotInitializedException();
            } catch (SensorNotInitializedException e){
                throw new RuntimeException(e);
            }
        }
        voltage = sensor.getVoltage();
        if (voltage == 0)
            voltage = 0.1;
    }

    public static double getVoltage() {
        try {
            if (voltage <= 0.5)
                throw new ReadingsNotUpdatedException();
            return voltage;
        } catch (ReadingsNotUpdatedException e){
            throw new RuntimeException(e);
        }
    }

    public static void debug(Telemetry telemetry){
        telemetry.addData("voltage", voltage);
    }
}
