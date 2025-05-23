package controllers;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorController {
    ColorSensor colorSensor;

    public static COLORS latestColor = COLORS.YELLOW;

    public ColorController(HardwareMap hardwareMap){
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
    }

    public enum COLORS {
        NONE, YELLOW, RED, BLUE
    }

    int green = 0;
    int blue = 0;
    int red = 0;

    public COLORS getColor(){
        float[] hsv = {0, 0, 0};
        green = (int)(colorSensor.green() * 0.0255);
        blue =  (int)(colorSensor.blue()  * 0.0255);
        red =   (int)(colorSensor.red()   * 0.0510);
        Color.RGBToHSV(red, green, blue, hsv);
        double hue = hsv[0];
        double saturation = hsv[1];
        double value = hsv[2];

        if (saturation < 0.58 || value < 0.08)
            return COLORS.NONE;

        if (saturation > 0.68 && 25 < hue && hue < 55)
            return COLORS.YELLOW;
        else if (saturation > 0.68 && (hue > 250 || hue < 20))
            return COLORS.RED;
        else if (219 < hue && hue < 251)
            return COLORS.BLUE;
        else
            return COLORS.NONE;
//        if (green > 9000){
//            latestColor = COLORS.YELLOW;
//            return COLORS.YELLOW;
//        }
//        else if (red > 3500){
//            latestColor = COLORS.RED;
//            return COLORS.RED;
//        }
//        else if (blue > 3500){
//            latestColor = COLORS.BLUE;
//            return COLORS.BLUE;
//        }
//        else
//            return COLORS.NONE;
    }

    public void debug(Telemetry telemetry){
        telemetry.addData("red", red);
        telemetry.addData("blue", blue);
        telemetry.addData("green", green);
    }
}
