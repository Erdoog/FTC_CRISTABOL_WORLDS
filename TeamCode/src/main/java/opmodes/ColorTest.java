package opmodes;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Config
@TeleOp
public class ColorTest extends LinearOpMode {
    public static double scale = 1;

    @Override
    public void runOpMode(){
        ColorSensor colorSensor = hardwareMap.colorSensor.get("colorSensor");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()){
            int red =   (int)(colorSensor.red()   * scale * 2);
            int green = (int)(colorSensor.green() * scale);
            int blue =  (int)(colorSensor.blue()  * scale);
            float[] hsv = {0, 0, 0};

            Color.RGBToHSV(red, green, blue, hsv);

            telemetry.addData("red", red);
            telemetry.addData("blue", blue);
            telemetry.addData("green", green);
            telemetry.addData("hue", hsv[0]);
            telemetry.addData("saturation", hsv[1]);
            telemetry.addData("value", hsv[2]);
            telemetry.update();
        }
    }
}
