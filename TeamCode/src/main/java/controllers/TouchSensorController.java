package controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
public class TouchSensorController {
    TouchSensor leftTouch, rightTouch;

    public TouchSensorController(HardwareMap hardwareMap){
        this.leftTouch = hardwareMap.touchSensor.get("leftTouch");
        this.rightTouch = hardwareMap.touchSensor.get("rightTouch");
    }

    public boolean getAtLeastOnePressed(){
        return (Math.round(leftTouch.getValue()) | Math.round(rightTouch.getValue())) == 1;
    }

    public boolean getBothPressed(){
        return (Math.round(leftTouch.getValue()) & Math.round(rightTouch.getValue())) == 1;
    }
}
