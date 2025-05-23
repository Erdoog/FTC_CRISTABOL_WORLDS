package controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import opmodes.ServoAndControllersCalibrate;

public class PitcherController {
    DawgServo leftServo;
    DawgServo rightServo;

    boolean state = false;

    public PitcherController(HardwareMap hardwareMap){
        leftServo = new DawgServo(hardwareMap, "leftPitcher");
        rightServo = new DawgServo(hardwareMap, "rightPitcher");
        rightServo.setDirection(Servo.Direction.REVERSE);
    }

    public void setActivated(boolean state){
        this.state = state;
        double pos = state ? 0.95 : .62;
        leftServo.setPosition(pos);
        rightServo.setPosition(pos + ServoAndControllersCalibrate.PITCHER_RIGHT_OFFSET);
    }

    public boolean getActivated(){
        return state;
    }

    public void switchActivated(){
        state = !state;
        setActivated(state);
    }

    public void pwmDisable(){
        leftServo.pwmDisable();
        rightServo.pwmDisable();
    }
}
