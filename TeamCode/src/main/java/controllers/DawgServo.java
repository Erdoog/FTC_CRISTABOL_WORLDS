package controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import util.LerpController;

@Config
public class DawgServo {
    Servo servo;
    DawgTimer actionTimer = new DawgTimer();
    LerpController lerpController = new LerpController();
    public double speedRangePerSecond = 0.85;
    public double estimatedPosition = 0.5;
    public double finalTargetPosition = -1;

    public DawgServo(HardwareMap hardwareMap, String name){
        servo = hardwareMap.servo.get(name);
    }
    public void setSpeedRangePerSecond(double speed){
        speedRangePerSecond = speed;
    }
    public void setDirection(Servo.Direction direction){
        servo.setDirection(direction);
    }
    public void setPosition(double position){
        setPosition(position, false);
    }

    public void setPosition(double position, boolean useLerp){
        updateEstimatedPos();
        if (!useLerp)
            lerpController.interrupt();
        if (position < 0.0 || 1.0 < position || position == finalTargetPosition){
            double currentTarget = finalTargetPosition;
            if (lerpController.isBusy() && useLerp){
                currentTarget = lerpController.getPosition();
            }
            servo.setPosition(currentTarget);
        } else {
            if (useLerp)
                lerpController.beginMovement(finalTargetPosition, position, Math.abs(finalTargetPosition - position) / speedRangePerSecond * 1050, false);
            finalTargetPosition = position;
            double currentTarget = finalTargetPosition;
            if (lerpController.isBusy() && useLerp){
                currentTarget = lerpController.getPosition();
            }
            servo.setPosition(currentTarget);
        }
        if (lerpController.getEndPos() != finalTargetPosition)
            lerpController.interrupt();
    }

    public void updateEstimatedPos(){
        double deltaSeconds = actionTimer.milliseconds() / 1000.0;
        actionTimer.reset();
        double errorMagnitude = Math.abs(finalTargetPosition - estimatedPosition);
        if (errorMagnitude == 0)
            return;
        double direction = (finalTargetPosition - estimatedPosition) / errorMagnitude;
        double voltageCompensationMultiplier = 1.09;
//        voltageCompensationMultiplier = VoltageProcessor.getVoltage() / 12.0;
        double increase = Math.min(errorMagnitude, speedRangePerSecond * voltageCompensationMultiplier * deltaSeconds) * direction;
        estimatedPosition += increase;
    }
    public boolean getBusy(){
        updateEstimatedPos();
        return estimatedPosition != finalTargetPosition;
    }

    public void pwmDisable(){
        servo.getController().pwmDisable();
    }
}
