package opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Autonomous")
public class GamepadGlowOpmode extends LinearOpMode {
    @Override
    public void runOpMode(){
        ElapsedTime gamepadTimer = new ElapsedTime();
        while (opModeInInit()){
            double brightness = Math.abs((double)(Math.round(gamepadTimer.milliseconds()) % 923) - 923) / 923;
            gamepad1.setLedColor(brightness, brightness, 0, -1);
            gamepad2.setLedColor(0, 0, brightness, -1);
//            gamepad1.runLedEffect(Gamepad.LedEffect.deserialize());
        }
    }
}
