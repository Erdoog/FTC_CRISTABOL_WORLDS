package opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class Experiment extends LinearOpMode{
    double prevCurrent = 0;
    @Override
    public void runOpMode(){
        MotorEx motor = new MotorEx(hardwareMap, "motor");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ElapsedTime timer = new ElapsedTime();
        waitForStart();
        while (opModeIsActive()){
            motor.set(-gamepad1.left_stick_y);
            double deltaTime = timer.milliseconds() / 1000.0;
            timer.reset();
            double current = motor.motorEx.getCurrent(CurrentUnit.MILLIAMPS) / 1000.0;
            telemetry.addData("Current draw", current);
            telemetry.addData("Rate of change of current draw", (current - prevCurrent) / deltaTime);
            prevCurrent = current;
            telemetry.update();
        }
    }
}