package opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import controllers.DawgTimer;
import controllers.ExtendoController;
import controllers.LiftController;

@Config
@TeleOp
public class ServoAndControllersCalibrate extends LinearOpMode {

    public static double INTAKE_ARM_BOTH = -1;

    public static double PITCHER_LEFT_POS    = 0.65;
    public static double PITCHER_RIGHT_POS   = 0.67;
    public static double PITCHER_BOTH_POS    =-1   ;
    public static double PITCHER_RIGHT_OFFSET = 0.04;

    public static double INTAKE_CLAW    = 0.5;
    public static double INTAKE_WRIST   = 0.5;
    public static double INTAKE_TWISTER = 0.5;
    public static double INTAKE_ARM1    = 0.5;
    public static double INTAKE_ARM2    = 0.5;
    public static double INTAKE_ARM2_OFFSET    = 0.03;

    public static double DEPOSIT_ARM_BOTH = -1;

    public static double DEPOSIT_CLAW   = 0.5;
    public static double DEPOSIT_WRIST  = 0.5;
    public static double DEPOSIT_ARM1   = 0.5;
    public static double DEPOSIT_ARM2   = 0.5;
    public static double DEPOSIT_ARM2_OFFSET    = -0.04;

    public static double LIFT_POSITION    = 0;
    public static double EXTENDO_POSITION = 0;

    @Override
    public void runOpMode(){
        ExtendoController extendoController = new ExtendoController(hardwareMap);
        LiftController liftController = new LiftController(hardwareMap);

        Servo leftPitcher = hardwareMap.servo.get("leftPitcher");
        Servo rightPitcher = hardwareMap.servo.get("rightPitcher");
        rightPitcher.setDirection(Servo.Direction.REVERSE);

        Servo depositClawServo =  hardwareMap.servo.get("depositClaw");
        Servo depositWristServo = hardwareMap.servo.get("depositWrist");
        Servo depositArm1Servo =  hardwareMap.servo.get("depositArm1");
        Servo depositArm2Servo =  hardwareMap.servo.get("depositArm2");
        depositArm2Servo.setDirection(Servo.Direction.REVERSE);

        Servo intakeClawServo = hardwareMap.servo.get("intakeClaw");
        Servo intakeTwisterServo = hardwareMap.servo.get("intakeTwister");
        Servo intakeWristServo = hardwareMap.servo.get("intakeWrist");
        Servo intakeArm1Servo = hardwareMap.servo.get("intakeArm1");
        Servo intakeArm2Servo = hardwareMap.servo.get("intakeArm2");
        intakeArm2Servo.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()){
            DawgTimer.updateAllTimers();
            intakeClawServo.setPosition(INTAKE_CLAW);
            intakeWristServo.setPosition(INTAKE_WRIST);
            intakeTwisterServo.setPosition(INTAKE_TWISTER);
            if (0 <= INTAKE_ARM_BOTH && INTAKE_ARM_BOTH <= 1){
                intakeArm1Servo.setPosition(INTAKE_ARM_BOTH);
                intakeArm2Servo.setPosition(INTAKE_ARM_BOTH + INTAKE_ARM2_OFFSET);
            } else {
                intakeArm1Servo.setPosition(INTAKE_ARM1);
                intakeArm2Servo.setPosition(INTAKE_ARM2);
            }

            depositClawServo.setPosition(DEPOSIT_CLAW);
            depositWristServo.setPosition(DEPOSIT_WRIST);
            if (0 <= DEPOSIT_ARM_BOTH && DEPOSIT_ARM_BOTH <= 1) {
                depositArm1Servo.setPosition(DEPOSIT_ARM_BOTH);
                depositArm2Servo.setPosition(DEPOSIT_ARM_BOTH + DEPOSIT_ARM2_OFFSET);
            } else {
                depositArm1Servo.setPosition(DEPOSIT_ARM1);
                depositArm2Servo.setPosition(DEPOSIT_ARM2);
            }
            if (0 <= PITCHER_BOTH_POS && PITCHER_BOTH_POS <= 1){
                leftPitcher.setPosition(PITCHER_BOTH_POS);
                rightPitcher.setPosition(PITCHER_BOTH_POS + PITCHER_RIGHT_OFFSET);
            } else {
                leftPitcher.setPosition(PITCHER_LEFT_POS);
                rightPitcher.setPosition(PITCHER_RIGHT_POS);
            }

            liftController.setFinalTargetPosition(LIFT_POSITION);
            liftController.update();
            extendoController.setFinalTargetPosition(EXTENDO_POSITION);
            extendoController.update();
        }
        intakeArm1Servo.setPosition(0.5);
        intakeArm2Servo.setPosition(0.5);
        intakeArm1Servo.getController().pwmDisable();
        intakeArm2Servo.getController().pwmDisable();
        leftPitcher.getController().pwmDisable();
        rightPitcher.getController().pwmDisable();
        sleep(200);
    }
}
