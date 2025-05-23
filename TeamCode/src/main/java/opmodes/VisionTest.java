package opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import controllers.ColorController;
import controllers.DawgTimer;
import controllers.DepositController;
import controllers.ExtendoController;
import controllers.IntakeController;
import controllers.LiftController;
import controllers.StateMachineController;
import controllers.VisionController;
import controllers.VoltageProcessor;

@Config
public class VisionTest extends LinearOpMode {
    public static double xkP = 0.003, ykP = 0.006, kI = 0.02;

    public class AllianceNotOverridenException extends Exception {}
    public ColorController.COLORS getAllianceColor(){
        throw new RuntimeException(new AllianceNotOverridenException());
    }

    @Override
    public void runOpMode(){
        VoltageProcessor.initializeSensor(hardwareMap);
        VisionController visionController = new VisionController(hardwareMap, getAllianceColor());
        LiftController liftController = new LiftController(hardwareMap);
        ExtendoController extendoController = new ExtendoController(hardwareMap);
        StateMachineController stateMachineController = new StateMachineController(hardwareMap, extendoController, liftController, null);
        IntakeController intakeController = stateMachineController.intakeController;
        DepositController depositController = stateMachineController.depositController;
        intakeController.setState(IntakeController.IntakeArmStates.CAMERA);

        PIDController xPID = new PIDController(xkP, kI, 0);
        PIDController yPID = new PIDController(ykP, kI, 0);

        Motor leftBack = new Motor(hardwareMap, "leftBack");
        Motor leftFront = new Motor(hardwareMap, "leftFront");
        Motor rightBack = new Motor(hardwareMap, "rightBack");
        Motor rightFront = new Motor(hardwareMap, "rightFront");

        HDrive drive = new HDrive(leftFront, rightFront, leftBack, rightBack);
        intakeController.update();
        waitForStart();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        while (opModeIsActive()){
            DawgTimer.updateAllTimers();
//            VoltageProcessor.updateReadings();
            xPID.setPID(xkP, kI, 0);
            yPID.setPID(ykP, kI, 0);
            VisionController.Location location = visionController.getLargestLocation(false);
            double x = location.x;
            double y = -location.y;
            double angle = location.angle;
            intakeController.setTwistAngle(angle);
            if (Double.isNaN(x)){
                drive.driveRobotCentric(0, 0, 0);
                continue;
            }

            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("angle", angle);
            telemetry.update();

            intakeController.update();
            double xPower = xPID.calculate(x, 0);
            double yPower = yPID.calculate(y, 0);

//            drive.driveRobotCentric(xPower, yPower ,0);
        }
    }
}
