package opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;

import controllers.ColorController;
import controllers.DawgTimer;
import controllers.ExtendoController;
import controllers.IntakeController;
import controllers.LiftController;
import controllers.StateMachineController;
import controllers.limelight.LL3ADetection;
import controllers.limelight.LLController;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp
public class LLTest extends LinearOpMode {
    int[] unwanted = {};

    @Override
    public void runOpMode(){
        DawgTimer.updateAllTimers();
        GamepadEx driver1 = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Constants.setConstants(FConstants.class, LConstants.class);
        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(new Pose());
        ExtendoController extendoController = new ExtendoController(hardwareMap);
        LiftController liftController = new LiftController(hardwareMap);
        StateMachineController stateMachineController = new StateMachineController(hardwareMap, extendoController, liftController, ColorController.COLORS.BLUE);
        LLController llController = new LLController(hardwareMap, telemetry, unwanted, follower, false);
        stateMachineController.update();
        waitForStart();
        while (opModeIsActive()){
            DawgTimer.updateAllTimers();
            llController.find();
            driver1.readButtons();
            if (driver1.wasJustPressed(GamepadKeys.Button.A)){
                Pose differencePose = llController.getDifference();
                ExtendoController.setFinalTargetPosition(31.63439937149052 * differencePose.getX());
                Pose detectionPose = new Pose(follower.getPose().getX(), llController.getTarget().getY(), 0);
                Path toDetection = new Path(new BezierCurve(
                        new Point(follower.getPose()),
                        new Point(detectionPose)
                ));
                toDetection.setConstantHeadingInterpolation(0);
                follower.followPath(toDetection);
                double detectionAngle = llController.getAngle();
                if (detectionAngle != 0) {
                    stateMachineController.intakeController.setTwist(IntakeController.TwisterStates.RIGHT_FULL);
                } else {
                    stateMachineController.intakeController.setTwist(IntakeController.TwisterStates.STRAIGHT);
                }
            }
            if (driver1.wasJustPressed(GamepadKeys.Button.X))
                stateMachineController.grab();
            if (driver1.wasJustPressed(GamepadKeys.Button.B))
                stateMachineController.release();
            extendoController.update();
            liftController.update();
            stateMachineController.update();
            follower.update();
            telemetry.addData("detection y", llController.getTarget().getY());
            telemetry.addData("detection x", llController.getTarget().getX());
            telemetry.addData("angle", llController.getAngle());
//            follower.telemetryDebug(telemetry);
            telemetry.update();
        }
        llController.off();
    }
}
