package competition_op_modes;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.List;

import controllers.ColorController;
import controllers.DawgTimer;
import controllers.DepositController;
import controllers.ExtendoController;
import controllers.IntakeController;
import controllers.LiftController;
import controllers.PitcherController;
import controllers.StateMachineController;
import controllers.VoltageProcessor;
import controllers.limelight.LLController;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(group = "")
public class MaybeSevenYellowBasketAuto extends OpMode {
    private enum PathStates{
        START(new Pose(10, 52, 0)),
        SCORE_PRELOAD(new Pose(22.5, 66.5, -PI / 5.0)),
        SCORE_FIRST(new Pose(22.5, 56.5, -PI / 8.0)),
        SCORE_SECOND(new Pose(22.5, 46.5, 0)),
        SCORE_THIRD(new Pose(22.5, 36.5, -PI / 5.0)),
        SCORE_SUB(new Pose(22.5, 66.5, -PI / 4.0)),
        RELEASING(null),
        PICKUP_FIRST(new Pose(22.5, 66.5, 0)),
        PICKUP_SECOND(new Pose(22.5, 56.5, 0)),
        PICKUP_THIRD(new Pose(22.5, 46.5, PI / 2.0)),
        GRABBING(null),
        PARK(new Pose(62, 28, -PI / 2.0)),
        APPROACH_SUBMERSIBLE(new Pose(62, 25, -PI / 2.0)),
        SHIFT_SUBMERSIBLE(new Pose(80, 25, -PI / 2.0)),
        SEARCH(null),
        GO_TO_DETECTION(null),
        SUB_WAIT_SAMPLE_GRAB(null),
        ;
        Pose finalPose;
        PathStates(Pose finalPose){
            this.finalPose = finalPose;
        }
    }

    PathChain scorePreloadPath;
    PathChain parkPath;
    PathChain[] approachSubmersiblePaths = {null, null, null, null, null};
    PathChain[] scorePaths = {null, null, null, null, null, null, null, null, null};
    PathChain shiftSubmersiblePath;

    PathChain[] grabPaths = {null, null, null, null, null};
    PathChain[] aimPaths = {null, null, null, null, null};

    public class AllianceNotOverridenException extends Exception {}
    public ColorController.COLORS getAllianceColor(){
        return null;
    }
    Follower follower;
    DawgTimer actionTimer  = new DawgTimer();
    DawgTimer retractTimer = new DawgTimer();

    ExtendoController extendoController;
    LiftController liftController;
    DepositController depositController;
    StateMachineController stateMachineController;
    IntakeController intakeController;
    PitcherController pitcherController;
    LLController llController;

    List<LynxModule> allHubs;

    PathStates pathState = PathStates.START;
    private void setPathState(PathStates state){
        pathState = state;
        actionTimer.reset();
    }

    @Override
    public void init(){
        int[] unwanted = {0, 1};
        if (getAllianceColor() == ColorController.COLORS.RED)
            unwanted[1] = -1;
        if (getAllianceColor() == ColorController.COLORS.BLUE)
            unwanted[0] = -1;
        FollowerConstants.pathEndTValueConstraint = 0.95;
        FollowerConstants.pathEndVelocityConstraint = 4;
        FollowerConstants.pathEndTranslationalConstraint = 1;
        FollowerConstants.pathEndHeadingConstraint = Math.PI / 36.0;

        DawgTimer.updateAllTimers();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.initialize();
        follower.setStartingPose(PathStates.START.finalPose);
        buildPaths();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        VoltageProcessor.initializeSensor(hardwareMap);

        extendoController = new ExtendoController(hardwareMap);
        liftController = new LiftController(hardwareMap);
        stateMachineController = new StateMachineController(hardwareMap, extendoController, liftController, null);
        intakeController = stateMachineController.intakeController;
        intakeController.setClawClosed(true);
        depositController = stateMachineController.depositController;
        depositController.setClawClosed(true);
        depositController.setState(DepositController.DepositArmStates.START);
        pitcherController = new PitcherController(hardwareMap);
        pitcherController.setActivated(false);
        llController = new LLController(hardwareMap, telemetry, unwanted, follower, false);

        stateMachineController.update();

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        retractTimer = new DawgTimer();
        retractTimer.reset();
    }

    void buildPaths(){
        scorePreloadPath = follower.pathBuilder().addPath(new Path(new BezierCurve(
                new Point(PathStates.START.finalPose),
                new Point(PathStates.START.finalPose.getX() + 15, PathStates.START.finalPose.getY() + 15),
                new Point(PathStates.SCORE_PRELOAD.finalPose)
        ))).build();
        scorePreloadPath.getPath(0).setLinearHeadingInterpolation(PathStates.START.finalPose.getHeading(), PathStates.SCORE_PRELOAD.finalPose.getHeading());
        grabPaths[0] = follower.pathBuilder().addPath(new Path(new BezierCurve(
                new Point(PathStates.SCORE_PRELOAD.finalPose),
                new Point(PathStates.SCORE_PRELOAD.finalPose.getX() + 4, PathStates.SCORE_PRELOAD.finalPose.getY() - 4),
                new Point(PathStates.PICKUP_FIRST.finalPose.getX() - 8, PathStates.PICKUP_FIRST.finalPose.getY()),
                new Point(PathStates.PICKUP_FIRST.finalPose)
        ))).build();
//        grabPaths[0].getPath(0).setTangentHeadingInterpolation();
        grabPaths[0].getPath(0).setPathEndTimeoutConstraint(0);
        grabPaths[0].getPath(0).setLinearHeadingInterpolation(PathStates.SCORE_PRELOAD.finalPose.getHeading(), PathStates.PICKUP_FIRST.finalPose.getHeading());
        grabPaths[1] = follower.pathBuilder().addPath(new Path(new BezierCurve(
                new Point(PathStates.SCORE_FIRST.finalPose),
                new Point(PathStates.SCORE_FIRST.finalPose.getX() + 4, PathStates.SCORE_FIRST.finalPose.getY() - 4),
                new Point(PathStates.PICKUP_SECOND.finalPose.getX() - 8, PathStates.PICKUP_SECOND.finalPose.getY()),
                new Point(PathStates.PICKUP_SECOND.finalPose)
        ))).build();
//        grabPaths[1].getPath(0).setTangentHeadingInterpolation();
        grabPaths[1].getPath(0).setPathEndTimeoutConstraint(0);
        grabPaths[1].getPath(0).setLinearHeadingInterpolation(PathStates.SCORE_FIRST.finalPose.getHeading(), PathStates.PICKUP_SECOND.finalPose.getHeading());
        grabPaths[2] = follower.pathBuilder().addPath(new Path(new BezierCurve(
                new Point(PathStates.SCORE_SECOND.finalPose),
                new Point(PathStates.SCORE_SECOND.finalPose.getX() + 4, PathStates.SCORE_SECOND.finalPose.getY() - 4),
                new Point(PathStates.PICKUP_THIRD.finalPose.getX() - 10, PathStates.PICKUP_THIRD.finalPose.getY() - 10),
                new Point(PathStates.PICKUP_THIRD.finalPose.getX(), PathStates.PICKUP_THIRD.finalPose.getY() - 10),
                new Point(PathStates.PICKUP_THIRD.finalPose)
        ))).build();
//        grabPaths[2].getPath(0).setTangentHeadingInterpolation();
        grabPaths[2].getPath(0).setPathEndTimeoutConstraint(0);
        grabPaths[2].getPath(0).setLinearHeadingInterpolation(PathStates.SCORE_SECOND.finalPose.getHeading(), PathStates.PICKUP_THIRD.finalPose.getHeading());

        aimPaths[0] = follower.pathBuilder().addPath(new Path(new BezierCurve(
                new Point(PathStates.PICKUP_FIRST.finalPose),
                new Point(PathStates.PICKUP_FIRST.finalPose.getX() - 6, PathStates.PICKUP_FIRST.finalPose.getY()),
                new Point(PathStates.SCORE_FIRST.finalPose.getX() + 5, PathStates.SCORE_FIRST.finalPose.getY() - 5),
                new Point(PathStates.SCORE_FIRST.finalPose)
        ))).build();
//        aimPaths[0].getPath(0).setTangentHeadingInterpolation();
        aimPaths[0].getPath(0).setReversed(true);
        aimPaths[0].getPath(0).setPathEndTimeoutConstraint(0);
        aimPaths[0].getPath(0).setLinearHeadingInterpolation(PathStates.PICKUP_FIRST.finalPose.getHeading(), PathStates.SCORE_FIRST.finalPose.getHeading());
        aimPaths[1] = follower.pathBuilder().addPath(new Path(new BezierCurve(
                new Point(PathStates.PICKUP_SECOND.finalPose),
                new Point(PathStates.PICKUP_SECOND.finalPose.getX() - 6, PathStates.PICKUP_SECOND.finalPose.getY()),
//                new Point(PathStates.APPROACH_SCORE.finalPose.getX() + 10, PathStates.APPROACH_SCORE.finalPose.getY() - 3),
                new Point(PathStates.SCORE_SECOND.finalPose.getX() + 3, PathStates.SCORE_SECOND.finalPose.getY() - 3),
                new Point(PathStates.SCORE_SECOND.finalPose)
        ))).build();
//        aimPaths[1].getPath(0).setTangentHeadingInterpolation();
        aimPaths[1].getPath(0).setReversed(true);
        aimPaths[1].getPath(0).setPathEndTimeoutConstraint(0);
        aimPaths[1].getPath(0).setLinearHeadingInterpolation(PathStates.PICKUP_SECOND.finalPose.getHeading(), PathStates.SCORE_SECOND.finalPose.getHeading());
        aimPaths[2] = follower.pathBuilder().addPath(new Path(new BezierCurve(
                new Point(PathStates.PICKUP_THIRD.finalPose),
                new Point(PathStates.PICKUP_THIRD.finalPose.getX(), PathStates.PICKUP_THIRD.finalPose.getY() - 10),
                new Point(PathStates.PICKUP_THIRD.finalPose.getX() - 10, PathStates.PICKUP_THIRD.finalPose.getY() - 10),
                new Point(PathStates.SCORE_THIRD.finalPose.getX() + 5, PathStates.SCORE_THIRD.finalPose.getY() - 5),
                new Point(PathStates.SCORE_THIRD.finalPose)
        ))).build();
//        aimPaths[2].getPath(0).setTangentHeadingInterpolation();
        aimPaths[2].getPath(0).setReversed(true);
        aimPaths[2].getPath(0).setPathEndTimeoutConstraint(0);
        aimPaths[2].getPath(0).setLinearHeadingInterpolation(PathStates.PICKUP_THIRD.finalPose.getHeading(), PathStates.SCORE_THIRD.finalPose.getHeading());

        parkPath = follower.pathBuilder().addPath(new Path(new BezierCurve(
                new Point(PathStates.SCORE_THIRD.finalPose),
                new Point(PathStates.PARK.finalPose.getX() + 5, PathStates.PARK.finalPose.getY() + 30),
                new Point(PathStates.PARK.finalPose.getX(), PathStates.PARK.finalPose.getY() + 20),
                new Point(PathStates.PARK.finalPose)
        ))).build();
        parkPath.getPath(0).setLinearHeadingInterpolation(PathStates.SCORE_THIRD.finalPose.getHeading(), PathStates.PARK.finalPose.getHeading());
    }

    @Override
    public void init_loop(){
        DawgTimer.updateAllTimers();
        for (LynxModule hub : allHubs)
            hub.clearBulkCache();
        if (retractTimer.milliseconds() < 2000){
            extendoController.forceMove(-0.5);
        }
        extendoController.update();
        stateMachineController.update();
        if (gamepad1.dpad_up)
            PathStates.APPROACH_SUBMERSIBLE.finalPose = new Pose(80, PathStates.APPROACH_SUBMERSIBLE.finalPose.getY(), PathStates.APPROACH_SUBMERSIBLE.finalPose.getHeading());
        if (gamepad1.dpad_left || gamepad1.dpad_right)
            PathStates.APPROACH_SUBMERSIBLE.finalPose = new Pose(71, PathStates.APPROACH_SUBMERSIBLE.finalPose.getY(), PathStates.APPROACH_SUBMERSIBLE.finalPose.getHeading());
        if (gamepad1.dpad_down)
            PathStates.APPROACH_SUBMERSIBLE.finalPose = new Pose(62, PathStates.APPROACH_SUBMERSIBLE.finalPose.getY(), PathStates.APPROACH_SUBMERSIBLE.finalPose.getHeading());

        telemetry.addData("current pos", (PathStates.APPROACH_SUBMERSIBLE.finalPose.getX() < 69 ? "close" : PathStates.APPROACH_SUBMERSIBLE.finalPose.getX() < 76 ? "mid" : "far"));
        telemetry.update();
    }

    @Override
    public void start(){
        for (int iteration = 0; iteration <= 4; iteration++){
            approachSubmersiblePaths[iteration] = follower.pathBuilder().addPath(new Path(new BezierCurve(
                    new Point(PathStates.SCORE_FIRST.finalPose),
                    new Point(PathStates.SCORE_FIRST.finalPose.getX() + 4, PathStates.SCORE_FIRST.finalPose.getY() - 4),
                    new Point(PathStates.APPROACH_SUBMERSIBLE.finalPose.getX() + 5, PathStates.APPROACH_SUBMERSIBLE.finalPose.getY() + 30),
                    new Point(PathStates.APPROACH_SUBMERSIBLE.finalPose.getX(), PathStates.APPROACH_SUBMERSIBLE.finalPose.getY() + 20),
                    new Point(PathStates.APPROACH_SUBMERSIBLE.finalPose)
            ))).build();
            approachSubmersiblePaths[iteration].getPath(0).setTangentHeadingInterpolation();
            //        approachSubmersiblePaths[iteration].setLinearHeadingInterpolation(PathStates.SCORE.finalPose.getHeading(), PathStates.APPROACH_SUBMERSIBLE.finalPose.getHeading());
        }
    }

    @Override
    public void loop(){
        DawgTimer.updateAllTimers();
        for (LynxModule hub : allHubs)
            hub.clearBulkCache();

        if (liftController.getCurrentPosition() > 1300)
//            follower.setMaxPower(0.65);
//        else if (liftController.getCurrentPosition() > 600)
            follower.setMaxPower(0.75);
        else
            follower.setMaxPower(1);

        updatePath();

        stateMachineController.update();
        liftController.update();
        extendoController.update();
        telemetry.addData("detection Pose Y", detectionPose.getY());
        telemetry.addData("difference X", differencePose.getX());
        telemetry.addData("detection angle", detectionAngle);
        telemetry.addData("state", pathState);
        follower.telemetryDebug(telemetry);

        follower.update();
    }

    private int scoreIteration = 0;
    Pose detectionPose = new Pose();
    Pose differencePose = new Pose();
    Pose samplePose = new Pose();
    Pose shiftSubmersiblePose = new Pose();
    Point recoverPoint = new Point(0, 0);
    double detectionAngle = 0;

    void updatePath(){
        switch (pathState){
            case START:
                setPathState(PathStates.SCORE_PRELOAD);
                LiftController.setFinalTargetPosition(LiftController.UPPER_BASKET_HEIGHT);
                depositController.setState(DepositController.DepositArmStates.BASKET);
                    follower.followPath(scorePreloadPath, true);
                break;
            case SCORE_PRELOAD:
                if (stateMachineController.getActionState() == StateMachineController.ActionStates.FREE && (depositController.getCurrentState() == DepositController.DepositArmStates.HUMAN_DROP || depositController.getCurrentState() == DepositController.DepositArmStates.BASKET)){
                    LiftController.setFinalTargetPosition(LiftController.UPPER_BASKET_HEIGHT);
                    ExtendoController.setFinalTargetPosition(ExtendoController.topLimit, true);
                }
                if (follower.atPoint(new Point(PathStates.SCORE_PRELOAD.finalPose), 2, 2) && liftController.getCurrentPosition() > 1450){
                    depositController.setState(DepositController.DepositArmStates.BASKET_PREDROP);
                    setPathState(PathStates.RELEASING);
                }
                break;
            case RELEASING:
                if (actionTimer.milliseconds() > 200)
                    stateMachineController.release();
                if (actionTimer.milliseconds() > 400){
                    switch (scoreIteration){
                        case 0:
                            follower.followPath(grabPaths[0], true);
                            setPathState(PathStates.PICKUP_FIRST);
                            break;
                        case 1:
                            follower.followPath(grabPaths[1], true);
                            setPathState(PathStates.PICKUP_SECOND);
                            break;
                        case 2:
                            follower.followPath(grabPaths[2], true);
                            intakeController.setTwist(IntakeController.TwisterStates.RIGHT_FULL);
                            setPathState(PathStates.PICKUP_THIRD);
                            break;
                        case 5:
                            follower.followPath(parkPath, true);
                            setPathState(PathStates.PARK);
                            break;
                        default:
                            follower.followPath(approachSubmersiblePaths[scoreIteration - 3], true);
                            setPathState(PathStates.APPROACH_SUBMERSIBLE);
                            break;
                    }
                    scoreIteration++;
                }
                break;
            case PICKUP_FIRST:
                if (!follower.atPoint(new Point(PathStates.SCORE_FIRST.finalPose), 4, 4))
                    LiftController.setFinalTargetPosition(LiftController.bottomLimit);
                if (follower.atPoint(new Point(PathStates.PICKUP_FIRST.finalPose), 9, 9)){
                    follower.setMaxPower(0.65);
                }
//                if (follower.atPoint(new Point(PathStates.PICKUP_FIRST.finalPose), 1, 1)){
                if (actionTimer.milliseconds() > 5000 || (follower.atPose(PathStates.PICKUP_FIRST.finalPose, 1, 0.5, PI / 45.0) && follower.getVelocityMagnitude() < 14)){
                    setPathState(PathStates.GRABBING);
                    stateMachineController.grab();
                }
                break;
            case SCORE_FIRST:
                if (stateMachineController.getActionState() == StateMachineController.ActionStates.FREE && depositController.getCurrentState()== DepositController.DepositArmStates.TRANSFER_READY)
                    stateMachineController.transfer();
                if (stateMachineController.getActionState() == StateMachineController.ActionStates.FREE && (depositController.getCurrentState() == DepositController.DepositArmStates.HUMAN_DROP || depositController.getCurrentState() == DepositController.DepositArmStates.BASKET)){
                    LiftController.setFinalTargetPosition(LiftController.UPPER_BASKET_HEIGHT);
                    ExtendoController.setFinalTargetPosition(ExtendoController.topLimit, true);
                }
                if (follower.atPoint(new Point(PathStates.SCORE_FIRST.finalPose), 2, 2) && liftController.getCurrentPosition() > 1450){
                    depositController.setState(DepositController.DepositArmStates.BASKET_PREDROP);
                    setPathState(PathStates.RELEASING);
                }
                break;
            case PICKUP_SECOND:
                if (!follower.atPoint(new Point(PathStates.SCORE_FIRST.finalPose), 4, 4))
                    LiftController.setFinalTargetPosition(LiftController.bottomLimit);
                if (follower.atPoint(new Point(PathStates.PICKUP_SECOND.finalPose), 6, 6)){
                    follower.setMaxPower(0.65);
                }
                if (actionTimer.milliseconds() > 5000 || (follower.atPose(PathStates.PICKUP_SECOND.finalPose, 1, 0.5, PI / 45.0) && follower.getVelocityMagnitude() < 14)){
//                if (follower.atPoint(new Point(PathStates.PICKUP_SECOND.finalPose), 1, 1)){
                    setPathState(PathStates.GRABBING);
                    stateMachineController.grab();
                }
                break;
            case SCORE_SECOND:
                if (stateMachineController.getActionState() == StateMachineController.ActionStates.FREE && depositController.getCurrentState()== DepositController.DepositArmStates.TRANSFER_READY)
                    stateMachineController.transfer();
                if (stateMachineController.getActionState() == StateMachineController.ActionStates.FREE && (depositController.getCurrentState() == DepositController.DepositArmStates.HUMAN_DROP || depositController.getCurrentState() == DepositController.DepositArmStates.BASKET)){
                    LiftController.setFinalTargetPosition(LiftController.UPPER_BASKET_HEIGHT);
                    ExtendoController.setFinalTargetPosition(ExtendoController.topLimit, true);
                }
                if (follower.atPoint(new Point(PathStates.SCORE_SECOND.finalPose), 2, 2) && liftController.getCurrentPosition() > 1450){
                    depositController.setState(DepositController.DepositArmStates.BASKET_PREDROP);
                    setPathState(PathStates.RELEASING);
                }
                break;
            case PICKUP_THIRD:
                if (!follower.atPoint(new Point(PathStates.SCORE_FIRST.finalPose), 4, 4))
                    LiftController.setFinalTargetPosition(LiftController.bottomLimit);
                if (follower.atPoint(new Point(PathStates.PICKUP_THIRD.finalPose), 6, 6))
                    follower.setMaxPower(0.65);
                if (actionTimer.milliseconds() > 5000 || (follower.atPose(PathStates.PICKUP_THIRD.finalPose, 1, 0.5, PI / 45.0) && follower.getVelocityMagnitude() < 14)){
//                if (follower.atPoint(new Point(PathStates.PICKUP_THIRD.finalPose), 1, 1)){
                    setPathState(PathStates.GRABBING);
                    stateMachineController.grab();
                }
                break;
            case SCORE_THIRD:
                if (stateMachineController.getActionState() == StateMachineController.ActionStates.FREE && depositController.getCurrentState()== DepositController.DepositArmStates.TRANSFER_READY)
                    stateMachineController.transfer();
                if (stateMachineController.getActionState() == StateMachineController.ActionStates.FREE && (depositController.getCurrentState() == DepositController.DepositArmStates.HUMAN_DROP || depositController.getCurrentState() == DepositController.DepositArmStates.BASKET)){
                    LiftController.setFinalTargetPosition(LiftController.UPPER_BASKET_HEIGHT);
                    ExtendoController.setFinalTargetPosition(ExtendoController.topLimit, true);
                }
                if (follower.atPoint(new Point(PathStates.SCORE_THIRD.finalPose), 2, 2) && liftController.getCurrentPosition() > 1450){
                    depositController.setState(DepositController.DepositArmStates.BASKET_PREDROP);
                    setPathState(PathStates.RELEASING);
                }
                break;
            case GRABBING:
                follower.setMaxPower(0.5);
                if (stateMachineController.getActionState() == StateMachineController.ActionStates.FREE){
                    if (scoreIteration <= 2)
                        stateMachineController.transfer();
                    if (scoreIteration <= 3)
                        follower.followPath(aimPaths[scoreIteration - 1], true);
                    switch (scoreIteration){
                        case 1:
                            setPathState(PathStates.SCORE_FIRST);
                            break;
                        case 2:
                            setPathState(PathStates.SCORE_SECOND);
                            break;
                        case 3:
                            setPathState(PathStates.SCORE_THIRD);
                            break;
                        default:
                            setPathState(PathStates.SCORE_SUB);
                            break;
                    }
                }
                break;
            case APPROACH_SUBMERSIBLE:
                if (!follower.atPoint(new Point(PathStates.SCORE_FIRST.finalPose), 4, 4))
                    LiftController.setFinalTargetPosition(LiftController.bottomLimit);
                if (follower.atPoint(new Point(PathStates.APPROACH_SUBMERSIBLE.finalPose), 6, 6))
                    follower.setMaxPower(0.65);
                if (actionTimer.milliseconds() > 5000 || (follower.atPose(PathStates.APPROACH_SUBMERSIBLE.finalPose, 5, 12, PI / 30.0) && follower.getVelocityMagnitude() < 14)){
                    setPathState(PathStates.SEARCH);
                }
                break;
            case SHIFT_SUBMERSIBLE:
                if (follower.atPoint(new Point(shiftSubmersiblePose), 6, 6))
                    follower.setMaxPower(0.65);
                if (actionTimer.milliseconds() > 5000 || (follower.atPose(shiftSubmersiblePose, 5, 12, PI / 30.0) && follower.getVelocityMagnitude() < 10)){
                    setPathState(PathStates.SEARCH);
                }
                break;
            case SEARCH:
                if (actionTimer.milliseconds() > 400){
                    llController.find();
                    differencePose = llController.getDifference();
                    if (llController.noDetections()){
                        setPathState(PathStates.SUB_WAIT_SAMPLE_GRAB);
                        break;
                    }
                    if (differencePose.getX() < 0)
                        detectionPose = new Pose(follower.getPose().getX() + differencePose.getY(), follower.getPose().getY() - differencePose.getX(), PathStates.APPROACH_SUBMERSIBLE.finalPose.getHeading());
                    else {
                        detectionPose = new Pose(follower.getPose().getX() + differencePose.getY(), follower.getPose().getY() + 0.5, PathStates.APPROACH_SUBMERSIBLE.finalPose.getHeading());
                        ExtendoController.setFinalTargetPosition(33.3 * differencePose.getX());
                    }
                    samplePose = llController.getSample();
                    PathChain toDetection = follower.pathBuilder().addPath(new Path(new BezierCurve(
                            new Point(follower.getPose()),
                            new Point(detectionPose)
                    ))).build();
                    toDetection.getPath(0).setConstantHeadingInterpolation(PathStates.APPROACH_SUBMERSIBLE.finalPose.getHeading());
                    follower.followPath(toDetection, true);
                    detectionAngle = llController.getAngle();
                    if (detectionAngle != 0) {
                        intakeController.setTwist(IntakeController.TwisterStates.RIGHT_FULL);
                    } else {
                        intakeController.setTwist(IntakeController.TwisterStates.STRAIGHT);
                    }
//                    follower.setMaxPower(0.6);
                    setPathState(PathStates.GO_TO_DETECTION);
//                    LiftController.setFinalTargetPosition(200);
                }
                break;
            case GO_TO_DETECTION:
                depositController.setState(DepositController.DepositArmStates.TRANSFER_READY);
                if ((follower.atPose(detectionPose, 1, 1, PI / 36.0)) && follower.getVelocityMagnitude() < 14 && !extendoController.isBusy()){
//                if (!follower.isBusy() && !extendoController.isBusy()){
                    setPathState(PathStates.SUB_WAIT_SAMPLE_GRAB);
                    stateMachineController.grab();
                }
                break;
            case SUB_WAIT_SAMPLE_GRAB:
                if (stateMachineController.getActionState() == StateMachineController.ActionStates.FREE){
                    Path fromSubApproachPath = new Path(new BezierCurve(
                            new Point(follower.getPose()),
                            new Point(follower.getPose().getX(), follower.getPose().getY() + 12),
                            new Point(PathStates.SCORE_SUB.finalPose.getX() + 4, PathStates.SCORE_SUB.finalPose.getY() - 4),
                            new Point(PathStates.SCORE_SUB.finalPose.getX() + 16, PathStates.SCORE_SUB.finalPose.getY() - 4),
                            new Point(PathStates.SCORE_SUB.finalPose)
                    ));
//                    fromSubApproachPath.setConstantHeadingInterpolation(PathStates.APPROACH_SCORE.finalPose.getHeading());
                    fromSubApproachPath.setTangentHeadingInterpolation();
                    fromSubApproachPath.setReversed(true);
                    follower.setMaxPower(1);
                    follower.followPath(fromSubApproachPath, true);
                    ExtendoController.setFinalTargetPosition(ExtendoController.bottomLimit);
                    setPathState(PathStates.SCORE_SUB);
                }
                break;
            case PARK:
                if (!follower.atPoint(new Point(PathStates.SCORE_FIRST.finalPose), 4, 4))
                    LiftController.setFinalTargetPosition(540);
                if (follower.atPoint(new Point(PathStates.PARK.finalPose), 20, 20)){
                    depositController.setState(DepositController.DepositArmStates.AUTO_PARK);
                    follower.setMaxPower(0.65);
                    if (follower.getVelocityMagnitude() < 14){
                        LiftController.setFinalTargetPosition(420);
//                        follower.breakFollowing();
                    }
                }
                break;
        }
    }
}
