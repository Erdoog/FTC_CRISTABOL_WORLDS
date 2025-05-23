package competition_op_modes;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
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

public class SpecAuto extends OpMode {
    private enum PathStates{
        START(new Pose(0, 72, 0)),
        SCORE_PRELOAD(new Pose(35, 81.5, 0)),
        APPROACH_FIRST(new Pose(53, 36.0, 0)),
        PUSH_FIRST(new Pose(7, 32.0, 0)),
        APPROACH_SECOND(new Pose(53, 26, 0)),
        PUSH_SECOND(new Pose(7, 22, 0)),
        APPROACH_THIRD(new Pose(54, 16.1, 0)),
        PUSH_THIRD(new Pose(7, 16.1, 0)),
        RECOVER_PUSH_THIRD(new Pose(13.2, 18, 0)),
        GO_TO_GRAB(new Pose(-1, 44, 0)),
        WAIT_BEFORE_GRAB(new Pose(-1, 44, 0)),
        WAIT_SPECIMEN_GRAB(new Pose(1, 43, 0)),
        SCORE(new Pose(37, 76.5, 0)),
        SEARCH(new Pose(35, 73, 0)),
        GO_TO_DETECTION(new Pose(0, 0, 0)),
        WAIT_SAMPLE_GRAB(new Pose(0, 0, 0)),
        BRING_SAMPLE(new Pose(8, 18, 0)),
        REST(new Pose(0, 0, 0)),
        ;

        final Pose finalPose;
        PathStates(Pose finalPose){
            this.finalPose = finalPose;
        }
    }

    Path scorePreloadPath;
    Path approachFirstPath;
    Path pushFirstPath;
    Path approachSecondPath;
    Path pushSecondPath;
    Path approachThirdPath;
    Path pushThirdPath;
    Path pushThirdAfterRecoverPath;
    PathChain[] grabPaths =  {null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null};
    PathChain[] scorePaths = {null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null};

    Follower follower;

    StateMachineController stateMachineController;
    DepositController depositController;
    IntakeController intakeController;
    LiftController liftController;
    ExtendoController extendoController;
    PitcherController pitcherController;
    LLController llController;

    DawgTimer actionTimer = new DawgTimer();
    DawgTimer retractTimer = new DawgTimer();
    DawgTimer stuckTimer = new DawgTimer();
    List<LynxModule> allHubs;

    public static class AllianceNotOverridenException extends Exception {}
    public ColorController.COLORS getAllianceColor(){
        throw new RuntimeException(new AllianceNotOverridenException());
    }

    PathStates pathState = PathStates.START;
    private void setPathState(PathStates state){
        pathState = state;
        actionTimer.reset();
    }

    @Override
    public void init(){
        int[] unwanted = {3, 2};
        if (getAllianceColor() == ColorController.COLORS.BLUE){
            unwanted[0] = 1;
        } else if (getAllianceColor() == ColorController.COLORS.RED){
            unwanted[0] = 0;
        } else {
            throw new RuntimeException();
        }

//        FollowerConstants.pathEndVelocityConstraint = 0.1;
//        FollowerConstants.pathEndTranslationalConstraint = 0.1;
//        FollowerConstants.pathEndHeadingConstraint = 0.007;
//        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndTValueConstraint = 0.95;
        FollowerConstants.pathEndVelocityConstraint = 4;
        FollowerConstants.pathEndTranslationalConstraint = 1;
        FollowerConstants.pathEndHeadingConstraint = Math.PI / 36.0;

        DawgTimer.updateAllTimers();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.initialize();
        follower.setStartingPose(PathStates.START.finalPose);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
        llController = new LLController(hardwareMap, telemetry, unwanted, follower, true);

        intakeController.setState(IntakeController.IntakeArmStates.START);

        stateMachineController.update();
        buildPaths();

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        retractTimer = new DawgTimer();
        retractTimer.reset();
    }

    void buildPaths(){
        scorePreloadPath = new Path(new BezierCurve(
                        new Point(PathStates.START.finalPose),
                        new Point(PathStates.SCORE_PRELOAD.finalPose)
        ));
        scorePreloadPath.setConstantHeadingInterpolation(0);
        approachFirstPath = new Path(new BezierCurve(
                        new Point(PathStates.SCORE_PRELOAD.finalPose),
                        new Point(PathStates.SCORE_PRELOAD.finalPose.getX() - 20,  PathStates.SCORE_PRELOAD.finalPose.getY() - 15, Point.CARTESIAN),
                        new Point(PathStates.SCORE_PRELOAD.finalPose.getX() - 20, PathStates.APPROACH_FIRST.finalPose.getY() - 6, Point.CARTESIAN),
                        new Point(PathStates.APPROACH_FIRST.finalPose.getX() - 5, PathStates.APPROACH_FIRST.finalPose.getY() + 10, Point.CARTESIAN),
                        new Point(PathStates.APPROACH_FIRST.finalPose)
        ));
        approachFirstPath.setLinearHeadingInterpolation(PathStates.SCORE_PRELOAD.finalPose.getHeading(), PathStates.APPROACH_FIRST.finalPose.getHeading());
        pushFirstPath = new Path(new BezierCurve(
                        new Point(PathStates.APPROACH_FIRST.finalPose),
                        new Point(PathStates.APPROACH_FIRST.finalPose.getX() - 4, PathStates.APPROACH_FIRST.finalPose.getY() - 7, Point.CARTESIAN),
                        new Point(PathStates.PUSH_FIRST.finalPose)
        ));
        pushFirstPath.setLinearHeadingInterpolation(PathStates.APPROACH_FIRST.finalPose.getHeading(), PathStates.PUSH_FIRST.finalPose.getHeading());
        approachSecondPath = new Path(new BezierCurve(
                        new Point(PathStates.PUSH_FIRST.finalPose),
                        new Point(PathStates.APPROACH_SECOND.finalPose.getX() - 4, PathStates.APPROACH_SECOND.finalPose.getY() + 7, Point.CARTESIAN),
                        new Point(PathStates.APPROACH_SECOND.finalPose)
        ));
        approachSecondPath.setLinearHeadingInterpolation(PathStates.PUSH_FIRST.finalPose.getHeading(), PathStates.APPROACH_SECOND.finalPose.getHeading());
        pushSecondPath = new Path(new BezierCurve(
                        new Point(PathStates.APPROACH_SECOND.finalPose),
                        new Point(PathStates.APPROACH_SECOND.finalPose.getX() - 4, PathStates.APPROACH_SECOND.finalPose.getY() - 7, Point.CARTESIAN),
                        new Point(PathStates.PUSH_SECOND.finalPose)
        ));
        pushSecondPath.setLinearHeadingInterpolation(PathStates.APPROACH_SECOND.finalPose.getHeading(), PathStates.PUSH_SECOND.finalPose.getHeading());


        approachThirdPath = new Path(new BezierCurve(
                        new Point(PathStates.PUSH_SECOND.finalPose),
                        new Point(PathStates.APPROACH_THIRD.finalPose.getX() - 4, PathStates.APPROACH_THIRD.finalPose.getY() + 7, Point.CARTESIAN),
                        new Point(PathStates.APPROACH_THIRD.finalPose)
        ));
        approachThirdPath.setLinearHeadingInterpolation(PathStates.PUSH_SECOND.finalPose.getHeading(), PathStates.APPROACH_THIRD.finalPose.getHeading());
        pushThirdPath = new Path(new BezierCurve(
                new Point(PathStates.APPROACH_THIRD.finalPose),
                new Point(PathStates.PUSH_THIRD.finalPose)
        ));
        pushThirdPath.setLinearHeadingInterpolation(PathStates.APPROACH_THIRD.finalPose.getHeading(), PathStates.PUSH_THIRD.finalPose.getHeading());

        pushThirdAfterRecoverPath = new Path(new BezierCurve(
                new Point(PathStates.APPROACH_THIRD.finalPose),
                new Point(PathStates.PUSH_THIRD.finalPose)
        ));
        pushThirdAfterRecoverPath.setLinearHeadingInterpolation(PathStates.APPROACH_THIRD.finalPose.getHeading(), PathStates.PUSH_THIRD.finalPose.getHeading());

        grabPaths[0] = follower.pathBuilder().addPath(new Path(new BezierCurve(
                new Point(PathStates.PUSH_THIRD.finalPose),
                new Point(PathStates.GO_TO_GRAB.finalPose.getX() + 15, PathStates.GO_TO_GRAB.finalPose.getY(), Point.CARTESIAN),
                new Point(PathStates.GO_TO_GRAB.finalPose)
        ))).build();
        grabPaths[0].getPath(0).setLinearHeadingInterpolation(PathStates.PUSH_THIRD.finalPose.getHeading(), PathStates.GO_TO_GRAB.finalPose.getHeading());
        grabPaths[1] = follower.pathBuilder().addPath(new Path(new BezierCurve(
                new Point(PathStates.BRING_SAMPLE.finalPose),
                new Point(PathStates.GO_TO_GRAB.finalPose.getX() + 15, PathStates.GO_TO_GRAB.finalPose.getY(), Point.CARTESIAN),
                new Point(PathStates.GO_TO_GRAB.finalPose)
        ))).build();
        grabPaths[1].getPath(0).setConstantHeadingInterpolation(0);

        for (int iteration = 0; iteration < 12; iteration++){
            grabPaths[4 + iteration] = follower.pathBuilder().addPath(new Path(new BezierCurve(
                    new Point(PathStates.SCORE.finalPose),
                    new Point(PathStates.SCORE.finalPose.getX() - 12, PathStates.SCORE.finalPose.getY()),
                    new Point(PathStates.GO_TO_GRAB.finalPose.getX() + 12, PathStates.GO_TO_GRAB.finalPose.getY()),
                    new Point(PathStates.GO_TO_GRAB.finalPose)
            ))).build();
//            grabPaths[2 + iteration].setTangentHeadingInterpolation();
//            grabPaths[2 + iteration].setReversed(true);
            grabPaths[4 + iteration].getPath(0).setConstantHeadingInterpolation(0);
        }

        for (int iteration = 0; iteration < 14; iteration++){
            scorePaths[iteration] = follower.pathBuilder().addPath(new Path(new BezierCurve(
                    new Point( PathStates.GO_TO_GRAB.finalPose),
                    new Point(PathStates.GO_TO_GRAB.finalPose.getX() + 5, PathStates.GO_TO_GRAB.finalPose.getY() + 12),
                    new Point(PathStates.SCORE.finalPose.getX() - 6, PathStates.SCORE.finalPose.getY()),
//                    new Point(PathStates.GRAB.finalPose.getX() + 7, PathStates.GRAB.finalPose.getY()),
//                    new Point(PathStates.SCORE.finalPose.getX() - 11.4, PathStates.SCORE.finalPose.getY()),
                    new Point(PathStates.SCORE.finalPose)
            ))).build();
//            scorePaths[iteration].setTangentHeadingInterpolation();
            scorePaths[iteration].getPath(0).setConstantHeadingInterpolation(0);
        }
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
    }

    @Override
    public void start(){
        intakeController.setState(IntakeController.IntakeArmStates.FLOOR_AIM);
    }

    @Override
    public void loop(){
        DawgTimer.updateAllTimers();
        for (LynxModule hub : allHubs)
            hub.clearBulkCache();

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


    private int specimenIteration = 0;
    Pose detectionPose = new Pose();
    Pose differencePose = new Pose();
    Pose samplePose = new Pose();
    Point recoverPoint = new Point(0, 0);
    double detectionAngle = 0;

    void updatePath(){
        switch(pathState){
            case START:
                follower.followPath(scorePreloadPath);
                depositController.setState(DepositController.DepositArmStates.CHAMBER);
                LiftController.setFinalTargetPosition(LiftController.CHAMBER_HEIGHT, true);
                setPathState(PathStates.SCORE_PRELOAD);
                break;
            case SCORE_PRELOAD:
                if ((follower.atPoint(new Point(PathStates.SCORE_PRELOAD.finalPose), 4, 4))
                  || (follower.atPoint(new Point(PathStates.SCORE_PRELOAD.finalPose), 8, 8) && follower.getVelocityMagnitude() < 12)){
                    stateMachineController.release();
                    follower.followPath(approachFirstPath, true);
                    setPathState(PathStates.APPROACH_FIRST);
                }
                break;
            case APPROACH_FIRST:
                if (follower.atPoint(new Point(PathStates.APPROACH_FIRST.finalPose), 15, 15)){
                    follower.setMaxPower(0.8);
                }
                if (follower.atPoint(new Point(PathStates.APPROACH_FIRST.finalPose), 7, 7)
                        || (follower.atPoint(new Point(PathStates.APPROACH_FIRST.finalPose), 8, 8) && !follower.isBusy())){
                    follower.setMaxPower(0.85);
                    follower.followPath(pushFirstPath);
                    setPathState(PathStates.PUSH_FIRST);
                }
                break;
            case PUSH_FIRST:
                if (!follower.atPoint(new Point(PathStates.PUSH_FIRST.finalPose), 20, 50))
                    follower.setMaxPower(1);
                if (follower.atPoint(new Point(PathStates.PUSH_FIRST.finalPose), 6, 6)
                        || (follower.atPoint(new Point(PathStates.PUSH_FIRST.finalPose), 7, 7) && !follower.isBusy())){
                    follower.setMaxPower(1);
                    follower.followPath(approachSecondPath, true);
                    setPathState(PathStates.APPROACH_SECOND);
                }
                break;
            case APPROACH_SECOND:
                if (follower.atPoint(new Point(PathStates.APPROACH_SECOND.finalPose), 15, 15)){
                    follower.setMaxPower(0.8);
                } else {
                    follower.setMaxPower(1);
                }
                if (follower.atPoint(new Point(PathStates.APPROACH_SECOND.finalPose), 7, 7)
                        || (follower.atPoint(new Point(PathStates.APPROACH_SECOND.finalPose), 8, 8) && !follower.isBusy())){
                    follower.followPath(pushSecondPath);
                    setPathState(PathStates.PUSH_SECOND);
                }
                break;
            case PUSH_SECOND:
                if (!follower.atPoint(new Point(PathStates.PUSH_SECOND.finalPose), 20, 50))
                    follower.setMaxPower(1);
                if (follower.atPoint(new Point(PathStates.PUSH_SECOND.finalPose), 6, 6)
                        || (follower.atPoint(new Point(PathStates.PUSH_SECOND.finalPose), 7, 7) && !follower.isBusy())
                ){
                    follower.followPath(approachThirdPath, true);
                    setPathState(PathStates.APPROACH_THIRD);
                }
                break;
            case APPROACH_THIRD:
                if (follower.atPoint(new Point(PathStates.APPROACH_THIRD.finalPose), 15, 15)){
                    follower.setMaxPower(0.8);
                }
                if (follower.atPoint(new Point(PathStates.APPROACH_THIRD.finalPose), 7, 7)
                        || (follower.atPoint(new Point(PathStates.APPROACH_THIRD.finalPose), 8, 8) && !follower.isBusy())
                ){
                    follower.followPath(pushThirdPath);
                    setPathState(PathStates.PUSH_THIRD);
                    follower.setMaxPower(0.75);
                    stuckTimer.reset();
                }
                break;
            case PUSH_THIRD:
                if (!follower.atPoint(new Point(PathStates.PUSH_THIRD.finalPose), 20, 50))
                    follower.setMaxPower(1);
                if (follower.getVelocityMagnitude() > 10)
                    stuckTimer.reset();
                if (stuckTimer.milliseconds() > 1500){
                    recoverPoint = new Point(follower.getPose().getX() + 0.5, follower.getPose().getY() + 2);
                    Path recoverPath = new Path(new BezierCurve(
                            new Point(follower.getPose()),
                            recoverPoint
                    ));
                    recoverPath.setConstantHeadingInterpolation(0);
                    follower.followPath(recoverPath);
                    setPathState(PathStates.RECOVER_PUSH_THIRD);
                }
                if (follower.atPoint(new Point(PathStates.PUSH_THIRD.finalPose), 8, 8)
                        || (follower.atPoint(new Point(PathStates.PUSH_THIRD.finalPose), 10, 10) && !follower.isBusy())
                ){
                    follower.followPath(grabPaths[0], true);
                    setPathState(PathStates.GO_TO_GRAB);
                    follower.setMaxPower(1);
                }
                break;
            case RECOVER_PUSH_THIRD:
                if (follower.atPoint(recoverPoint, 1, 1)
                        || (follower.atPoint(recoverPoint, 5, 5) && !follower.isBusy())
                ){
                    follower.followPath(pushThirdAfterRecoverPath);
                    setPathState(PathStates.PUSH_THIRD);
                }
                break;
            case GO_TO_GRAB:
                if (actionTimer.milliseconds() < 300)
                    stateMachineController.release();
                if (follower.atPoint(new Point(PathStates.GO_TO_GRAB.finalPose), 8.5, 8.5))
                    follower.setMaxPower(0.21);
                else if (follower.atPoint(new Point(PathStates.GO_TO_GRAB.finalPose), 18, 18) && specimenIteration >= 2)
                    follower.setMaxPower(0.7);
                if (follower.atPoint(new Point(PathStates.GO_TO_GRAB.finalPose), 3.5, 3.5)
                 || (follower.atPoint(new Point(PathStates.GO_TO_GRAB.finalPose), 7, 7) && follower.getVelocityMagnitude() < 12)){
                    if (specimenIteration <= 4){
                        setPathState(PathStates.WAIT_BEFORE_GRAB);
                    } else {
                        follower.breakFollowing();
                    }
                }
                break;
            case WAIT_BEFORE_GRAB:
                if ((actionTimer.milliseconds() > 50 && follower.getVelocityMagnitude() < 4 && follower.atPoint(new Point(PathStates.GO_TO_GRAB.finalPose), 3.3, 3.3)) || actionTimer.milliseconds() > 1300){
                    follower.setMaxPower(1);
                    stateMachineController.grab();
                    setPathState(PathStates.WAIT_SPECIMEN_GRAB);
                }
                break;
            case WAIT_SPECIMEN_GRAB:
                if (actionTimer.milliseconds() > 210){
                    if (stateMachineController.getActionState() == StateMachineController.ActionStates.FREE){
                        follower.followPath(scorePaths[specimenIteration], true);
                        setPathState(PathStates.SCORE);
                    }
                }
                break;
            case SCORE:
                follower.setMaxPower(1);
                if (!follower.atPoint(new Point(follower.getClosestPose()), 8, 8))
                    follower.followPath(scorePaths[specimenIteration]);
                if ((follower.atPoint(new Point(PathStates.SCORE.finalPose), 4, 4))
                  || (follower.atPoint(new Point(PathStates.SCORE.finalPose), 7, 7) && follower.getVelocityMagnitude() < 12)){
                    if (specimenIteration == 0){
                        if (follower.getVelocityMagnitude() > 14)
                            break;
                        setPathState(PathStates.SEARCH);
                        intakeController.setState(IntakeController.IntakeArmStates.CAMERA);
                    } else {
                        setPathState(PathStates.GO_TO_GRAB);
                        follower.followPath(grabPaths[specimenIteration + 5], true);
                    }
                    stateMachineController.release();
                    specimenIteration++;
                }
                break;
            case SEARCH:
                if (actionTimer.milliseconds() > 1250){
                    llController.find();
                    differencePose = llController.getDifference();
                    if (differencePose.getX() < 0)
                        detectionPose = new Pose(follower.getPose().getX() + differencePose.getX(), llController.getTarget().getY(), 0);
                    else {
                        detectionPose = new Pose(follower.getPose().getX(), llController.getTarget().getY(), 0);
                        ExtendoController.setFinalTargetPosition(32 * differencePose.getX(), true);
                    }
//                    if (differencePose.getY() > 9.7){
//                        detectionPose = follower.getPose();
//                        detectionPose.setHeading(Math.toRadians(llController.getYAngle()));
//                        follower.turn(llController.getYAngle() - follower.getPose().getHeading(), true);
//                        ExtendoController.setFinalTargetPosition(32 * (differencePose.getVector().getMagnitude()) - LLController.clawForwardOffset, true);
//                    } else {
                        samplePose = llController.getSample();
                        PathChain toDetection = follower.pathBuilder().addPath(new Path(new BezierCurve(
                                new Point(follower.getPose()),
                                new Point(detectionPose)
                        ))).build();
                        toDetection.getPath(0).setConstantHeadingInterpolation(0);
                        follower.followPath(toDetection, true);
//                    }
                    detectionAngle = llController.getAngle();
                    if (detectionAngle != 0) {
                        intakeController.setTwist(IntakeController.TwisterStates.RIGHT_FULL);
                    } else {
                        intakeController.setTwist(IntakeController.TwisterStates.STRAIGHT);
                    }
//                    follower.setMaxPower(0.6);
                    if (llController.noDetections()){
                        setPathState(PathStates.BRING_SAMPLE);
                        depositController.setState(DepositController.DepositArmStates.BASKET);
                        Path bringSamplePath = new Path(new BezierCurve(
                                new Point(follower.getPose()),
                                new Point(follower.getPose().getX() - 24, follower.getPose().getY()),
                                new Point(PathStates.BRING_SAMPLE.finalPose)
                        ));
                        bringSamplePath.setConstantHeadingInterpolation(0);
                        follower.setMaxPower(1);
                        follower.followPath(bringSamplePath, true);
                    }
                    else{
                        intakeController.setState(IntakeController.IntakeArmStates.FLOOR_AIM);
                        setPathState(PathStates.GO_TO_DETECTION);
                    }
//                    LiftController.setFinalTargetPosition(200);
                }
                break;
            case GO_TO_DETECTION:
                depositController.setState(DepositController.DepositArmStates.TRANSFER_READY);
                if ((((follower.atPoint(new Point(detectionPose), 1, 1)) && (Math.abs(follower.getPose().getHeading() - detectionPose.getHeading()) < Math.PI / 72.0 || Math.abs(follower.getPose().getHeading() - detectionPose.getHeading()) > 143.0 * 2 * Math.PI / 144.0) && follower.getVelocityMagnitude() < 9) || !follower.isBusy()) && extendoController.atTarget(50)){
//                if (!follower.isBusy() && !extendoController.isBusy()){
                    setPathState(PathStates.WAIT_SAMPLE_GRAB);
                    stateMachineController.grab();
                }
                break;
            case WAIT_SAMPLE_GRAB:
                if (actionTimer.milliseconds() > 500){
                    Path bringSamplePath = new Path(new BezierCurve(
                            new Point(follower.getPose()),
                            new Point(follower.getPose().getX() - 24, follower.getPose().getY()),
                            new Point(PathStates.BRING_SAMPLE.finalPose)
                    ));
                    bringSamplePath.setConstantHeadingInterpolation(0);
                    follower.setMaxPower(1);
                    follower.followPath(bringSamplePath, true);
                    ExtendoController.setFinalTargetPosition(ExtendoController.bottomLimit);
                    setPathState(PathStates.BRING_SAMPLE);
                }
                break;
            case BRING_SAMPLE:
                if (actionTimer.milliseconds() > 400 && depositController.getCurrentState() == DepositController.DepositArmStates.TRANSFER_READY && stateMachineController.getActionState() == StateMachineController.ActionStates.FREE)
                    stateMachineController.transfer();
                if (follower.atPoint(new Point(PathStates.BRING_SAMPLE.finalPose), 17, 17)){
                    stateMachineController.release();
                    follower.followPath(grabPaths[1], true);
                    intakeController.setState(IntakeController.IntakeArmStates.START);
                    setPathState(PathStates.GO_TO_GRAB);
                }
                break;
            default:
                break;
        }
    }

    @Override
    public void stop(){
        llController.off();
    }
}
