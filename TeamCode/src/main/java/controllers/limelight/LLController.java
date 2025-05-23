package controllers.limelight;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

@Config
public class LLController {
    // Limelight and claw configuration
    public static double limelightHeight = 11.4; // Camera height in inches
    public static double limelightAngle = 55; // Camera angle (0° = down, 90° = forward)
    public static double clawForwardOffset = 6.3; // Claw's forward offset from the camera
//    public static double clawLateralOffset = 0; // Claw's lateral (right is +) offset from the camera
    public static double clawLateralOffset = -3.14; // Claw's lateral (right is +) offset from the camera

    private Pose sample = new Pose(), difference = new Pose(), target = new Pose(); // The best sample's position
    private Pose sampleRelativeToDrivetrainCenter = new Pose(); // The best sample's position
    private Pose cachedTarget = new Pose(); // Cached best sample
    private Limelight3A limelight;
    private PathChain toTarget;
    private LLResult result;
    private Telemetry telemetry;
    private int[] unwanted;
    private double bestAngle;
    private double score;
    private Follower f;
    private boolean spec;
    private boolean noDetecions = true;

    public LLController(HardwareMap hardwareMap, Telemetry telemetry, int[] unwanted, Follower f, boolean spec) {
        this.spec = spec;
        this.unwanted = unwanted;
        this.telemetry = telemetry;
        this.f = f;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(1);
        limelight.start();
        f.update();
        cachedTarget = f.getPose();
        f.update();
    }

    double robotYAngle = 0;

    public void find() {
        result = limelight.getLatestResult();
        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

        if (detections.isEmpty()) {
            telemetry.addData("Detections", "None");
            target = cachedTarget.copy();
            return;
        }

        // List to store scored detections
        List<LL3ADetection> scoredDetections = new ArrayList<>();

        for (LLResultTypes.DetectorResult detection : detections) {
            int c = detection.getClassId();

            boolean colorMatch = true;

            for (int o : unwanted) {
                if (c == o) {
                    colorMatch = false;
                    break;
                }
            }

            if (colorMatch) {
                // Compute angles
                double xAngle = Math.toRadians(detection.getTargetYDegreesNoCrosshair());
                double yAngle = Math.toRadians(detection.getTargetXDegreesNoCrosshair());

                telemetry.addData("X T Angle", detection.getTargetYDegrees());
                telemetry.addData("Y T Angle", detection.getTargetXDegrees());

                // Compute distances
//                double xDistance = (((limelightHeight * 2) * Math.sin(xAngle)) / Math.sin(Math.toRadians(90 + limelightAngle) - xAngle));
//                double yDistance = Math.tan(yAngle) * xDistance;
                double xDistance = ((limelightHeight * Math.sin(xAngle + Math.toRadians(limelightAngle))) / Math.sin(Math.toRadians(90 - limelightAngle) - xAngle)) * 0.94;
                double yDistance = limelightHeight / Math.sin(Math.toRadians(90 - limelightAngle) - xAngle) * Math.tan(yAngle) * 1.3;
                if (xDistance > 13)
                    xDistance *= 1.08;
                if (Math.abs(xDistance) > 22){
                    xDistance += 1.5;
                    yDistance /= 1.15;
                }
                if (yDistance > -2.5)
                    yDistance -= 1.6;
//                if (yDistance > -2.5)
//                    yDistance -= 1.6;

                if ((yDistance < (spec ? -7.5 : -8)) || Math.abs(xDistance) > 26 || yDistance > 3.4)
//                if (Math.abs(xDistance) > 26 || yDistance > 3.4)
                    continue;

                if (xDistance < clawForwardOffset - 1)
                    continue;

                telemetry.addData("X Distance", xDistance);
                telemetry.addData("Y Distance", yDistance);
                telemetry.addData("classID", detection.getClassId());
                telemetry.addLine("=============================");

                // Score based on alignment
                double rotationScoreVertical = -Math.abs((detection.getTargetCorners().get(0).get(0) -
                        detection.getTargetCorners().get(1).get(0)) /
                        (detection.getTargetCorners().get(1).get(1) -
                                detection.getTargetCorners().get(2).get(1)) - (1.5 / 3.5));
                score = (yDistance - 2) - Math.abs(xDistance - clawForwardOffset) + 8 * rotationScoreVertical; // Weighted scoring
//                if (rotationScoreVertical > -0.65)
                scoredDetections.add(new LL3ADetection(detection, score, yDistance, xDistance, rotationScoreVertical, 0));
                double rotationScoreHorizontal = -Math.abs((detection.getTargetCorners().get(0).get(0) -
                        detection.getTargetCorners().get(1).get(0)) /
                        (detection.getTargetCorners().get(1).get(1) -
                                detection.getTargetCorners().get(2).get(1)) - (3.5 / 1.5));
                score = (yDistance - 2) - Math.abs(xDistance - clawForwardOffset) + 4 * rotationScoreHorizontal; // Weighted scoring
                if (Math.abs(xDistance) < 3)
                    xDistance *= 0.8;
//                if (Math.abs(xDistance) > 22)
//                    xDistance -= 1.5;
                if (xDistance > 13)
                    xDistance /= 1.15;
//                if (xDistance < 15 && xDistance > 7.3)
//                    xDistance += 2;
//                if (rotationScoreHorizontal > -0.72)
                scoredDetections.add(new LL3ADetection(detection, score, yDistance, xDistance, rotationScoreHorizontal, 90));
//
//                double angle = 0;
//
//                if (detection.getTargetCorners() == null || detection.getTargetCorners().size() < 4) {
//                    angle = Double.NaN;
//                }
//
//                List<List<Double>> corners = detection.getTargetCorners();
//
//                double dx = Math.toRadians(corners.get(1).get(0) - corners.get(0).get(0));
//                double dy = Math.toRadians(corners.get(2).get(1) - corners.get(0).get(1));
//                angle = ((Math.atan2(dy, dx)) * 4.5);

            }
        }

        // Find the best detection

        if (!scoredDetections.isEmpty()) {
            scoredDetections.sort(Comparator.comparingDouble(LL3ADetection::getScore).reversed());
            LL3ADetection bestDetection = scoredDetections.get(0);

            score = bestDetection.getScore();
            bestAngle = bestDetection.getAngle();

            // Convert to coordinates and apply claw offsets
            sample = new Pose(
                    bestDetection.getXDistance(),
                    bestDetection.getYDistance(),
                    0
            );

            difference = new Pose(sample.getX() - clawForwardOffset, -(sample.getY() + clawLateralOffset), 0);
            sampleRelativeToDrivetrainCenter = new Pose(difference.getX() + 7.7, difference.getY());
            robotYAngle = (90 - Math.atan2(sampleRelativeToDrivetrainCenter.getX(), sampleRelativeToDrivetrainCenter.getY()) / Math.PI * 180);

            target = new Pose(f.getPose().getX() + difference.getX(), f.getPose().getY() + difference.getY(), f.getPose().getHeading());

            cachedTarget = target.copy();

            toTarget = new PathBuilder()
                    .addPath(new BezierLine(f.getPose(), target)).setConstantHeadingInterpolation(f.getPose().getHeading()).build();

            telemetry.addData("angle", robotYAngle);
            telemetry.addData("Best Detection", bestDetection.getDetection().getClassName());
            telemetry.addData("Sample Position", "X: %.2f, Y: %.2f", sample.getX(), sample.getY());
            telemetry.addData("diff", difference);
            telemetry.addData("target", target);
            telemetry.addData("current", f.getPose());
            noDetecions = false;
        } else {
            noDetecions = true;
            target = cachedTarget.copy();
        }
    }

    public boolean noDetections(){
        return noDetecions;
    }
    public Pose getTarget() {
        return target;
    }
    public Pose getSample() {
        return sample;
    }
    public Pose getDifference() {
        return difference;
    }

    public PathChain toTarget() {
        toTarget = new PathBuilder()
                .addPath(new BezierLine(f.getPose(), target))//new Pose(target, target.getY())))
                .setConstantHeadingInterpolation(f.getPose().getHeading())
                .build();
        return toTarget;
    }

    public double getYAngle() {return robotYAngle;}

    public void off() {
        limelight.stop();
    }

    public void on() {
        limelight.start();
    }

    public double getAngle() {return bestAngle;}

    public double getScore() {return score;}
}