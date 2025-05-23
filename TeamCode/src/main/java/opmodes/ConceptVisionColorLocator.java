package opmodes;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.List;

import controllers.IntakeController;

@TeleOp(name = "Concept: Vision Color-Locator", group = "Concept")
public class ConceptVisionColorLocator extends LinearOpMode {

    private double norm(Point point) {
        return Math.sqrt(point.dot(point));
    }

    @Override
    public void runOpMode() {
        IntakeController intakeController = new IntakeController(hardwareMap);
        intakeController.setState(IntakeController.IntakeArmStates.CAMERA);
        intakeController.update();

        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
//                .setTargetColorRange(new ColorRange(ColorSpace.YCrCb, new Scalar(100, 100, 0), new Scalar(255, 210, 70)))
                .setTargetColorRange(new ColorRange(ColorSpace.HSV, new Scalar(15, 65, 85), new Scalar(45, 255, 255)))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        Size resolution = new Size(320, 240);
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(resolution)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.setMsTransmissionInterval(50);
        dashboardTelemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        dashboard.startCameraStream(portal, 60);

        while (opModeIsActive() || opModeInInit()) {
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            ColorBlobLocatorProcessor.Util.filterByArea(10000, 200000, blobs);
//            ColorBlobLocatorProcessor.Util.filterByArea(10000, 20000, blobs);

            for (ColorBlobLocatorProcessor.Blob b : blobs) {
                RotatedRect boxFit = b.getBoxFit();

                Point[] pt = new Point[4];
                boxFit.points(pt);

                Point edge1 = new Point(pt[1].x - pt[0].x, pt[1].y - pt[0].y);
                Point edge2 = new Point(pt[2].x - pt[1].x, pt[2].y - pt[1].y);

                Point usedEdge = edge1;
                if (norm(edge1) < norm(edge2)) usedEdge = edge2;

                double height = Math.max(norm(edge1), norm(edge2));
                double width  = Math.min(norm(edge1), norm(edge2));

                if (height > 250 && width > 170 && height / width < 1.5)
                    continue;

                Point reference = new Point(1, 0);
                double angle = Math.toDegrees(Math.acos(reference.dot(usedEdge) / norm(usedEdge)));

                dashboardTelemetry.addLine(String.format("Angle %3d", (int)angle));
                dashboardTelemetry.addData("width", height);
                dashboardTelemetry.addData("height", width);
//                dashboardTelemetry.addLine(String.format("Distance (%3d %3d)", boxFit.center.x - resolution.getWidth() / 2, boxFit.center.y - resolution.getHeight() / 2));
            }

            dashboardTelemetry.update();
        }
    }
}
