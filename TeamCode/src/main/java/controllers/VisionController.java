package controllers;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
//import org.firstinspires.ftc.vision.opencv.ColorRange;
//import org.firstinspires.ftc.vision.opencv.ColorSpace;
//import org.firstinspires.ftc.vision.opencv.ImageRegion;
import vision.ColorBlobLocatorProcessor;
import vision.ColorRange;
import vision.ColorSpace;
import vision.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.List;

@Config
public class VisionController {
    ColorBlobLocatorProcessor colorLocatorYellow, colorLocatorAllianceSpecific;
    Size resolution = new Size(320, 240);
    VisionPortal portal;
    Telemetry dashboardTelemetry;

    public VisionController(HardwareMap hardwareMap, ColorController.COLORS alliance){
        colorLocatorYellow = new ColorBlobLocatorProcessor.Builder()
//                .setTargetColorRange(new ColorRange(ColorSpace.YCrCb, new Scalar(100, 100, 0), new Scalar(255, 210, 70)))
                .setTargetColorRange(new ColorRange(ColorSpace.HSV, new Scalar(14, 65, 100), new Scalar(37, 255, 255)))
//                .setTargetColorRange(new ColorRange(ColorSpace.HSV, new Scalar(0, 65, 100), new Scalar(0, 255, 255)))
                .setErodeSize(15)
//                .setDilateSize(6)
//                .setBlurSize(4)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        if (alliance == ColorController.COLORS.RED){
            colorLocatorAllianceSpecific = new ColorBlobLocatorProcessor.Builder()
    //                .setTargetColorRange(new ColorRange(ColorSpace.YCrCb, new Scalar(100, 100, 0), new Scalar(255, 210, 70)))
                    .setTargetColorRange(new ColorRange(ColorSpace.HSV, new Scalar(148, 85, 90), new Scalar(11, 255, 255)))
                    .setErodeSize(17)
    //                .setDilateSize(6)
    //                .setBlurSize(4)
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                    .setRoi(ImageRegion.entireFrame())
                    .setDrawContours(true)
                    .setBlurSize(5)
                    .build();
        } else if (alliance == ColorController.COLORS.BLUE) {
            colorLocatorAllianceSpecific = new ColorBlobLocatorProcessor.Builder()
                    //                .setTargetColorRange(new ColorRange(ColorSpace.YCrCb, new Scalar(100, 100, 0), new Scalar(255, 210, 70)))
                    .setTargetColorRange(new ColorRange(ColorSpace.HSV, new Scalar(92, 85, 110), new Scalar(140, 255, 255)))
                    .setErodeSize(15)
                    //                .setDilateSize(6)
                    //                .setBlurSize(4)
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                    .setRoi(ImageRegion.entireFrame())
                    .setDrawContours(true)
                    .setBlurSize(5)
                    .build();
        } else {
            throw new RuntimeException();
        }

        portal = new VisionPortal.Builder()
                .addProcessor(colorLocatorYellow)
                .addProcessor(colorLocatorAllianceSpecific)
                .setCameraResolution(resolution)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.setMsTransmissionInterval(50);
        dashboardTelemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        dashboard.startCameraStream(portal, 60);
    }

    public class Location{
        public double x, y, angle;
        public Location(double x, double y, double angle){
            this.x = x;
            this.y = y;
            this.angle = angle;
        }
    }

    public Location getLargestLocation(boolean smallerArea){
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocatorYellow.getBlobs();
        List<ColorBlobLocatorProcessor.Blob> blobsAllianceSpecific = colorLocatorAllianceSpecific.getBlobs();
        blobs.addAll(blobsAllianceSpecific);

        if (smallerArea)
            ColorBlobLocatorProcessor.Util.filterByArea(3800, 12000, blobs);
        else
            ColorBlobLocatorProcessor.Util.filterByArea(4500, 12000, blobs);
        double maxArea = 0;

        double x = Double.NaN, y = Double.NaN, angle = Double.NaN;

        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            double blobaArea = b.getContourArea();

            RotatedRect boxFit = b.getBoxFit();

            Point[] pt = new Point[4];
            boxFit.points(pt);

            Point edge1 = new Point(pt[1].x - pt[0].x, pt[1].y - pt[0].y);
            Point edge2 = new Point(pt[2].x - pt[1].x, pt[2].y - pt[1].y);

            Point usedEdge = edge1;
            if (norm(edge1) < norm(edge2)) usedEdge = edge2;

            double height = Math.max(norm(edge1), norm(edge2));
            double width  = Math.min(norm(edge1), norm(edge2));

            if (width > 100 && height / width < 1.27)
                continue;

            double area = height * width;
            if (area < maxArea)
                continue;
            maxArea = area;

            Point reference = new Point(1, 0);

//            dashboardTelemetry.addLine(String.format("Angle %3d", (int)angle));
//            dashboardTelemetry.addData("blob area", blobaArea);
//            dashboardTelemetry.addData("width", height);
//            dashboardTelemetry.addData("height", width);
//                dashboardTelemetry.addLine(String.format("Distance (%3d %3d)", boxFit.center.x - resolution.getWidth() / 2, boxFit.center.y - resolution.getHeight() / 2));
//            angle = Math.toDegrees(Math.acos(reference.dot(usedEdge) / norm(usedEdge)));
            angle = Math.toDegrees(Math.atan2(usedEdge.y, usedEdge.x)) + 90;
            x = boxFit.center.x - (double) resolution.getWidth() / 2.0;
            y = boxFit.center.y - (double) resolution.getHeight() / 2.0;
        }

//        dashboardTelemetry.update();
        return new Location(x, y, angle);
    }

    public void pause(){
        portal.stopStreaming();
    }

    public void resume(){ portal.resumeStreaming(); }

    private double norm(Point point) {
        return Math.sqrt(point.dot(point));
    }
}
