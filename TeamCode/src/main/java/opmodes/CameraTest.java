package opmodes;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class CameraTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        Size resolution = new Size(320, 240);

        VisionPortal portal = new VisionPortal.Builder()
                .setCameraResolution(resolution)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("", portal.getActiveCamera().getSerialNumber());
            telemetry.addData("", portal.getActiveCamera().isAttached());
            telemetry.addData("", portal.getActiveCamera().getConnectionInfo());
            telemetry.update();
        }
    }
}
