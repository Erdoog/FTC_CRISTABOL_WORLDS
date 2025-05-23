package opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import controllers.ColorController;

@TeleOp
public class RedVisionTest extends VisionTest{
    public ColorController.COLORS getAllianceColor(){
        return ColorController.COLORS.RED;
    }
}
