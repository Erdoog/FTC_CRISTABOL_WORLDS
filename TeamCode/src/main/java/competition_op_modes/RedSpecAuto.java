package competition_op_modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import controllers.ColorController;

@Autonomous(group = "!spec")
public class RedSpecAuto extends SpecAuto {
    @Override
    public ColorController.COLORS getAllianceColor(){
        return ColorController.COLORS.RED;
    }
}
