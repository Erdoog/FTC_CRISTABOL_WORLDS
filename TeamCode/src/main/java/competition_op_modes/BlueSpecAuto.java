package competition_op_modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import controllers.ColorController;

@Autonomous(group = "!spec")
public class BlueSpecAuto extends SpecAuto {
    @Override
    public ColorController.COLORS getAllianceColor(){
        return ColorController.COLORS.BLUE;
    }
}
