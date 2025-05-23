package competition_op_modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import controllers.ColorController;

@Autonomous(group = "!basket")
public class RedBasketAuto extends YellowBasketAuto {
    @Override
    public ColorController.COLORS getAllianceColor(){
        return ColorController.COLORS.RED;
    }
}
