package competition_op_modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import controllers.ColorController;

@Autonomous(group = "!basket")
public class BlueBasketAuto extends YellowBasketAuto {
    @Override
    public ColorController.COLORS getAllianceColor(){
        return ColorController.COLORS.BLUE;
    }
}
