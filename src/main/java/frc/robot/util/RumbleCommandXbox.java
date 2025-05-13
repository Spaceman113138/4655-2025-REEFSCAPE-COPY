package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleCommandXbox extends CommandXboxController {

   /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
    public RumbleCommandXbox(int port) {
        super(port);
    }

    /**
     * Makes the controler rumble
     * 
     * @param value The strength of the rumble from 0-1
     S*/
    public void setRumble(double value) {
        getHID().setRumble(RumbleType.kBothRumble, value);
    }

}