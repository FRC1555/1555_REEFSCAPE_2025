package frc.robot.subsystems;

import frc.robot.Constants.ClimbSubsystemConstants;
import frc.robot.Constants.ClimbSubsystemConstants.ClimbSetpoints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbSubsystem extends SubsystemBase {

    private double climbCurrentTarget = ClimbSetpoints.kNeutral;

    public enum Setpoint {
        kNeutral,
        kForward,
        kBackward,
    }   
    
    public Command setSetpointCommand(Setpoint setpoint) {
        return this.runOnce (
            () -> {
                switch (setpoint) {
                    case kNeutral:
                        climbCurrentTarget = ClimbSetpoints.kNeutral;
                        break;
                    case kForward:
                        climbCurrentTarget = ClimbSetpoints.kForward;  
                        break;
                    case kBackward:
                        climbCurrentTarget = ClimbSetpoints.kBackward;
                        break;
                }
            }
        );
    }
}





