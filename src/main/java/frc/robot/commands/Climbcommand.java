package frc.robot.commands;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem.CSetpoint;
import frc.robot.subsystems.AlgaeSubsystem;

public class Climbcommand {
    private final ClimbSubsystem m_climbSubsystem;
    private final AlgaeSubsystem m_algaeSubsystem;

    public Climbcommand(ClimbSubsystem climbSubsystem, AlgaeSubsystem algaeSubsystem) {
        this.m_climbSubsystem = climbSubsystem;
        this.m_algaeSubsystem = algaeSubsystem;
    }

    public void execute(int pov) {
        // Call the pushOutIntake method from AlgaeSubsystem
        m_algaeSubsystem.pushIntakeOut();

        // Determine the setpoint based on the POV value
        CSetpoint setpoint = CSetpoint.kNeutral; // Default value
        switch (pov) {
            case 0: // POV up
                setpoint = CSetpoint.kBackward;
                break;
            case 90: // POV right
                setpoint = CSetpoint.kNeutral;
                break;
            case 180: // POV down
                setpoint = CSetpoint.kForward;
                break;
        }

        // Call the moveToSetpoint method with the determined setpoint
        m_climbSubsystem.setSetpointCommand(setpoint);
    }
}
