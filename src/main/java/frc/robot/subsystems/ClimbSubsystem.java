package frc.robot.subsystems;

import frc.robot.Constants.AlgaeSubsystemConstants;
import frc.robot.Constants.ClimbSubsystemConstants;
import frc.robot.Constants.ClimbSubsystemConstants.ClimbSetpoints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;


public class ClimbSubsystem extends SubsystemBase {

    private SparkFlex climbMotor =
      new SparkFlex(ClimbSubsystemConstants.kClimbMotorCanId, MotorType.kBrushless);
    private SparkClosedLoopController climbController = climbMotor.getClosedLoopController();
    private RelativeEncoder climbEncoder = climbMotor.getEncoder();

    private double climbCurrentTarget = ClimbSetpoints.kNeutral;

    public enum CSetpoint {
        kNeutral,
        kForward,
        kBackward,
    }   
    
    public ClimbSubsystem() {
        // Constructor logic can go here if needed
        climbMotor.configure(
            Configs.ClimbSubsystem.climbConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        
    }
 
    private void moveToSetpoint() {
        climbController
            .setReference(climbCurrentTarget, ControlType.kMAXMotionPositionControl);
      }

    public Command setSetpointCommand(CSetpoint setpoint) {
        return this.runOnce(
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
                moveToSetpoint();
            }
        );
    }
}
