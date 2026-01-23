// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.VisionSubsystem;
// import frc.robot.subsystems.DriveSubsystem;

// public class AlignToTargetCommand extends CommandBase {
//     private final VisionSubsystem visionSubsystem;
//     private final DriveSubsystem driveSubsystem;

//     public AlignToTargetCommand(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
//         this.visionSubsystem = visionSubsystem;
//         this.driveSubsystem = driveSubsystem;
//         addRequirements(visionSubsystem, driveSubsystem);
//     }

//     @Override
//     public void execute() {
//         if (visionSubsystem.hasTarget()) {
//             double yaw = visionSubsystem.getTargetYaw();
//             // Example: Use yaw to turn the robot
//             driveSubsystem.arcadeDrive(0, -yaw * 0.02); // Adjust gain as needed
//         } else {
//             driveSubsystem.arcadeDrive(0, 0); // Stop if no target
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         return false; // Run until interrupted
//     }
// }