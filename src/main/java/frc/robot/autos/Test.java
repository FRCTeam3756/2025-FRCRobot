// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.autos;

// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.units.Units;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.constants.SwerveConstants;
// import frc.robot.subsystems.CommandSwerveDrivetrain;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class Test extends Command {
//   /** Creates a new Test. */
//   CommandSwerveDrivetrain drivetrain;
//   private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
//       .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

//   double time;

//   public Test(CommandSwerveDrivetrain drivetrain) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     drivetrain = this.drivetrain;
//     addRequirements(drivetrain);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     time = Timer.getFPGATimestamp();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     drivetrain.applyRequest(() ->
//                 drive.withVelocityX(-0.5 * SwerveConstants.SPEED_AT_12_VOLTS.in(Units.MetersPerSecond))
//                 .withVelocityY(0)
//                 .withRotationalRate(0)
//             );
//             System.out.println("running");
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     System.out.println("done");
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return Timer.getFPGATimestamp() - time > 5;
//   }
// }
