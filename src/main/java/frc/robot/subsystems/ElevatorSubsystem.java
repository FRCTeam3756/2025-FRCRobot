package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;

public class ElevatorSubsystem extends SubsystemBase{
  private TalonSRX elevatorMotor = new TalonSRX(ClimbConstants.LEFT_CAN_ID);

  public void elevatorUp() {
    elevatorMotor.set(ControlMode.PercentOutput, ClimbConstants.CLIMB_SPEED);
  }

  public void elevatorDown() {
    elevatorMotor.set(ControlMode.PercentOutput, -ClimbConstants.CLIMB_SPEED);
  }

  public void elevatorStop() {
    elevatorMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {}
}
