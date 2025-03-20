// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PositionArm extends Command {

  private final Arm arm;
  private final double targetAngleDegrees;
  private final PIDController pidController;


  public PositionArm(Arm arm,double targetAngleDegrees) {
    this.arm = arm;
    this.targetAngleDegrees = targetAngleDegrees;
    addRequirements(arm);
    pidController = new PIDController(ArmConstants.kp,ArmConstants.ki,ArmConstants.kd);

    pidController.setTolerance(ArmConstants.kPositionToleranceDegrees);
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    pidController.reset();

    double p = SmartDashboard.getNumber("Arm/P Gain", ArmConstants.kp);
    double i = SmartDashboard.getNumber("Arm/I Gain", ArmConstants.ki);
    double d = SmartDashboard.getNumber("Arm/D Gain", ArmConstants.kd);
    pidController.setPID(p, i, d);
    
  }

  public void execute() {

    double currentAngle = arm.getAngle();
    double output = pidController.calculate(currentAngle, targetAngleDegrees);
    arm.setArmSpeed(output);
    SmartDashboard.putNumber("Arm/Current Angle", currentAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
