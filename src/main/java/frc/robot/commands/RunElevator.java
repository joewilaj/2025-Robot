// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/** An example command that uses an example subsystem. */

public class RunElevator extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Elevator elevator;
  private final Arm arm;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunElevator(Elevator elevator,Arm arm) {
    this.elevator = elevator;
    this.arm = arm;

    addRequirements(elevator);
  }


    // Called every time the scheduler runs while the command is scheduled.

  @Override
    public void execute() {
      double speed = RobotContainer.xbox.getLeftY();
      elevator.setElevatorSpeed(speed);
      }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopElevator();
  }


}