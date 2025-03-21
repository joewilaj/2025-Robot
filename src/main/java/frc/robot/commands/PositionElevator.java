// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

/** An example command that uses an example subsystem. */

public class PositionElevator extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Elevator elevator;
  //private final double height;
  private final Arm arm;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PositionElevator(Elevator elevator,Arm arm,double height) {
    this.elevator = elevator;
    //this.height = height;
    this.arm = arm;
    addRequirements(elevator);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}



    // Called every time the scheduler runs while the command is scheduled.

  @Override
    public void execute() {
        //elevator.setElevatorPosition(height);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopElevator();
  }


}