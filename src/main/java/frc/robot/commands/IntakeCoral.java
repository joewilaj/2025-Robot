// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;


/** An example command that uses an example subsystem. */

public class IntakeCoral extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Intake intake;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeCoral(Intake intake,double speed) {
    this.intake = intake;
    addRequirements(intake);
  }


    // Called every time the scheduler runs while the command is scheduled.

  @Override
    public void execute() {
        intake.IntakeWithTimeout(IntakeConstants.CORAL_INTAKE_TIME); // Run the intake at 30% speed for 1 second
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }


}