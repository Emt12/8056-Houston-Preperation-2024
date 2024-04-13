package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class ResetRelative extends Command{
    private final Swerve swerve;

    public ResetRelative(Swerve swerve){
        this.swerve = swerve;
    }

    @Override
    public void execute() {
        swerve.resetRelative();
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0, false, false);
        super.end(interrupted);
    }
}
