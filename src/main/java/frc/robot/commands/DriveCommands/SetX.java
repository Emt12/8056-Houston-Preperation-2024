package frc.robot.commands.DriveCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;


/**
 * Sets wheels to X position.
 * @param Swerve {@link Swerve} for driving.
 */
public class SetX extends Command{
    private Swerve swerve;
    public SetX(Swerve swerve){
        this.swerve = swerve;
    }
    
    @Override
    public void execute() {
        swerve.setX();
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(0, 0, 0, false, false);
        super.end(interrupted);
    }
}
