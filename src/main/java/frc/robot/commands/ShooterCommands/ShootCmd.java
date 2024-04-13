package frc.robot.commands.ShooterCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

/**
 * This command runs the <b>shooter wheels</b>.
 * @param shooter {@link Shooter}, which is a subsystem that includes grabber.
 */
public class ShootCmd extends Command{

    private Shooter shooter;  

    public ShootCmd(Shooter shooter, Index index){
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        shooter.shoot(-1.0);
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.shoot(.0);
        super.end(interrupted);
    }
}
