package frc.robot.commands.ShooterCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;


/**
 * This command runs the <b>Index wheel</b>.
 * @param index {@link Index}, which is a subsystem that includes grabber.
 * @param speed speed in double
 */
public class IndexCmd extends Command {
    private final Index index;
    double speed;
    
    public IndexCmd(Index index, double speed){
        this.index = index;
        this.speed = speed;

    }

    @Override
    public void execute() {
        index.setIndex(speed);
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        index.setIndex(0.0);
        super.end(interrupted);
    }
}
