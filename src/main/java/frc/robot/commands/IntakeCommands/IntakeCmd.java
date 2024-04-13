package frc.robot.commands.IntakeCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCmd extends Command{
    Intake intake;
    double speed;
    /**
     * Intake Command
     * @param intake
     * @param speed (+) Give ---- (-) Take 
     */
    public IntakeCmd(Intake intake, double speed){
        this.intake = intake;
        this.speed = speed;
    }

    @Override
    public void execute() {
        intake.setRoller(speed);
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setRoller(0);
        super.end(interrupted);
    }
}
