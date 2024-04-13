package frc.robot.commands.CombinedCommands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.IntakeCommands.IntakeCmd;
import frc.robot.commands.ShooterCommands.IndexCmd;
import frc.robot.commands.ShooterCommands.ShootCmd;

public class Shoot extends ParallelCommandGroup{
    IndexCmd indexCmdIn;
    IndexCmd indexCmdOut;
    ShootCmd shootCmd;
    IntakeCmd intakeCmd;

    public Shoot(IndexCmd indexCmdIn,
                 IndexCmd indexCmdOut,
                 ShootCmd shootCmd,
                 IntakeCmd intakeCmd){

        this.indexCmdIn = indexCmdIn;
        this.indexCmdOut = indexCmdOut;
        this.shootCmd = shootCmd;
        this.intakeCmd = intakeCmd;

        SequentialCommandGroup indexSequal 
            = new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        indexCmdOut,
                        new WaitCommand(.3)
                    ),

                    new WaitCommand(.5),

                    new ParallelCommandGroup(
                        indexCmdIn,
                        intakeCmd
                    )
            );
        
        SequentialCommandGroup shootSequal
            = new SequentialCommandGroup(
                new WaitCommand(.5),
                shootCmd
            );
        
        addCommands(
            shootSequal,
            indexSequal
        );

    }
}
