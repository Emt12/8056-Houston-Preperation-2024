package frc.robot.commands.CombinedCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.IntakeCommands.IntakeCmd;
import frc.robot.commands.IntakeCommands.MZ80Cmd;
import frc.robot.commands.ShooterCommands.IndexCmd;

public class Take extends ParallelRaceGroup {
    IntakeCmd intakeCmd;
    IndexCmd indexCmd;
    MZ80Cmd mz80Cmd;

    public Take(IntakeCmd intakeCmd, IndexCmd indexCmd, MZ80Cmd mz80Cmd){
        this.indexCmd = indexCmd;
        this.intakeCmd = intakeCmd;
        this.mz80Cmd = mz80Cmd;

        ParallelCommandGroup take = 
                new ParallelCommandGroup(
                        indexCmd,
                        intakeCmd
                    );

        addCommands(
            take,
            mz80Cmd
        );
    }
}
