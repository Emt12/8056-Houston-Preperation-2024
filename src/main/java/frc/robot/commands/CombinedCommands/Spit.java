package frc.robot.commands.CombinedCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ShooterCommands.IndexCmd;
import frc.robot.commands.IntakeCommands.IntakeCmd;

public class Spit extends ParallelCommandGroup{
    IntakeCmd intakeGiveCmd;
    IndexCmd indexCmd;

    public Spit(IntakeCmd intakeGiveCmd, IndexCmd indexCmd){
        this.indexCmd = indexCmd;
        this.intakeGiveCmd = intakeGiveCmd;

        addCommands(
            indexCmd,
            intakeGiveCmd
        );
    }
}
