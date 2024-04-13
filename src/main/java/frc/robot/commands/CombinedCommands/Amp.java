package frc.robot.commands.CombinedCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.ShooterCommands.IndexCmd;
import frc.robot.commands.ShooterCommands.WristPIDCmd;

public class Amp extends SequentialCommandGroup{

    WristPIDCmd wristUp;
    WristPIDCmd wristDown;
    IndexCmd indexBack;
    public Amp(IndexCmd indexBack, WristPIDCmd wristUp, WristPIDCmd wristDown){

        this.wristUp = wristUp;
        this.wristDown = wristDown;
        this.indexBack = indexBack;

        ParallelRaceGroup wristSetUp = 
        new ParallelRaceGroup(
            wristUp,
            new WaitCommand(2)
        );

        ParallelRaceGroup wristSetDown =
        new ParallelRaceGroup(
            wristDown,
            new WaitCommand(2)
        );

        ParallelRaceGroup indexSpit = 
        new ParallelRaceGroup(
            indexBack,
            new WaitCommand(2)
        );



        addCommands(
            wristSetUp,
            indexSpit,
            wristSetDown    
        );
    }
}
