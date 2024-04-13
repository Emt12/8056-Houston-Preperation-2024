package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.custom.MZ80;

public class MZ80Cmd extends Command{
    MZ80 mz80;
    public MZ80Cmd(MZ80 mz80){
        this.mz80 = mz80;
    }

    @Override
    public boolean isFinished() {
        return !mz80.get();
    }
}
