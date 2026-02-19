package frc.robot.commands.Index;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index;

public class RodarIndex extends Command {

    private final Index index;

    public RodarIndex(Index index) {
        this.index = index;
        addRequirements(index);
    }

    @Override
    public void initialize() {
        index.entrarModoForcado(0.3);
    }

    @Override
    public void end(boolean interrupted) {
        index.sairModoForcado();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
