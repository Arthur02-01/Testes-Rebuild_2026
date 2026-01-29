package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberStep extends Command {

    private final Climber climber;
    private final double step;

    public ClimberStep(Climber climber, double step) {
        this.climber = climber;
        this.step = step;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        if (step > 0) {
            climber.subirStep(step);
        } else {
            climber.descerStep(Math.abs(step));
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}