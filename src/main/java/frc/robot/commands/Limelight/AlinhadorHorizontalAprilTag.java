package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Traction;

public class AlinhadorHorizontalAprilTag extends Command {

    private final Limelight limelight;
    private final Traction traction;

    private static final double KP_ROT = 0.035;
    private static final double ANGULO_OK = 0.05;

    public AlinhadorHorizontalAprilTag(Limelight limelight, Traction traction) {
        this.limelight = limelight;
        this.traction = traction;
        addRequirements(traction);
    }

    @Override
    public void initialize() {
        limelight.setPipeline(0);
        limelight.ligarLED();
    }

    @Override
    public void execute() {

        if (!limelight.temAlvo()) {
            traction.stop();
            return;
        }

        double erroX = limelight.getTx();
        double rot = 0.0;

        if (Math.abs(erroX) > ANGULO_OK) {
            rot = erroX * KP_ROT;
        }

        rot = Math.max(Math.min(rot, 0.8), -0.8);
        traction.arcadeMode(0.0, rot);
    }

    @Override
    public void end(boolean interrupted) {
        traction.stop();
        limelight.desligarLED();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
