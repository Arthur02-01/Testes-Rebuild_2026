package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Traction;

public class AlinhadorHorizontalAprilTag extends Command {

    private final Limelight limelight;
    private final Traction traction;

    private static final double KP_ROT = 0.035;

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

        double erroX = limelight.getTxFiltrado();

        if (Math.abs(erroX) < 0.03) {
            traction.stop();
            return;
        }

        double erroAbs = Math.abs(erroX);

        double ganhoDinamico =
            erroAbs > 5.0 ? 1.0 :
            erroAbs > 2.0 ? 0.5 :
            erroAbs > 1.0 ? 0.25 :
                            0.15;

        double rot = erroX * KP_ROT * ganhoDinamico;

        rot = Math.max(Math.min(rot, 0.45), -0.45);

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
