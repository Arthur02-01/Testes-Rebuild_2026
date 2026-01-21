package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Traction;

public class AlinhadorVerticalAprilTag extends Command {

    private final Limelight limelight;
    private final Traction traction;

    private static final double KP_DIST = 0.45;
    private static final double DIST_DESEJADA = 1.25;
    private static final double DIST_OK = 0.05;

    public AlinhadorVerticalAprilTag(Limelight limelight, Traction traction) {
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

        double distanciaAtual = limelight.getDistanciaAprilTag();
        double erro = distanciaAtual - DIST_DESEJADA;

        double avanco = 0.0;

        if (Math.abs(erro) > DIST_OK) {
            avanco = erro * KP_DIST;
        }

        avanco = Math.max(Math.min(avanco, 0.7), -0.7);
        traction.arcadeMode(avanco, 0.0);
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
