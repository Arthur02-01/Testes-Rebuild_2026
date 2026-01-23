package frc.robot.commands.Angulador;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angulador;
import frc.robot.subsystems.Limelight;

public class AngularAuto extends Command {

    private final Angulador angulador;
    private final Limelight limelight;

    private enum Estado {
        INFERIOR,
        CENTRAL,
        SUPERIOR
    }

    private Estado estado = Estado.CENTRAL;
    private static final double HISTERESIS = 0.07;

    public AngularAuto(Angulador angulador, Limelight limelight) {
        this.angulador = angulador;
        this.limelight = limelight;
        addRequirements(angulador);
    }

    @Override
    public void execute() {

        if (!limelight.temAlvo()) return;

        double d = limelight.getDistanciaFiltrada();

        if (estado == Estado.CENTRAL && d > 2.30 + HISTERESIS)
            estado = Estado.SUPERIOR;
        else if (estado == Estado.SUPERIOR && d < 2.30 - HISTERESIS)
            estado = Estado.CENTRAL;
        else if (estado == Estado.INFERIOR && d > 1.55 + HISTERESIS)
            estado = Estado.CENTRAL;
        else if (estado == Estado.CENTRAL && d < 1.55 - HISTERESIS)
            estado = Estado.INFERIOR;

        switch (estado) {
            case SUPERIOR -> angulador.moverParaAngulo(Angulador.LIMITE_SUPERIOR);
            case CENTRAL  -> angulador.moverParaAngulo(Angulador.LIMITE_CENTRAL);
            case INFERIOR -> angulador.moverParaAngulo(Angulador.LIMITE_INFERIOR);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
