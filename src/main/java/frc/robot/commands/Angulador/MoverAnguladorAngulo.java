package frc.robot.commands.Angulador;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angulador;

public class MoverAnguladorAngulo extends Command {

    private final Angulador angulador;
    private final double angulo;

    public MoverAnguladorAngulo(Angulador angulador, double angulo) {
        this.angulador = angulador;
        this.angulo = angulo;
        addRequirements(angulador);
    }

    @Override
    public void initialize() {
        angulador.moverParaAngulo(angulo);
    }

    @Override
    public boolean isFinished() {
        return angulador.emHold();
    }
}