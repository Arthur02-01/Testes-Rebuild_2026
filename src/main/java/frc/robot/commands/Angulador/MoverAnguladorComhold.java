package frc.robot.commands.Angulador;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angulador;

public class MoverAnguladorComhold extends Command {

    private final Angulador angulador;
    private final double alvo;

    public MoverAnguladorComhold(Angulador angulador, double alvo) {
        this.angulador = angulador;
        this.alvo = alvo;
        addRequirements(angulador);
    }

    @Override
    public void initialize() {
        angulador.moverParaAngulo(alvo);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(angulador.getAngulo() - alvo)
               <= Angulador.MargenErro;
    }

    @Override
    public void end(boolean interrupted) {
        angulador.moverParaAngulo(alvo);
    }
}
