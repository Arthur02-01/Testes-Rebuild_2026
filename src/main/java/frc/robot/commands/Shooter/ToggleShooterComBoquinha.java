package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Index;
import frc.robot.Constantes.ConstantesShooter;
import frc.robot.Constantes.ConstantesIndex;

public class ToggleShooterComBoquinha extends Command {

    private enum Estado {
        DESLIGADO,
        LIGANDO_SHOOTER,
        LIGANDO_BOQUINHA,
        ATIVO,
        DESLIGANDO_BOQUINHA,
        DESLIGANDO_SHOOTER
    }

    private final Shooter shooter;
    private final Index boquinha;

    private Estado estado = Estado.DESLIGADO;
    private double tempoEstado;

    private static final double ATRASO_LIGAR_BOQUINHA_S = 1.0;
    private static final double ATRASO_DESLIGAR_SHOOTER_S = 1.0;

    public ToggleShooterComBoquinha(Shooter shooter, Index boquinha) {
        this.shooter = shooter;
        this.boquinha = boquinha;
        addRequirements(shooter, boquinha);
    }

    @Override
    public void initialize() {
        tempoEstado = Timer.getFPGATimestamp();

        if (estado == Estado.DESLIGADO) {
            estado = Estado.LIGANDO_SHOOTER;
        } else if (estado == Estado.ATIVO) {
            estado = Estado.DESLIGANDO_BOQUINHA;
        }
    }

    @Override
    public void execute() {
        double agora = Timer.getFPGATimestamp();

        switch (estado) {

            case LIGANDO_SHOOTER -> {
                shooter.setVelocidade(ConstantesShooter.Velocidade.NORMAL);
                shooter.atirarFrente();

                tempoEstado = agora;
                estado = Estado.LIGANDO_BOQUINHA;
            }

            case LIGANDO_BOQUINHA -> {
                if (agora - tempoEstado >= ATRASO_LIGAR_BOQUINHA_S) {
                    boquinha.setVelocidade(ConstantesIndex.VelocidadeIndex.NORMAL);
                    boquinha.ligarBoquinha(
                        ConstantesIndex.VelocidadeIndex.NORMAL.rpm
                    );
                    estado = Estado.ATIVO;
                }
            }

            case DESLIGANDO_BOQUINHA -> {
                boquinha.desligarBoquinha();
                tempoEstado = agora;
                estado = Estado.DESLIGANDO_SHOOTER;
            }

            case DESLIGANDO_SHOOTER -> {
                if (agora - tempoEstado >= ATRASO_DESLIGAR_SHOOTER_S) {
                    shooter.parar();
                    estado = Estado.DESLIGADO;
                }
            }

            default -> {}
        }
    }

    @Override
    public boolean isFinished() {
        return estado == Estado.DESLIGADO || estado == Estado.ATIVO;
    }
}
