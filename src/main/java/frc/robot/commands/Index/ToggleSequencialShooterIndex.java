package frc.robot.commands.Index;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;
import frc.robot.Constantes.ConstantesIndex;

public class ToggleSequencialShooterIndex extends Command {

    private enum Estado {
        LIGAR_SHOOTER,
        ESPERAR_SHOOTER,
        LIGAR_BOQUINHA,
        ESPERAR_BOQUINHA,
        LIGAR_INDEX,
        FINAL
    }

    private final Shooter shooter;
    private final Index index;

    private final Timer timer = new Timer();
    private Estado estado;

    public ToggleSequencialShooterIndex(Shooter shooter, Index index) {
        this.shooter = shooter;
        this.index = index;
        addRequirements(shooter, index);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        index.setVelocidade(ConstantesIndex.VelocidadeIndex.NORMAL);
        estado = Estado.LIGAR_SHOOTER;
    }

    @Override
    public void execute() {

        switch (estado) {

            case LIGAR_SHOOTER -> {
                shooter.atirarFrente();
                timer.reset();
                estado = Estado.ESPERAR_SHOOTER;
            }

            case ESPERAR_SHOOTER -> {
                if (timer.hasElapsed(1.0)) {
                    estado = Estado.LIGAR_BOQUINHA;
                }
            }

            case LIGAR_BOQUINHA -> {
                index.ligar(); 
                timer.reset();
                estado = Estado.ESPERAR_BOQUINHA;
            }

            case ESPERAR_BOQUINHA -> {
                if (timer.hasElapsed(0.5)) {
                    estado = Estado.LIGAR_INDEX;
                }
            }

            case LIGAR_INDEX -> {
                shooter.setAlimentando(true);
                index.entrarModoForcado(0.4);
                estado = Estado.FINAL;
            }

            default -> {}
        }
    }

    @Override
    public boolean isFinished() {
        return estado == Estado.FINAL;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
}