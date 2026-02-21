package frc.robot.commands.Index;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

public class PararSequencialIndexShooter extends Command {

    private enum Estado {
        PARAR_INDEX,
        ESPERAR_INDEX,
        PARAR_BOQUINHA,
        ESPERAR_BOQUINHA,
        PARAR_SHOOTER,
        FINAL
    }

    private final Shooter shooter;
    private final Index index;

    private final Timer timer = new Timer();
    private Estado estado;

    public PararSequencialIndexShooter(Shooter shooter, Index index) {
        this.shooter = shooter;
        this.index = index;
        addRequirements(shooter, index);
    }

    @Override
    public void initialize() {
        estado = Estado.PARAR_INDEX;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {

        switch (estado) {

            case PARAR_INDEX -> {
                index.sairModoForcado();
                timer.reset();
                estado = Estado.ESPERAR_INDEX;
            }

            case ESPERAR_INDEX -> {
                if (timer.hasElapsed(0.5)) {
                    estado = Estado.PARAR_BOQUINHA;
                }
            }

            case PARAR_BOQUINHA -> {
                index.desligarBoquinha();
                timer.reset();
                estado = Estado.ESPERAR_BOQUINHA;
            }

            case ESPERAR_BOQUINHA -> {
                if (timer.hasElapsed(0.5)) {
                    estado = Estado.PARAR_SHOOTER;
                }
            }

            case PARAR_SHOOTER -> {
                shooter.parar();
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
        shooter.parar();
        index.desligarBoquinha();
    }
}