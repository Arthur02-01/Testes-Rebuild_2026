package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Index;
import frc.robot.Constantes.ConstantesShooter;
import frc.robot.Constantes.ConstantesIndex;

public class DesativarShooterComBoquinha extends Command {

    private final Shooter shooter;
    private final Index boquinha;

    private double tempoInicio;
    private static final double ATRASO_SHOOTER_S = 1.0;

    public DesativarShooterComBoquinha(Shooter shooter, Index boquinha) {
        this.shooter = shooter;
        this.boquinha = boquinha;
        addRequirements(shooter, boquinha);
    }

    @Override
    public void initialize() {
        boquinha.desligarBoquinha();
        tempoInicio = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (Timer.getFPGATimestamp() - tempoInicio >= ATRASO_SHOOTER_S) {
            shooter.parar();
        }
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - tempoInicio >= ATRASO_SHOOTER_S;
    }
}

