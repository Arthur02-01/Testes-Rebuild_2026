package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.Constantes.ConstantesShooter;;

public class AtivarAtrasShooter extends Command {

    private final Shooter shooter;
    private final ConstantesShooter.Velocidade velocidade;

    public AtivarAtrasShooter(
        Shooter shooter,
        ConstantesShooter.Velocidade velocidade
    ) {
        this.shooter = shooter;
        this.velocidade = velocidade;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.atirarTras(velocidade);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
