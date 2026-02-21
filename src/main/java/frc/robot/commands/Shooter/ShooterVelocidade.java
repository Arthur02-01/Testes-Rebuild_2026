package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constantes.ConstantesShooter;
import frc.robot.subsystems.Shooter;

public class ShooterVelocidade extends InstantCommand {

    public ShooterVelocidade(
        Shooter shooter,
        ConstantesShooter.Velocidade alvo
    ) {
        super(() -> shooter.setVelocidade(alvo), shooter);
    }
}