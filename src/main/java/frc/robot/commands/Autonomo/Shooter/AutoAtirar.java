package frc.robot.commands.Autonomo.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constantes.ConstantesShooter;
import frc.robot.commands.Shooter.AtivarFrenteShooter;
import frc.robot.commands.Shooter.PararShooter;
import frc.robot.subsystems.Shooter;


public class AutoAtirar extends SequentialCommandGroup {

    public AutoAtirar(Shooter shooter) {

        
        addCommands(
            new PararShooter(shooter),
            new AtivarFrenteShooter(
            shooter,
            ConstantesShooter.Velocidade.TURBO
            ),
            new WaitCommand(10),
            new WaitCommand(0.5),
            new PararShooter(shooter)
        );

    }
}
