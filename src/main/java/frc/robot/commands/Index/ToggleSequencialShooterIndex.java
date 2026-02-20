package frc.robot.commands.Index;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Index;


public class ToggleSequencialShooterIndex extends Command {

    private final Shooter shooter;
    private final Index index;

    private final Timer timer = new Timer();
    private boolean desligando = false;

    public ToggleSequencialShooterIndex(Shooter shooter, Index index) {
        this.shooter = shooter;
        this.index = index;
        addRequirements(shooter, index);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        // TOGGLE
        if (shooter.estaAtivo()) {
            desligando = true;
        } else {
            desligando = false;
            shooter.atirarFrente(); // liga shooter
        }
    }

    @Override
    public void execute() {

        double t = timer.get();

        if (!desligando) {
            

            if (t >= 1.0) {
                index.ligar(); 
            }

            if (t >= 1.5) {
                index.entrarModoForcado(0.4); 
            }

        } else {
            

            if (t >= 0.0) {
                index.entrarModoForcado(0.4); 
            }

            if (t >= 0.5) {
                index.desligar(); 
            }

            if (t >= 1.0) {
                shooter.parar(); 
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
}