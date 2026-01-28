package frc.robot.commands.Angulador;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angulador;
import frc.robot.subsystems.AnguloPreset;

public class MoverAnguladoPreset extends Command {

    private final Angulador angulador;
    private final AnguloPreset preset;

    public MoverAnguladoPreset(Angulador angulador, AnguloPreset preset) {
        this.angulador = angulador;
        this.preset = preset;
        addRequirements(angulador);
    }

    @Override
    public void initialize() {
        angulador.moverParaPreset(preset);
    }

    @Override
    public boolean isFinished() {
        return angulador.emHold();
    }

    @Override
    public void end(boolean interrupted) {
        // nada a fazer, o HOLD já mantém posição
    }
}