package frc.robot.commands.Autonomo.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.IntakeFloor;
import frc.robot.commands.Pivot.MoverPivotPreset;
import frc.robot.Extras.AngulosPresetPivot;

public class AutoIntakeFloor extends SequentialCommandGroup {

    public AutoIntakeFloor(IntakeFloor intake) {

        addCommands(

            // 1) Abaixa o intake até o preset de coleta
            new MoverPivotPreset(intake, AngulosPresetPivot.ALTO),

            // 2) Liga o intake
            new InstantCommand(intake::IntakeOn, intake),

            // 3) Tempo coletando a peça
            new WaitCommand(1.5),

            // 4) Para o intake
            new InstantCommand(intake::PararIntake, intake),

            // 5) Sobe o intake para posição segura
            new MoverPivotPreset(intake, AngulosPresetPivot.BAIXO)
        );
    }
}