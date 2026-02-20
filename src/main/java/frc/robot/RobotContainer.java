package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Traction;
import frc.robot.subsystems.Angulador;
import frc.robot.subsystems.IntakeFloor;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Index;
import frc.robot.commands.Limelight.AlinhadorHorizontalAprilTag;
import frc.robot.commands.Limelight.AlinhadorVerticalAprilTag;
import frc.robot.commands.Pivot.MoverPivotPreset;
import frc.robot.commands.IntakeFloor.GirarIntake;
import frc.robot.commands.IntakeFloor.GirarIntakeReverse;
import frc.robot.commands.IntakeFloor.PararIntake;
import frc.robot.commands.Autonomo.Angulador.AnguladorAuto;
//import frc.robot.commands.Autonomo.LimelightAuto.AlinhadorHorizontalAuto;
//import frc.robot.commands.Autonomo.LimelightAuto.AlinhadorVerticalAuto;
import frc.robot.commands.Autonomo.Shooter.AutoAtirar;
import frc.robot.commands.Autonomo.Shooter.BoquinhaAntesShooterAuto;
import frc.robot.commands.Autonomo.Tracao.AndarEncoder;
import frc.robot.commands.Autonomo.Tracao.GiroPorAngulo;
import frc.robot.commands.Autonomo.intake.AutoIntakeFloor;
import frc.robot.commands.Autonomo.intake.Autobaixointake;
import frc.robot.commands.Index.Indexando;
import frc.robot.commands.Index.PararSequencialIndexShooter;
import frc.robot.commands.Index.RodarIndex;
import frc.robot.commands.Index.ToggleSequencialShooterIndex;
import frc.robot.CommandsRetirados.AtirarComBoquinhaAtrasado;
//import frc.robot.commands.Autonomo.Tracao.GiroPorAngulo;
import frc.robot.Constantes.ConstantesShooter;
import frc.robot.Extras.AnguloPreset;
import frc.robot.Extras.AngulosPresetPivot;
import frc.robot.commands.Angulador.MoverAnguladoPreset;
import frc.robot.commands.Shooter.*;
import frc.robot.commands.Traction.AtivarTurbo;
import frc.robot.commands.Traction.Controller;

@SuppressWarnings ("unused")
public class RobotContainer {

    /* ===== SUBSYSTEMS ===== */
    private final Shooter shooter = new Shooter();
    private final Traction traction = new Traction();
    private final Angulador angulador = new Angulador();
    private final Limelight limelight = new Limelight();
    private final IntakeFloor intakeFloor = new IntakeFloor();
    private final Index index = new Index();
    //private final Climber climber = new Climber(); 

    /* ===== CONTROLES ===== */
    private final XboxController xbox1 = new XboxController(0);
    private final XboxController xbox2 = new XboxController(1);

    /* ===== BOTOES ===== */
    private final JoystickButton btnTurbo =
        new JoystickButton(xbox1, XboxController.Button.kA.value);

    // LIMELIGHT
    private final JoystickButton rb =
        new JoystickButton(xbox2, XboxController.Button.kRightBumper.value);

    private final JoystickButton lb =
        new JoystickButton(xbox2, XboxController.Button.kLeftBumper.value);

    private final Trigger rt =
    new Trigger(() -> xbox2.getRightTriggerAxis() > 0.2);

    private final Trigger lt =
        new Trigger(() -> xbox2.getLeftTriggerAxis() > 0.2);

    // SHOOTER
    private final JoystickButton btnA =
        new JoystickButton(xbox2, XboxController.Button.kA.value);


    private final JoystickButton btnX =
        new JoystickButton(xbox2, XboxController.Button.kX.value);

    private final JoystickButton btnB =
        new JoystickButton(xbox2, XboxController.Button.kB.value);

    private final JoystickButton btnY =
        new JoystickButton(xbox2, XboxController.Button.kY.value);
    private final JoystickButton btnRb = new JoystickButton(xbox1, 6);
    private final JoystickButton btnLb = new JoystickButton(xbox1, 5);

    public RobotContainer() {
        configureBindings();

        // CONTROLE MANUAL DA TRAÇÃO (DEFAULT)
        traction.setDefaultCommand(
            new Controller(traction, xbox1)
        );
    }

    private void configureBindings() {

        /* ===== TRACAO ===== */
        btnTurbo.onTrue(new AtivarTurbo(traction));

        /* ===== LIMELIGHT / APRILTAG ===== */
        btnRb.whileTrue(
            new AlinhadorHorizontalAprilTag(limelight, traction )
        );

        btnLb.whileTrue(
            new AlinhadorVerticalAprilTag(limelight, traction)
        );

        /* ===== SHOOTER ===== */
        btnX.onTrue(
    new ShooterVelocidade(
        shooter,
        ConstantesShooter.Velocidade.MEDIA
    )
);

btnB.whileTrue(
    new ShooterVelocidade(
        shooter,
        ConstantesShooter.Velocidade.ALTA
    )
);

btnY.whileTrue(
    new ShooterVelocidade(
        shooter,
        ConstantesShooter.Velocidade.TURBO
    )
);

        rt.onTrue(new ToggleSequencialShooterIndex(shooter, index));

        lt.onTrue(
            new PararSequencialIndexShooter(shooter, index));

        /*rb.whileTrue(new MoverPivotPreset(
        intakeFloor,
        AngulosPresetPivot.BAIXO
        ));

        lb.whileTrue(new MoverPivotPreset(
        intakeFloor,
        AngulosPresetPivot.ALTO
        ));*/

        rb.onTrue(new GirarIntakeReverse(intakeFloor) );
        lb.onTrue(new PararIntake(intakeFloor));

    /*new POVButton(xbox2, 0)
        .onTrue(new MoverAnguladoPreset(
            angulador,
            AnguloPreset.ALTO
        ));

    new POVButton(xbox2, 270)
        .onTrue(new MoverAnguladoPreset(
            angulador,
            AnguloPreset.CENTRAL
        ));

    new POVButton(xbox2, 180)
        .onTrue(new MoverAnguladoPreset(
            angulador,
            AnguloPreset.BAIXO
        ));*/
    new POVButton(xbox2, 0)
        .whileTrue(new MoverPivotPreset(
            intakeFloor,
            AngulosPresetPivot.ALTO
        ));
    new POVButton(xbox2, 180)
        .whileTrue(new MoverPivotPreset(
            intakeFloor,
            AngulosPresetPivot.BAIXO
        ));
    }

    /* ===== AUTONOMO ===== */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
    new AnguladorAuto(angulador, AnguloPreset.CENTRAL)
       );
    }
}
