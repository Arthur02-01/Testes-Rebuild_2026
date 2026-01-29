package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.subsystems.Traction;
import frc.robot.subsystems.Angulador;
import frc.robot.subsystems.AnguloPreset;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.VelocidadeShooter;
import frc.robot.commands.Limelight.AlinhadorHorizontalAprilTag;
import frc.robot.commands.Limelight.AlinhadorVerticalAprilTag;
import frc.robot.commands.Autonomo.LimelightAuto.AlinhadorHorizontalAuto;
import frc.robot.commands.Autonomo.LimelightAuto.AlinhadorVerticalAuto;
import frc.robot.commands.Autonomo.Tracao.GiroPorAngulo;
import frc.robot.commands.Angulador.MoverAnguladoPreset;
import frc.robot.commands.Climber.ClimberStep;
import frc.robot.commands.Shooter.*;
import frc.robot.commands.Traction.AtivarTurbo;
import frc.robot.commands.Traction.Controller;

public class RobotContainer {

    /* ===== SUBSYSTEMS ===== */
    private final Shooter shooter = new Shooter();
    private final Traction traction = new Traction();
    private final Angulador angulador = new Angulador();
    private final Limelight limelight = new Limelight();
    private final Climber climber = new Climber(); 

    /* ===== CONTROLES ===== */
    private final XboxController xbox1 = new XboxController(0);
    private final XboxController xbox2 = new XboxController(1);

    /* ===== BOTÕES ===== */
    private final JoystickButton btnTurbo =
        new JoystickButton(xbox1, XboxController.Button.kA.value);

    // LIMELIGHT
    private final JoystickButton rb =
        new JoystickButton(xbox2, XboxController.Button.kRightBumper.value);

    private final JoystickButton lb =
        new JoystickButton(xbox2, XboxController.Button.kLeftBumper.value);

    // SHOOTER
    private final JoystickButton btnA =
        new JoystickButton(xbox2, XboxController.Button.kA.value);


    private final JoystickButton btnX =
        new JoystickButton(xbox2, XboxController.Button.kX.value);

    private final JoystickButton btnB =
        new JoystickButton(xbox2, XboxController.Button.kB.value);

    private final JoystickButton btnY =
        new JoystickButton(xbox2, XboxController.Button.kY.value);
        private final JoystickButton L4 = new JoystickButton(xbox2, 7);
    private final JoystickButton R4 = new JoystickButton(xbox2, 8);

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

        /* ===== TRAÇÃO ===== */
        btnTurbo.onTrue(new AtivarTurbo(traction));

        /* ===== LIMELIGHT / APRILTAG ===== */
        btnRb.whileTrue(
            new AlinhadorHorizontalAprilTag(limelight, traction )
        );

        btnLb.whileTrue(
            new AlinhadorVerticalAprilTag(limelight, traction)
        );

        /* ===== SHOOTER ===== */
        rb.onTrue(new AtivarFrenteShooter(shooter));
        lb.onTrue(new AtivarAtrasShooter(shooter));//lb trocado para testar

        btnA.onTrue(new PararShooter(shooter));

        btnY.whileTrue(new ShooterAutoPorDistancia(shooter, limelight));

        
        btnX.onTrue(new ShooterVelocidade(shooter, VelocidadeShooter.MEDIA));
        btnB.onTrue(new ShooterVelocidade(shooter, VelocidadeShooter.ALTA));
        btnY.onTrue(new ShooterVelocidade(shooter, VelocidadeShooter.TURBO));

        new JoystickButton(xbox1, 0);

        R4.debounce(0.15).onTrue(
            new ClimberStep(
                climber,
                0.10   // sobe 10 cm
            )
        );

        L4.debounce(0.15).onTrue(
            new ClimberStep(
                climber,
                -0.10  // desce 10 cm
            )
        );  

new POVButton(xbox2, 0)
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
    ));
    }

    /* ===== AUTÔNOMO ===== */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
     new GiroPorAngulo(traction, 180),
     new AlinhadorHorizontalAuto(limelight, traction, 7),
     new AlinhadorVerticalAuto(limelight, traction, 7)
       );
    }
}
