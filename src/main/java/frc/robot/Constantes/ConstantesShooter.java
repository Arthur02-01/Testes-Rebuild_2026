package frc.robot.Constantes;

public class ConstantesShooter {

    public enum Velocidade {
        NORMAL(2400),
        MEDIA(3500),
        ALTA(4200),
        TURBO(5700);

        public final double rpm;

        Velocidade(double rpm) {
            this.rpm = rpm;
        }
    }

    public static final double TOLERANCIA_RPM = 100;

    private ConstantesShooter() {}
}
