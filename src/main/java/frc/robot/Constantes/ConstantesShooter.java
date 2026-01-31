package frc.robot.Constantes;

public class ConstantesShooter {

    public enum Velocidade {
        NORMAL(0.40),
        MEDIA(0.60),
        ALTA(0.80),
        TURBO(0.825);

        public final double valor;

        Velocidade(double valor) {
            this.valor = valor;
        }
    }

    public static final double RPM_MIN_GIRANDO = 50.0;

    private ConstantesShooter() {}
}
