package frc.robot.StatesMachines;

public class StateMachineAngulador {
    public enum Estado {
        DESABILITADO,
        MANUAL,
        PERFIL,
        HOLD,
        FALHA
    }

    private Estado estado = Estado.DESABILITADO;

    public Estado get() {
        return estado;
    }

    public boolean is(Estado e) {
        return estado == e;
    }

    public void set(Estado novo) {
        estado = novo;
    }
}
