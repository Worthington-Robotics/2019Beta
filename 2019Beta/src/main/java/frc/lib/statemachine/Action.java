package frc.lib.statemachine;

public interface Action {
    void onStart();
    void onLoop();
    boolean isFinsihed();
    void onStop();
}
