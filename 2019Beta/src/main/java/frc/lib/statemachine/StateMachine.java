package frc.lib.statemachine;

import edu.wpi.first.wpilibj.Timer;

import java.util.concurrent.ConcurrentLinkedQueue;

public class StateMachine {

    private volatile static int state = -1;
    private volatile static ConcurrentLinkedQueue<ActionGroup> qedStates;
    private volatile static ActionGroup currentState;
    private static Runnable Man = () -> {
        state = 0;
        //smart dash stuff here.
        if (qedStates == null) {
            state = -2;
        } else {
            while (!qedStates.isEmpty()) {
                currentState = qedStates.poll();
                currentState.onStart();
                while (!currentState.isFinnished()) {
                    currentState.onLoop();
                    Timer.delay(0.01);
                }
                currentState.onStop();
                state++;

            }
        }
    };

    public static void runMan(StateMachineDescriptor Dis) {
        qedStates = Dis.getStates();
        Thread thread = new Thread(Man);
        thread.start();

    }


}
