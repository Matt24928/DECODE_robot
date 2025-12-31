package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Wait {

    private final ElapsedTime timer = new ElapsedTime();
    private boolean running = false;

    /** Pornește wait-ul */
    public void start() {
        timer.reset();
        running = true;
    }

    /** Oprește / resetează wait-ul */
    public void stop() {
        running = false;
    }

    /**
     * Verifică dacă wait-ul s-a terminat
     * @param ms durata în milisecunde
     */
    public boolean done(long ms) {
        if (!running) return true;

        if (timer.milliseconds() >= ms) {
            running = false;
            return true;
        }
        return false;
    }

    /** @return true dacă este activ */
    public boolean isRunning() {
        return running;
    }

    /** @return timp scurs (ms) */
    public long elapsed() {
        return (long) timer.milliseconds();
    }
}
