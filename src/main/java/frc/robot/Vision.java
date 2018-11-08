package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

public class Vision {
    private SerialPort port;
    private boolean initialized = false;
    private StatusCheck enabledStatus;
    private Thread thread;

    public enum Color {
        RED, BLUE, NONE,
    }

    private String remainingInput = "";
    private volatile Color ballColor;

    public Vision(StatusCheck enabled) {
        enabledStatus = enabled;

        initializePort();
        thread = createThread();
    }

    public interface StatusCheck {
        public boolean get();
    }

    public void start() {
        thread.start();
    }

    public void stop() {
        thread.interrupt();
        thread = createThread();
    }

    public synchronized Color getBallColor() {
        return ballColor;
    }

    private Thread createThread() {
        return new Thread(() -> {
            while (!initialized && !Thread.interrupted() && enabledStatus.get()) {
                Timer.delay(2.0);
                initializePort();
            }

            while (!Thread.interrupted() && enabledStatus.get()) {
                try {
                    parseInput(remainingInput + port.readString());
                } catch (Exception e) {
                    System.out.println(e.toString());
                }
                Timer.delay(0.034);
            }
        });
    }

    private void parseInput(String input) {
        int redIndex = input.lastIndexOf("RED");
        int blueIndex = input.lastIndexOf("BLUE");
        int noneIndex = input.lastIndexOf("NONE");

        if (redIndex > blueIndex && redIndex > noneIndex) {
            setBallColor(Color.RED);
            remainingInput = input.substring(redIndex + 3);
        } else if (blueIndex > redIndex && blueIndex > noneIndex) {
            setBallColor(Color.BLUE);
            remainingInput = input.substring(blueIndex + 4);
        } else if (noneIndex > redIndex && noneIndex > blueIndex) {
            setBallColor(Color.NONE);
            remainingInput = input.substring(noneIndex + 4);
        } else {
            remainingInput = input;
        }
    }

    private synchronized void setBallColor(Color ballColor) {
        this.ballColor = ballColor;
    }

    private void initializePort() {
        try {
            port = new SerialPort(9600, SerialPort.Port.kUSB);
            initialized = true;
        } catch (Exception e) {
            initialized = false;
        }
    }
}
