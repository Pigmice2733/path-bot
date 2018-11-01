package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

public class Vision {
    private SerialPort port;
    private boolean initialized = false;
    private StatusCheck enabledStatus;
    private Thread thread;

    private volatile String remainingInput = "";
    private volatile int[] ballData = { 0, 0, 0, 0 };

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

    public synchronized int[] getBallData() {
        return ballData.clone();
    }

    private Thread createThread() {
        return new Thread(() -> {
            while (!initialized && !Thread.interrupted() && enabledStatus.get()) {
                Timer.delay(2.0);
                initializePort();
            }

            port.setTimeout(0.03);

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
        int i = input.indexOf("RED");
        if (i != -1) {
            String[] redData;
            try {
                redData = input.substring(i + 4, i + 13).split(" ");
                if (redData.length == 2) {
                    try {
                        int x = Integer.parseInt(redData[0]);
                        int y = Integer.parseInt(redData[1]);
                        writeRedData(x, y);
                    } catch (NumberFormatException e) {
                        System.out.println(e.toString());
                    }
                }
            } catch (StringIndexOutOfBoundsException e) {
            }
        }

        i = input.indexOf("BLUE");
        if (i != -1) {
            String[] blueData;
            try {
                blueData = input.substring(i + 5, i + 14).split(" ");
                if (blueData.length == 2) {
                    try {
                        int x = Integer.parseInt(blueData[0]);
                        int y = Integer.parseInt(blueData[1]);
                        writeBlueData(x, y);
                    } catch (NumberFormatException e) {
                        System.out.println(e.toString());
                    }
                }
            } catch (StringIndexOutOfBoundsException e) {
            }

            try {
                remainingInput = input.substring(i + 18);
            } catch (StringIndexOutOfBoundsException e) {
            }
        }
    }

    private synchronized void writeRedData(int x, int y) {
        ballData[0] = x;
        ballData[1] = y;
    }

    private synchronized void writeBlueData(int x, int y) {
        ballData[2] = x;
        ballData[3] = y;
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