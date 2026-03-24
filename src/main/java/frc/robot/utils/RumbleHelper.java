public class RumbleHelper {
    private enum State { IDLE, RUMBLING, WAITING }
    
    private final XboxController controller;
    private final Timer timer = new Timer();
    private State state = State.IDLE;
    private double rumbleDuration;
    private double gapDuration;
    private double intensity;
    private int rumbleCount;
    private int currentRumbleCount = 0;

    public RumbleHelper(XboxController controller) {
        this.controller = controller;
    }


    /**
    * @param duration Duration of pulse
    * @param gap time between pulses
    * @param count number of pulses
    * @param intensity Rumble intensity from 0 to 1
    */
    public void rumbleForDuration(double duration, double gap, int count, double intensity) {
        this.rumbleDuration = duration;
        this.gapDuration = gap;
        this.rumbleCount = count;
        this.currentRumbleCount = 0;
        this.intensity = MathUtil.clamp(intensity, 0, 1);
        startRumble();
    }

    public void update() {
        switch (state) {
            case RUMBLING:
                if (timer.hasElapsed(rumbleDuration)) {
                    stopRumble();
                    currentRumbleCount++;
                    if (currentRumbleCount < rumbleCount) {
                        // start waiting for the gap
                        timer.restart();
                        state = State.WAITING;
                    }
                }
                break;

            case WAITING:
                if (timer.hasElapsed(gapDuration)) {
                    startRumble();
                }
                break;

            case IDLE:
            default:
                break;
        }
    }

    public boolean isFinished() {
        return state == State.IDLE && currentRumbleCount >= rumbleCount;
    }

    private void startRumble() {
        controller.setRumble(XboxController.RumbleType.kBothRumble, intensity);
        timer.restart();
        state = State.RUMBLING;
    }

    private void stopRumble() {
        controller.setRumble(XboxController.RumbleType.kBothRumble, 0);
        state = State.IDLE;
    }
}
