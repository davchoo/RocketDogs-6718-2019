public class Profiler {
    private long startingTime;
    private long lastTime;

    public void start() {
        startingTime = System.nanoTime();
        lastTime = startingTime;
    }

    public void section(String name) {
        long currentTime = System.nanoTime();
        double deltaTime = (double) (currentTime - lastTime) / 1e6d;
        lastTime = currentTime;
        System.out.print(String.format("%s: %.2f ", name, deltaTime));
    }

    public void stop() {
        long currentTime = System.nanoTime();
        double deltaTime = (double) (currentTime - startingTime) / 1e6d;
        System.out.println(String.format("Total: %.2f Update Rate: %.2f", deltaTime, 1000d / deltaTime));
    }
}
