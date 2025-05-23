package controllers;

public class DawgTimer {
    public class ReadingsNotUpdatedException extends Exception {}

    private static double currentMillisTimestamp = Double.NaN;
    public static void updateAllTimers(){
        currentMillisTimestamp = (double)System.nanoTime() / 1e6;
    }

    private double offset = 0;
    public void reset(){
        offset = currentMillisTimestamp;
    }
    public void unreset(){
        offset = -10000;
    }

    public double milliseconds() {
        try {
            if (Double.isNaN(currentMillisTimestamp))
                throw new ReadingsNotUpdatedException();
            return currentMillisTimestamp - offset;
        } catch (ReadingsNotUpdatedException e){
            throw new RuntimeException(e);
        }
    }
    public double seconds() {
        try {
            if (Double.isNaN(currentMillisTimestamp))
                throw new ReadingsNotUpdatedException();
            return (currentMillisTimestamp - offset) / 1e3;
        } catch (ReadingsNotUpdatedException e){
            throw new RuntimeException(e);
        }
    }
}
