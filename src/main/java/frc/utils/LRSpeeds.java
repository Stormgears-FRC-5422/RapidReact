package frc.utils;

import edu.wpi.first.math.Pair;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class LRSpeeds extends Pair<Double, Double> implements Loggable {

    Pair<Boolean, Boolean> disable = new Pair<>(false, false);

    public LRSpeeds(double left, double right) {
        super(left, right);
    }

    public LRSpeeds() {
        this(0,0);
    }

  @Log(name = "Left Speed")
  public double left() {
        if (Boolean.TRUE.equals(disable.getFirst())) return 0;
        return getFirst();
    }

  @Log(name = "Right Speed")
  public double right() {
        if (Boolean.TRUE.equals(disable.getSecond())) return 0;
        return getSecond();
    }

    public void disableRight(){
        disable = new Pair<>(disable.getFirst(), true);
    }

    public void disableLeft(){
        disable = new Pair<>(true, disable.getSecond());
    }

    public static LRSpeeds stop() {
        return new LRSpeeds();
    }
}
