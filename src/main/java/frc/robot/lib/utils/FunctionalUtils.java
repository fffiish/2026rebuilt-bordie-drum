package frc.robot.lib.utils;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

public final class FunctionalUtils {

  private FunctionalUtils() {}

  public static <T> Supplier<T> map(Function<T, T> mapFunc, Supplier<T> supplier) {
    return () -> mapFunc.apply(supplier.get());
  }

  public static DoubleSupplier map(Function<Double, Double> mapFunc, DoubleSupplier supplier) {
    return () -> mapFunc.apply(supplier.getAsDouble());
  }
}
