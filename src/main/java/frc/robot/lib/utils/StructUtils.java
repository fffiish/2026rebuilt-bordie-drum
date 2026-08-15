package frc.robot.lib.utils;

import static java.nio.charset.StandardCharsets.UTF_8;

import java.nio.ByteBuffer;

public class StructUtils {
  public static String readString(ByteBuffer bb, int length) {
    byte[] arr = new byte[length];
    bb.get(arr);
    int zeroIndex = 0;
    while (zeroIndex < arr.length && arr[zeroIndex] != 0) zeroIndex++;
    return new String(arr, 0, zeroIndex, UTF_8);
  }

  public static void writeString(ByteBuffer bb, String str, int length) {
    byte[] arr = str.getBytes(UTF_8);
    int bytesToWrite = Math.min(arr.length, length);
    bb.put(arr, 0, bytesToWrite);
    for (int i = bytesToWrite; i < length; i++) {
      bb.put((byte) 0);
    }
  }
}
