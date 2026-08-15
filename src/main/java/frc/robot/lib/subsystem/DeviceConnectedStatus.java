package frc.robot.lib.subsystem;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;
import lombok.Getter;
import lombok.Setter;

@Setter
@Getter
public class DeviceConnectedStatus implements StructSerializable {
  private boolean connected;
  private int id;

  public DeviceConnectedStatus(boolean connected, int id) {
    this.connected = connected;
    this.id = id;
  }

  @SuppressWarnings("unused")
  public static final Struct<DeviceConnectedStatus> struct =
      new Struct<>() {
        @Override
        public Class<DeviceConnectedStatus> getTypeClass() {
          return DeviceConnectedStatus.class;
        }

        @Override
        public String getTypeName() {
          return "DeviceConnectedStatus";
        }

        @Override
        public int getSize() {
          return kSizeBool + kSizeInt32;
        }

        @Override
        public String getSchema() {
          // spotless:off
                    return "bool Connected;int32 Id";
                    // spotless:on
        }

        @Override
        public DeviceConnectedStatus unpack(ByteBuffer bb) {
          boolean connected = bb.get() != 0;
          int id = bb.getInt();

          return new DeviceConnectedStatus(connected, id);
        }

        @Override
        public void pack(ByteBuffer bb, DeviceConnectedStatus value) {
          bb.put((byte) (value.connected ? 1 : 0));
          bb.putInt(value.getId());
        }

        @Override
        public boolean isImmutable() {
          return true;
        }
      };
}
