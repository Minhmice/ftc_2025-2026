package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;
import java.nio.ByteOrder;

@I2cDeviceType()
@DeviceProperties(name = "TCS34725 Color Sensor", description = "Color Sensor from ams", xmlTag = "TCS34725")
public class TCS34725_ColorSensor extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    public static final I2cAddr I2C_ADDRESS = I2cAddr.create7bit(0x29);

    // --- THANH GHI CƠ BẢN ---
    private static final byte COMMAND_BIT = (byte) 0x80;
    private static final byte REG_ENABLE = 0x00;
    private static final byte REG_CDATAL = 0x14;
    private static final byte REG_RDATAL = 0x16;
    private static final byte REG_GDATAL = 0x18;
    private static final byte REG_BDATAL = 0x1A;

    public TCS34725_ColorSensor(HardwareMap hardwareMap, String deviceName) {
        this(hardwareMap.get(I2cDeviceSynch.class, deviceName));
    }

    public TCS34725_ColorSensor(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
        this.deviceClient.setI2cAddress(I2C_ADDRESS);
        this.deviceClient.engage();
        this.initializeSensor();
    }

    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }

    private void initializeSensor() {
        // Chỉ bật nguồn và bộ chuyển đổi ADC, không tinh chỉnh gì thêm.
        write8(REG_ENABLE, (byte) 0x01); // Bật nguồn (PON)
        try { Thread.sleep(3); } catch (InterruptedException e) { Thread.currentThread().interrupt(); } // Đợi bộ dao động ổn định
        write8(REG_ENABLE, (byte) 0x03); // Bật bộ chuyển đổi ADC (AEN)
    }

    private void write8(byte reg, byte value) {
        deviceClient.write8(COMMAND_BIT | reg, value);
    }

    // --- PHƯƠNG THỨC ĐỌC DỮ LIỆU THÔ ---

    private int read16(byte reg) {
        // Đọc 2 byte (16-bit) từ một thanh ghi cụ thể
        byte[] buffer = deviceClient.read(COMMAND_BIT | reg, 2);
        return TypeConversion.unsignedShortToInt(TypeConversion.byteArrayToShort(buffer, 0, ByteOrder.LITTLE_ENDIAN));
    }

    /**
     * Trả về giá trị thô (0-65535) của kênh màu Đỏ.
     */
    public int red() {
        return read16(REG_RDATAL);
    }

    /**
     * Trả về giá trị thô (0-65535) của kênh màu Xanh lá.
     */
    public int green() {
        return read16(REG_GDATAL);
    }

    /**
     * Trả về giá trị thô (0-65535) của kênh màu Xanh dương.
     */
    public int blue() {
        return read16(REG_BDATAL);
    }

    /**
     * Trả về giá trị thô (0-65535) của kênh trong suốt (độ sáng).
     */
    public int clear() {
        return read16(REG_CDATAL);
    }

    @Override
    public Manufacturer getManufacturer() { return Manufacturer.Other; }

    @Override
    public String getDeviceName() { return "TCS34725 Default"; }
}
