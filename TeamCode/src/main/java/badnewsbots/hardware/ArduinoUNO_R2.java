package badnewsbots.hardware;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import java.nio.charset.StandardCharsets;

//https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Writing-an-I2C-Driver

@I2cDeviceType
@DeviceProperties(name="Arduino UNO R2", xmlTag="ArduinoUNOR2")
public class ArduinoUNO_R2 extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    private static final String DEVICE_NAME = "Arduino UNO R2";
    private static final I2cAddr DEVICE_ADDRESS = new I2cAddr(9); // this must be the address specified the arduino code

    @Override
    public Manufacturer getManufacturer() {return Manufacturer.Other;}

    @Override
    protected synchronized boolean doInitialize() { // This method should return true if the device was initialized successfully
        return true;
    }
    @Override
    public String getDeviceName() {return DEVICE_NAME;}

    public ArduinoUNO_R2(I2cDeviceSynch deviceClient) { // called when init button on DS pressed???
        super(deviceClient, true);

        setOptimalReadWindow();
        deviceClient.setI2cAddress(DEVICE_ADDRESS);

        super.registerArmingStateCallback(false); // FTC SDK stuff I don't understand, has to do with disconnections
        deviceClient.engage(); // "Engage" the device - enables the use of it by code
    }

    public enum Register {
        // "registers" are more or less variables sent individually by a sensor using hexadecimal bVals to distinguish between them
        FIRST(0),
        MESSAGE(0x01),
        LAST(MESSAGE.bVal);
        // FIRST and LAST are used to generalize the code so you can just add new registers
        // as you need to and the read window will be recalculated (happens at runtime) according to the number of enums

        public int bVal;
        Register(int bVal) {
            this.bVal = bVal;
        }
    }

    protected void setOptimalReadWindow() {
        // Set a "read window" - the FTC SDK implementation of I2C sensor drivers
        // will be constantly running in a loop in another thread, reading ALL registers each time
        // for efficiency - rather than reading individual registers as requested
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        deviceClient.setReadWindow(readWindow);
    }

    public String readMessage() {
        return new String(deviceClient.read(Register.MESSAGE.bVal, 5), StandardCharsets.US_ASCII); // 5 byte length
    }
}
