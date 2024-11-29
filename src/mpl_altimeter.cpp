// MPLAltimeterSensor.cpp
#include "mpl_altimeter.h"

MPLAltimeterSensor::MPLAltimeterSensor() : mpl(), newDataFlag_(false), lastUpdateTime(0) {}

bool MPLAltimeterSensor::begin()
{
    Wire.begin();
    if (!mpl.begin())
    {
        return false;
    }

    setOversampleRate(MPL3115A2_CTRL_REG1_OS1);
    mpl.setMode();
    return true;
}

void MPLAltimeterSensor::setOversampleRate(uint8_t oversampleRate)
{
    Wire.beginTransmission(MPL3115A2_ADDRESS);
    Wire.write(MPL3115A2_CTRL_REG1); // Register to write to
    Wire.write(oversampleRate);      // Oversample rate value
    Wire.endTransmission();

    Serial.print("Oversample rate set to: ");
    Serial.println(oversampleRate, HEX);
}

void MPLAltimeterSensor::update()
{
    unsigned long currentTime = millis();
    unsigned long desiredInterval = getUpdateInterval();
    if (currentTime - lastUpdateTime >= desiredInterval)
    {
        // Retrieve data from the sensor
        data_.pressure = mpl.getPressure(); // Populate pressure in Pa
        data_.altitude = mpl.getAltitude(); // Populate altitude in meters

        // Set new data flag
        newDataFlag_ = true;

        // Update the last update time
        lastUpdateTime = currentTime;
    }
}

flatbuffers::Offset<SensorLog::SensorMessage> MPLAltimeterSensor::serialize(flatbuffers::FlatBufferBuilder &builder, unsigned long timestamp) const
{
    // Create FlatBuffers MPLAltimeterData from the struct
    auto mplData = SensorLog::CreateMPLAltimeterData(builder, data_.pressure, data_.altitude);

    // Get the union offset
    auto dataOffset = mplData.Union();

    // Create the SensorMessage FlatBuffers object
    return SensorLog::CreateSensorMessage(
        builder,
        SensorLog::SensorType_MPLAltimeter,          // sensor_type
        timestamp,                                   // timestamp
        SensorLog::SensorDataUnion_MPLAltimeterData, // data_type (union type)
        dataOffset                                   // data (union data)
    );
}

String MPLAltimeterSensor::getName() const
{
    return "MPL3115A2";
}

SensorType MPLAltimeterSensor::getSensorType() const
{
    return SensorType::MPLAltimeter;
}

const SensorData *MPLAltimeterSensor::getData() const
{
    return &data_;
}

unsigned long MPLAltimeterSensor::getUpdateInterval() const
{
    return 40; // Approximately 30 Hz (1000 ms / 30 ≈ 33 ms)
}

bool MPLAltimeterSensor::hasNewData() const
{
    return newDataFlag_;
}

void MPLAltimeterSensor::resetNewDataFlag()
{
    newDataFlag_ = false;
}

void MPLAltimeterSensor::getData(void *data) const
{
    if (data)
    {
        memcpy(data, &data_, sizeof(MPLAltimeterDataStruct));
    }
}