

import inet.common.INETDefs;
import inet.common.packet.chunk.Chunk;


class VeinsInetSampleMessage extends inet::FieldsChunk
{
    string roadId;
    string carId;        // ID of the car that made the accident
    double accidentX;    // X coordinate of the accident location
    double accidentY;    // Y coordinate of the accident location
}
