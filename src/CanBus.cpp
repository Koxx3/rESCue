#include "CanBus.h"
#include <Logger.h>

#ifdef CANBUS_ENABLED

CAN_device_t CAN_cfg;

CanBus::CanBus()
{
    stream = new LoopbackStream(BUFFER_SIZE);
}

void CanBus::init()
{
    CAN_cfg.speed = CAN_SPEED_500KBPS;
    CAN_cfg.tx_pin_id = (gpio_num_t)PIN_CAN_TX;
    CAN_cfg.rx_pin_id = (gpio_num_t)PIN_CAN_RX;
    CAN_cfg.rx_queue = xQueueCreate(1000, sizeof(CAN_frame_t));
    /*
      CAN_filter_t p_filter;
      p_filter.FM = Single_Mode;

      p_filter.ACR0 = 0x00;
      p_filter.ACR1 = 0x00;
      p_filter.ACR2 = 0x10;
      p_filter.ACR3 = 0x19;

      p_filter.AMR0 = 0x1F;
      p_filter.AMR1 = 0xFF;
      p_filter.AMR2 = 0xFF;
      p_filter.AMR3 = 0xFF;
      ESP32Can.CANConfigFilter(&p_filter);
    */
    // start CAN Module
    ESP32Can.CANInit();
    vesc_id = AppConfiguration::getInstance()->config.vescId;
    esp_can_id = vesc_id + 1;
    ble_proxy_can_id = vesc_id + 2;
    RECV_STATUS_1 = (uint32_t(0x0000) << 16) + (uint16_t(CAN_PACKET_STATUS) << 8) + this->vesc_id;
    RECV_STATUS_2 = (uint32_t(0x0000) << 16) + (uint16_t(CAN_PACKET_STATUS_2) << 8) + this->vesc_id;
    RECV_STATUS_3 = (uint32_t(0x0000) << 16) + (uint16_t(CAN_PACKET_STATUS_3) << 8) + this->vesc_id;
    RECV_STATUS_4 = (uint32_t(0x0000) << 16) + (uint16_t(CAN_PACKET_STATUS_4) << 8) + vesc_id;
    RECV_STATUS_5 = (uint32_t(0x0000) << 16) + (uint16_t(CAN_PACKET_STATUS_5) << 8) + vesc_id;

    RECV_FILL_RX_BUFFER = (uint32_t(0x0000) << 16) + (uint16_t(CAN_PACKET_FILL_RX_BUFFER) << 8) + esp_can_id;
    RECV_PROCESS_RX_BUFFER = (uint32_t(0x0000) << 16) + (uint16_t(CAN_PACKET_PROCESS_RX_BUFFER) << 8) + esp_can_id;

    RECV_PROCESS_SHORT_BUFFER_PROXY =
        (uint32_t(0x0000) << 16) + (uint16_t(CAN_PACKET_PROCESS_SHORT_BUFFER) << 8) + ble_proxy_can_id;
    RECV_FILL_RX_BUFFER_PROXY =
        (uint32_t(0x0000) << 16) + (uint16_t(CAN_PACKET_FILL_RX_BUFFER) << 8) + ble_proxy_can_id;
    RECV_FILL_RX_BUFFER_LONG_PROXY =
        (uint32_t(0x0000) << 16) + (uint16_t(CAN_PACKET_FILL_RX_BUFFER_LONG) << 8) + ble_proxy_can_id;
    RECV_PROCESS_RX_BUFFER_PROXY =
        (uint32_t(0x0000) << 16) + (uint16_t(CAN_PACKET_PROCESS_RX_BUFFER) << 8) + ble_proxy_can_id;
}

/*
  The VESC has to be configured to send status 1-5 regularly. It is recommenden to reduce the
  interval from 50Hz to something around 1-5Hz, which is absolutely sufficient for this application.
*/
void CanBus::loop()
{
    int frameCount = 0;
    CAN_frame_t rx_frame;
    /*
        if (initialized)
        {
            if (millis() - lastRealtimeData > interval) {
                requestRealtimeData();
            }
        }
        else if (initRetryCounter > 0 && millis() - lastRetry > 500)
        {
            requestFirmwareVersion();
            initRetryCounter--;
            lastRetry = millis();
            if (initRetryCounter == 0)
            {
                Logger::error("CANBUS initialization failed");
            }
        }
        */

    // receive next CAN frame from queue
    while (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE)
    {
        if (!initialized)
        {
            Logger::notice(LOG_TAG_CANBUS, "CANBUS is now initialized");
            initialized = true;
        }
        frameCount++;
        // VESC only uses ext packages, so skip std packages
        if (rx_frame.FIR.B.FF == CAN_frame_ext)
        {
            if (Logger::getLogLevel() <= Logger::VERBOSE)
            {
                printFrame(rx_frame, frameCount);
            }
            processFrame(rx_frame, frameCount);
        }
        if (frameCount > 1000)
        {
            // WORKAROUND if messages arrive too fast
            Logger::error(LOG_TAG_CANBUS, "reached 1000 frames in one loop, abort");
            buffer.clear();
            return;
        }
    }
    if (Logger::getLogLevel() <= Logger::VERBOSE)
    {
        // dumpVescValues();
    }
}

void CanBus::requestFirmwareVersion()
{
    Logger::notice(LOG_TAG_CANBUS, "requestFirmwareVersion");
    CAN_frame_t tx_frame = {};

    tx_frame.FIR.B.FF = CAN_frame_ext;
    tx_frame.MsgID = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_PROCESS_SHORT_BUFFER) << 8) + vesc_id;
    tx_frame.FIR.B.DLC = 0x03;
    tx_frame.data.u8[0] = esp_can_id;
    tx_frame.data.u8[1] = 0x00;
    tx_frame.data.u8[2] = 0x00; // COMM_FW_VERSION
    sendCanFrame(&tx_frame);
}

void CanBus::requestRealtimeData()
{
    Logger::notice(LOG_TAG_CANBUS, "requestRealtimeData");
    CAN_frame_t tx_frame = {};

    tx_frame.FIR.B.FF = CAN_frame_ext;
    tx_frame.MsgID = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_PROCESS_SHORT_BUFFER) << 8) + vesc_id;
    tx_frame.FIR.B.DLC = 0x07;
    tx_frame.data.u8[0] = esp_can_id;
    tx_frame.data.u8[1] = 0x00;
    tx_frame.data.u8[2] = 0x32; // COMM_GET_VALUES_SELECTIVE
    // mask
    tx_frame.data.u8[3] = 0x00;      // Byte1 of mask (Bits 24-31)
    tx_frame.data.u8[4] = 0x00;      // Byte2 of mask (Bits 16-23)
    tx_frame.data.u8[5] = B10000111; // Byte3 of mask (Bits 8-15)
    tx_frame.data.u8[6] = B11000011; // Byte4 of mask (Bits 0-7)
    sendCanFrame(&tx_frame);
}

void CanBus::requestBalanceData()
{
    Logger::notice(LOG_TAG_CANBUS, "requestBalanceData");
    CAN_frame_t tx_frame = {};

    tx_frame.FIR.B.FF = CAN_frame_ext;
    tx_frame.MsgID = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_PROCESS_SHORT_BUFFER) << 8) + vesc_id;
    tx_frame.FIR.B.DLC = 0x03;
    tx_frame.data.u8[0] = esp_can_id;
    tx_frame.data.u8[1] = 0x00;
    tx_frame.data.u8[2] = 0x4F;
    sendCanFrame(&tx_frame);
}

void CanBus::ping()
{
    CAN_frame_t tx_frame = {};
    tx_frame.FIR.B.FF = CAN_frame_ext;
    tx_frame.MsgID = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_PING) << 8) + vesc_id;
    tx_frame.FIR.B.DLC = 0x01;
    tx_frame.data.u8[0] = esp_can_id;
    sendCanFrame(&tx_frame);
}

void CanBus::printFrame(CAN_frame_t rx_frame, int frameCount)
{
    if (rx_frame.FIR.B.RTR == CAN_RTR)
        printf("#%d RTR from 0x%08x, DLC %d\r\n", frameCount, rx_frame.MsgID, rx_frame.FIR.B.DLC);
    else
    {
        printf("#%d from 0x%08x, DLC %d, data [", frameCount, rx_frame.MsgID, rx_frame.FIR.B.DLC);
        for (int i = 0; i < 8; i++)
        {
            printf("%d", (uint8_t)rx_frame.data.u8[i]);
            if (i != 7)
            {
                printf("\t");
            }
        }
        printf("]\n");
    }
}

void CanBus::processFrame(CAN_frame_t rx_frame, int frameCount)
{
    String frametype = "";
    uint32_t ID = rx_frame.MsgID;
    if (RECV_STATUS_1 == ID)
    {
        frametype = "status1";
    }
    if (RECV_STATUS_2 == ID)
    {
        frametype = "status2";
    }
    if (RECV_STATUS_3 == ID)
    {
        frametype = "status3";
    }
    if (RECV_STATUS_4 == ID)
    {
        frametype = "status4";
    }
    if (RECV_STATUS_5 == ID)
    {
        frametype = "status5";
    }

    if (RECV_PROCESS_SHORT_BUFFER_PROXY == ID)
    {
        frametype = "process short buffer for <<BLE proxy>>";
        for (int i = 1; i < rx_frame.FIR.B.DLC; i++)
        {
            proxybuffer.push_back(rx_frame.data.u8[i]);
        }
        proxyOut(proxybuffer.data(), proxybuffer.size(), rx_frame.data.u8[4], rx_frame.data.u8[5]);
        proxybuffer.clear();
    }

    if (RECV_FILL_RX_BUFFER == ID)
    {
        frametype = "fill rx buffer";
        for (int i = 1; i < rx_frame.FIR.B.DLC; i++)
        {
            buffer.push_back(rx_frame.data.u8[i]);
        }
    }

    if (RECV_FILL_RX_BUFFER_PROXY == ID || RECV_FILL_RX_BUFFER_LONG_PROXY == ID)
    {
        boolean longBuffer = RECV_FILL_RX_BUFFER_LONG_PROXY == ID;
        frametype = longBuffer ? "fill rx long buffer" : "fill rx buffer";
        for (int i = (longBuffer ? 2 : 1); i < rx_frame.FIR.B.DLC; i++)
        {
            proxybuffer.push_back(rx_frame.data.u8[i]);
        }
    }

    if (RECV_PROCESS_RX_BUFFER == ID || RECV_PROCESS_RX_BUFFER_PROXY == ID)
    {
        boolean isProxyRequest = false;
        frametype = "process rx buffer for ";
        if (RECV_PROCESS_RX_BUFFER_PROXY == ID)
        {
            frametype += " <<BLE proxy>> ";
            isProxyRequest = true;
        }
        // Serial.printf("bytes %d\n", buffer.size());

        if ((!isProxyRequest && buffer.size() <= 0) || (isProxyRequest && proxybuffer.size() <= 0))
        {
            Serial.printf("buffer empty, abort");
            return;
        }
        uint8_t command = (isProxyRequest ? proxybuffer : buffer).at(0);
        if (command == 0x00)
        {
            frametype += "firmwareversion";
            int offset = 1;
        }
        else if (command == 0x4F)
        { // 0x4F = 79 DEC
            frametype += "balancedata";
            int offset = 1;
            lastBalanceData = millis();
        }
        else if (command == 0x32)
        { // 0x32 = 50 DEC
            frametype += "realtimeData";
            int offset = 1;
            lastRealtimeData = millis();
        }
        else if (command == 0x04)
        {
            frametype += "COMM_GET_VALUES";
        }
        else if (command == 0x0E)
        { // 0x0E = 14 DEC
            frametype += "COMM_GET_MCCONF";
        }
        else if (command == 0x11)
        { // 0x11 = 17 DEC
            frametype += "COMM_GET_APPCONF";
        }
        else if (command == 0x2F)
        { // 0x2F = 47 DEC
            frametype += "COMM_GET_VALUES_SETUP";
        }
        else if (command == 0x33)
        { // 0x33 = 51 DEC
            frametype += "COMM_GET_VALUES_SETUP_SELECTIVE";
        }
        else if (command == 0x41)
        { // 0x41 = 65 DEC
            frametype += "COMM_GET_IMU_DATA";
        }
        else
        {
            frametype += command;
        }
        if (isProxyRequest)
        {
            proxyOut(proxybuffer.data(), proxybuffer.size(), rx_frame.data.u8[4], rx_frame.data.u8[5]);
            proxybuffer.clear();
        }
        else
        {
            buffer.clear();
        }
    }

    if (Logger::getLogLevel() <= Logger::NOTICE)
    {
        char buf[128];
        snprintf(buf, 128, "vesc_can_read processed frame #%d, type %s", frameCount, frametype.c_str());
        Logger::notice(LOG_TAG_CANBUS, buf);
    }
}

void CanBus::proxyIn(std::string in)
{
    uint8_t packet_type = (uint8_t)in.at(0);

    if (!longPacket)
    {
        switch (packet_type)
        {
        case 2:
            length = (uint8_t)in.at(1);
            command = (uint8_t)in.at(2);
            break;
        case 3:
            length = ((uint8_t)in.at(1) << 8) + (uint8_t)in.at(2);
            command = (uint8_t)in.at(3);
            longPacket = true;
            break;
        default:
            return;
        }
        if (Logger::getLogLevel() <= Logger::NOTICE)
        {
            char buf[64];
            snprintf(buf, 64, "proxyIn  : BLE/UART => CAN, command %d, length %d, longPacket %d", command, length, longPacket);
            Logger::notice(LOG_TAG_CANBUS, buf);
        }
    }

    if (longPacket)
    {
        longPackBuffer += in;
        // 7 bytes overhead
        if (length > (longPackBuffer.size() - 6))
        {
            if (Logger::getLogLevel() <= Logger::NOTICE)
            {
                char buf[64];
                snprintf(buf, 64, "proxyIn  : BLE/UART => CAN, Buffer not full, needed %d, is %d", length, longPackBuffer.size() - 6);
                Logger::notice(LOG_TAG_CANBUS, buf);
            }
            return;
        }

        if (Logger::getLogLevel() <= Logger::NOTICE)
        {
            char buf[128];
            snprintf(buf, 128, "proxyIn  : BLE/UART => CAN, Buffer full now processing, needed %d, is %d", length, longPackBuffer.size() - 6);
            Logger::notice(LOG_TAG_CANBUS, buf);
        }

        unsigned int end_a = 0;
        int offset = 2;
        // skip first two bytes CRC here
        for (unsigned int byteNum = offset; byteNum < length; byteNum += 7)
        {
            if (byteNum > 255 + offset)
            {
                break;
            }

            end_a = byteNum + 7;

            CAN_frame_t tx_frame = {};
            tx_frame.FIR.B.FF = CAN_frame_ext;
            tx_frame.MsgID = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_FILL_RX_BUFFER) << 8) + vesc_id;
            tx_frame.FIR.B.DLC = 8;
            tx_frame.data.u8[0] = byteNum - offset; // startbyte counter of frame

            int sendLen = (longPackBuffer.length() >= byteNum + 7) ? 7 : longPackBuffer.length() - byteNum;
            for (int i = 1; i < sendLen + 1; i++)
            {
                // Serial.printf("Reading byte %d, length %d\n", byteNum + i, longPackBuffer.length());
                tx_frame.data.u8[i] = (uint8_t)longPackBuffer.at(byteNum + i);
            }
            sendCanFrame(&tx_frame);
        }

        for (unsigned int byteNum = end_a - 1; byteNum < length; byteNum += 6)
        {
            CAN_frame_t tx_frame = {};
            tx_frame.FIR.B.FF = CAN_frame_ext;
            tx_frame.MsgID = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_FILL_RX_BUFFER_LONG) << 8) + vesc_id;
            tx_frame.FIR.B.DLC = 8;
            tx_frame.data.u8[0] = (byteNum - 1) >> 8;
            tx_frame.data.u8[1] = (byteNum - 1) & 0xFF;

            int sendLen = (longPackBuffer.length() >= byteNum + 6) ? 6 : longPackBuffer.length() - byteNum;
            for (int i = 2; i < sendLen + 2; i++)
            {
                // Serial.printf("Reading byte %d, length %d\n", byteNum + i, longPackBuffer.length());
                tx_frame.data.u8[i] = (uint8_t)longPackBuffer.at(byteNum + i);
            }

            sendCanFrame(&tx_frame);
        }

        CAN_frame_t tx_frame = {};
        tx_frame.FIR.B.FF = CAN_frame_ext;
        tx_frame.MsgID = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_PROCESS_RX_BUFFER) << 8) + vesc_id;
        tx_frame.FIR.B.DLC = 6;
        tx_frame.data.u8[0] = ble_proxy_can_id;
        tx_frame.data.u8[1] = 0; // IS THIS CORRECT?????
        tx_frame.data.u8[2] = length >> 8;
        tx_frame.data.u8[3] = length & 0xFF;
        tx_frame.data.u8[4] = longPackBuffer.at(longPackBuffer.size() - 3);
        tx_frame.data.u8[5] = longPackBuffer.at(longPackBuffer.size() - 2);
        sendCanFrame(&tx_frame);
    }
    else
    {
        CAN_frame_t tx_frame = {};
        tx_frame.FIR.B.FF = CAN_frame_ext;
        tx_frame.MsgID = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_PROCESS_SHORT_BUFFER) << 8) + vesc_id;
        tx_frame.FIR.B.DLC = 0x02 + length;
        tx_frame.data.u8[0] = ble_proxy_can_id;
        tx_frame.data.u8[1] = 0x00;
        tx_frame.data.u8[2] = command;
        for (int i = 3; i < length + 2; i++)
        {
            tx_frame.data.u8[i] = (uint8_t)in.at(i);
        }
        sendCanFrame(&tx_frame);
    }

    length = 0;
    command = 0;
    longPacket = false;
    longPackBuffer = "";
}

void CanBus::proxyOut(uint8_t *data, int size, uint8_t crc1, uint8_t crc2)
{
    if (size > BUFFER_SIZE)
    {
        Logger::error(LOG_TAG_CANBUS, "proxyOut - Buffer size exceeded, abort (message not sent via proxy)");
        return;
    }
    if (Logger::getLogLevel() <= Logger::NOTICE)
    {
        char buf[64];
        snprintf(buf, 64, "proxyOut : CAN => BLE/UART, sending %d bytes", size);
        Logger::notice(LOG_TAG_CANBUS, buf);
    }
    // Start bit, package size
    if (size <= 255)
    {
        // Serial.print(0x02);
        stream->write(0x02);
        // size
        // Serial.print(size);
        stream->write(size);
    }
    else if (size <= 65535)
    {
        // Serial.print(0x03);
        stream->write(0x03);
        // size
        // Serial.print(size >> 8);
        // Serial.print(size & 0xFF);
        stream->write(size >> 8);
        stream->write(size & 0xFF);
    }
    else
    {
        // Serial.print(0x04);
        stream->write(0x04);
        // size
        // Serial.print(size >> 16);
        // Serial.print((size >> 8) & 0x0F);
        // Serial.print(size & 0xFF);
        stream->write(size >> 16);
        stream->write((size >> 8) & 0x0F);
        stream->write(size & 0xFF);
    }

    // data
    for (int i = 0; i < size; i++)
    {
        // Serial.print(data[i]);
        stream->write(data[i]);
    }

    // crc 2 byte
    // Serial.print(crc1);
    // Serial.print(crc2);
    stream->write(crc1);
    stream->write(crc2);

    // Stop bit
    // Serial.print(0x03);
    stream->write(0x03);

    // Serial.println("");
}

void CanBus::sendCanFrame(const CAN_frame_t *p_frame)
{
    if (Logger::getLogLevel() <= Logger::NOTICE)
    {
        char buf[128];
        snprintf(buf, 128, "sendCanFrame : Sending CAN frame %x [%02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x]",
                 p_frame->MsgID,
                 p_frame->data.u8[0],
                 p_frame->data.u8[1],
                 p_frame->data.u8[2],
                 p_frame->data.u8[3],
                 p_frame->data.u8[4],
                 p_frame->data.u8[5],
                 p_frame->data.u8[6],
                 p_frame->data.u8[7]);
        Logger::notice(LOG_TAG_CANBUS, buf);
    }
    xSemaphoreTake(mutex_v, portMAX_DELAY);
    ESP32Can.CANWriteFrame(p_frame);
    xSemaphoreGive(mutex_v);
}

#endif // CANBUS_ENABLED