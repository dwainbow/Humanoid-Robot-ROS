#include <dynamixel_sdk/dynamixel_sdk.h>
#include <ros/ros.h>

void scan_motors(){
    const char* port_name = "/dev/ttyUSB0"; // Replace with your actual port
    int baud_rates[] = {57600, 3000000};
    const int num_baud_rates = sizeof(baud_rates) / sizeof(baud_rates[0]);

    // Protocol versions to test
    float protocol_versions[] = {2.0};
    const int num_protocols = sizeof(protocol_versions) / sizeof(protocol_versions[0]);

    dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(port_name);

    for (int i = 0; i < num_baud_rates; ++i) {
        if (!portHandler->openPort()) {
            ROS_ERROR("Failed to open port: %s", port_name);
            return ;
        }

        if (!portHandler->setBaudRate(baud_rates[i])) {
            ROS_ERROR("Failed to set baud rate: %d", baud_rates[i]);
            portHandler->closePort();
            continue;
        }

        ROS_INFO("Scanning with baud rate: %d", baud_rates[i]);

        for (int j = 0; j < num_protocols; ++j) {
            float protocol_version = protocol_versions[j];
            dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(protocol_version);
            ROS_INFO("Using Protocol Version: %.1f", protocol_version);

            // Scan through all valid IDs (0 to 252)
            for (int id = 0; id <= 20; ++id) {
                uint16_t model_number = 0;
                uint8_t dxl_error = 0;

                int dxl_comm_result = packetHandler->ping(portHandler, id, &model_number, &dxl_error);

                if (dxl_comm_result == COMM_SUCCESS) {
                    ROS_INFO("Motor found! ID: %d, Model Number: %d, Protocol: %.1f, Baud Rate: %d",
                             id, model_number, protocol_version, baud_rates[i]);
                    break;
                } else if (dxl_comm_result != COMM_TX_FAIL) {
                    ROS_WARN("Error with ID %d: %s", id, packetHandler->getTxRxResult(dxl_comm_result));
                }
            }
        }

        portHandler->closePort();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "scan_motors");
    ros::NodeHandle nh;

    scan_motors();

    return 0;
}