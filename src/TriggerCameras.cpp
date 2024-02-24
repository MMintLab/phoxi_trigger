#include <sys/stat.h>
#include "PhoXi.h"
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>

#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std;

// number of scanners you want to connect to
const size_t SCANNERS_COUNT = 2;
const string MOTIONCAM_SERIAL = "DHV-146";
const string PHOTONEO_SERIAL = "JHE-129";

class TriggerCameras {
private:
    pho::api::PhoXiFactory Factory;
    std::vector<pho::api::PPhoXi> PhoXiDevices;
    std::vector<pho::api::PhoXiDeviceInformation> DeviceList;
    int frame_idx = 0;

    bool ConnectScanners();

    pho::api::PPhoXi ConnectPhoXiDeviceBySerial(const std::string &HardwareIdentification);

    void StartCameras();

    void Disconnect();

    void SaveFrames(std::vector<pho::api::PFrame> Frames);


public:
    TriggerCameras() = default;

    void Setup();

    void SoftwareTrigger();
};

pho::api::PPhoXi TriggerCameras::ConnectPhoXiDeviceBySerial(const std::string &HardwareIdentification) {
    pho::api::PhoXiTimeout Timeout = pho::api::PhoXiTimeout::ZeroTimeout;
    auto PhoXiDevice = Factory.CreateAndConnect(HardwareIdentification, Timeout);
    if (PhoXiDevice) {
        std::cout << "Connection to the device " << HardwareIdentification
                  << " was successful!" << std::endl;
        return PhoXiDevice;
    } else {
        std::cout << "Connection to the device " << HardwareIdentification
                  << " was unsuccessful!" << std::endl;
        return nullptr;
    }
}

bool TriggerCameras::ConnectScanners() {
    // check if scanner is not already connected
    for (size_t j = 0; j < PhoXiDevices.size(); ++j) {
        if (PhoXiDevices.at(j)->HardwareIdentification == MOTIONCAM_SERIAL) {
            std::cout << "Already connected " << std::endl;
            return false;
        }
    }
    auto Motioncam = ConnectPhoXiDeviceBySerial(MOTIONCAM_SERIAL);
    if (Motioncam) {
        PhoXiDevices.push_back(Motioncam);
    } else {
        return false;
    }

    for (size_t j = 0; j < PhoXiDevices.size(); ++j) {
        if (PhoXiDevices.at(j)->HardwareIdentification == PHOTONEO_SERIAL) {
            std::cout << "Already connected " << std::endl;
            return false;
        }
    }
    auto Photoneo = ConnectPhoXiDeviceBySerial(PHOTONEO_SERIAL);
    if (Photoneo) {
        PhoXiDevices.push_back(Photoneo);
    } else {
        return false;
    }

    // check if connected to requested num of devices
    if (PhoXiDevices.size() != SCANNERS_COUNT) {
        return false;
    }
    // check if all devices are connected
    for (size_t i = 0; i < SCANNERS_COUNT; ++i) {
        if (!PhoXiDevices[i]->isConnected()) {
            PhoXiDevices.clear();
            return false;
        }
    }

    // all devices are connected
    return true;
}

void TriggerCameras::Disconnect() {
    // The whole API is designed on C++ standards, using smart pointers and
    // constructor/destructor logic All resources will be closed automatically,
    // but the device state will not be affected -> it will remain connected in
    // PhoXi Control and if in freerun, it will remain Scanning To Stop the
    // device, just
    for (size_t i = 0; i < PhoXiDevices.size(); ++i) {
        if (PhoXiDevices[i]) {
            PhoXiDevices[i]->StopAcquisition();
        }
    }
}


void TriggerCameras::StartCameras() {
    // Check if the device is connected
    for (size_t i = 0; i < SCANNERS_COUNT; ++i) {
        if (!PhoXiDevices[i] || !PhoXiDevices[i]->isConnected()) {
            std::cout << "Device is not created, or not connected!" << std::endl;
            return;
        }
    }
    // If it is not in Software trigger mode, we need to switch the modes
    for (size_t i = 0; i < SCANNERS_COUNT; ++i) {
        if (PhoXiDevices[i]->TriggerMode !=
            pho::api::PhoXiTriggerMode::Software) {
            if (PhoXiDevices[i]->isAcquiring()) {
                if (!PhoXiDevices[i]->StopAcquisition()) {
                    throw std::runtime_error("Error in StopAcquistion");
                }
            }
            PhoXiDevices[i]->TriggerMode = pho::api::PhoXiTriggerMode::Software;
            if (!PhoXiDevices[i]->TriggerMode.isLastOperationSuccessful()) {
                throw std::runtime_error(PhoXiDevices[i]->TriggerMode.GetLastErrorMessage().c_str());
            }
        }
    }

    auto MarkerSpaceSetting = pho::api::PhoXiCoordinatesSettings();
    MarkerSpaceSetting.CoordinateSpace = pho::api::PhoXiCoordinateSpace::MarkerSpace;
    MarkerSpaceSetting.MarkersSettings.InvertedMarkers = true;
    MarkerSpaceSetting.RecognizeMarkers = true;

    for (size_t i = 0; i < SCANNERS_COUNT; ++i) {
        bool set_values = PhoXiDevices[i]->CoordinatesSettings.SetValue(MarkerSpaceSetting);
        if (!set_values) {
            throw std::runtime_error("Could not set camera settings");
        }
    }

    // Start the device acquisition, if necessary
    for (size_t i = 0; i < SCANNERS_COUNT; ++i) {
        if (!PhoXiDevices[i]->isAcquiring()) {
            if (!PhoXiDevices[i]->StartAcquisition()) {
                throw std::runtime_error("Error in StartAcquisition");
            }
        }
    }

    // We can clear the current Acquisition buffer -- This will not clear Frames
    // that arrives to the PC after the Clear command is performed
    for (size_t i = 0; i < SCANNERS_COUNT; ++i) {
        int ClearedFrames = PhoXiDevices[i]->ClearBuffer();
    }
}


void TriggerCameras::SoftwareTrigger() {
    frame_idx += 1;

    std::vector<pho::api::PFrame> Frames;
    for (size_t j = 0; j < SCANNERS_COUNT; ++j) {
        auto phoxi = PhoXiDevices[j];
        int FrameID = phoxi->TriggerFrame(true);
        if (FrameID < 0) {
            // If negative number is returned trigger was unsuccessful
            std::cout << "Trigger was unsuccessful! code=" << FrameID << std::endl;
            continue;
        }

        pho::api::PFrame Frame = PhoXiDevices[j]->GetFrame();

        auto const base_path = "/home/andrea/phoxi_ws/src/phoxi_trigger/data/";
        auto const stem = phoxi->HardwareIdentification.GetValue() + "_" + std::to_string(frame_idx);
        mkdir((base_path + stem).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        phoxi->SaveLastOutput(base_path + stem + "/" + stem + ".ply");
        phoxi->SaveLastOutput(base_path + stem + "/" + stem + ".tif");
        phoxi->SaveLastOutput(base_path + stem + "/" + stem + ".png");
    }
}

void TriggerCameras::Setup() {
    try {
        auto success = ConnectScanners();
        if (!success) {
            Disconnect();
            std::cout << "Can not connect to all requested scanners." << std::endl;
            return;
        }
        StartCameras();
    } catch (std::runtime_error &InternalException) {
        std::cout << std::endl << "Exception was thrown: " << InternalException.what() << std::endl;
        for (auto Device: PhoXiDevices) {
            if (Device->isConnected()) {
                Device->Disconnect(true);
            }
        }
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "cpp_camera_trigger");

    TriggerCameras triggerCameras;

    triggerCameras.Setup();

    for (auto i = 0; i < 4; ++i) {
        triggerCameras.SoftwareTrigger();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

//    void TriggerCallback(const std_msgs::String::ConstPtr& msg){
//        DataCollection.SoftwareTrigger();
//    }
//    ros::NodeHandle n;
//    ros::Subscriber sub = n.subscribe("camera_trigger", 10, TriggerCallback);
//    ros::spin();
    return 0;
}





