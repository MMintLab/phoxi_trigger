#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <sys/stat.h>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "PhoXi.h"

namespace fs = std::filesystem;

// number of scanners you want to connect to
const size_t SCANNERS_COUNT = 2;
const std::string MOTIONCAM_SERIAL = "DHV-146";
const std::string PHOTONEO_SERIAL = "JHE-129";

class TriggerCameras {
public:
    explicit TriggerCameras(std::shared_ptr<ros::NodeHandle> nh) : nh(nh) {
        sub = nh->subscribe("camera_trigger", 10, &TriggerCameras::TriggerCallback, this);
    }

    void TriggerCallback(const std_msgs::String &msg) {
        SoftwareTrigger();
    }

    pho::api::PPhoXi ConnectPhoXiDeviceBySerial(const std::string &HardwareIdentification) {
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

    bool ConnectScanners() {
        // check if scanner is not already connected
        for (size_t j = 0; j < PhoXiDevices.size(); ++j) {
            if (PhoXiDevices.at(j)->HardwareIdentification == MOTIONCAM_SERIAL) {
                std::cout << "Already connected " << std::endl;
                return false;
            }
        }

        auto Motioncam = ConnectPhoXiDeviceBySerial(MOTIONCAM_SERIAL);

        pho::api::PhoXiMotionCam currentSettings = Motioncam->MotionCam.GetValue();
        // Check if the CurrentResolution has been retrieved succesfully
        if (!Motioncam->MotionCam.isLastOperationSuccessful()) {
            throw std::runtime_error(Motioncam->MotionCam.GetLastErrorMessage().c_str());
        }

        currentSettings.OperationMode = pho::api::PhoXiOperationMode::Scanner;

        Motioncam->MotionCam.SetValue(currentSettings);
        if (!Motioncam->MotionCam.isLastOperationSuccessful()) {
            throw std::runtime_error(Motioncam->MotionCam.GetLastErrorMessage().c_str());
        }

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

    void Disconnect() {
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


    void StartCameras() {
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


    void SoftwareTrigger() {
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

            auto const data_dir = "/home/andrea/phoxi_ws/src/phoxi_trigger/data/";
            auto const stem = phoxi->HardwareIdentification.GetValue() + "_" + std::to_string(frame_idx);
            auto const subdir = data_dir + stem + "/";

            fs::create_directory(subdir);

            auto const root = subdir + stem;
            phoxi->SaveLastOutput(root + ".ply");
            phoxi->SaveLastOutput(root + ".tif");
            phoxi->SaveLastOutput(root + ".png");
        }
    }

    void Setup() {
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

private:
    std::shared_ptr<ros::NodeHandle> nh;
    ros::Subscriber sub;

    pho::api::PhoXiFactory Factory;
    std::vector<pho::api::PPhoXi> PhoXiDevices;
    std::vector<pho::api::PhoXiDeviceInformation> DeviceList;
    int frame_idx = 0;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "cpp_camera_trigger");

    auto nh = std::make_shared<ros::NodeHandle>();
    TriggerCameras tc(nh);
    tc.Setup();

    ros::spin();

    return 0;
}





