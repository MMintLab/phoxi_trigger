#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <sys/stat.h>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#define PHOXI_PCL_SUPPORT

#include "PhoXi.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/ply_io.h>

namespace fs = std::filesystem;

// number of scanners you want to connect to
const size_t SCANNERS_COUNT = 2;
const std::string MOTIONCAM_SERIAL = "DHV-146";
const std::string PHOTONEO_SERIAL = "JHE-129";

class TriggerCameras {
public:
    explicit TriggerCameras(std::shared_ptr<ros::NodeHandle> nh) : nh(nh) {
        sub = nh->subscribe("camera_trigger", 10, &TriggerCameras::TriggerCallback, this);
        pc_pub = nh->advertise<sensor_msgs::PointCloud2>("point_cloud", 10);
    }

    void TriggerCallback(const std_msgs::String &msg) {

        using PointT = pcl::PointXYZRGBNormal;
        using PointCloudT = pcl::PointCloud<PointT>;
        PointCloudT combined_pcl_cloud;

        auto const data_dir = "/home/andrea/phoxi_ws/src/phoxi_trigger/data/";

        for (size_t j = 0; j < SCANNERS_COUNT; ++j) {
            auto phoxi = PhoXiDevices[j];
            phoxi->ClearBuffer();
            int FrameID = phoxi->TriggerFrame(true);
            if (FrameID < 0) {
                // If negative number is returned trigger was unsuccessful
                std::cout << "Trigger was unsuccessful! code=" << FrameID << std::endl;
                continue;
            }

            pho::api::PFrame frame = PhoXiDevices[j]->GetFrame();

            if (!frame) {
                std::cerr << "Frame was empty!" << std::endl;
                continue;
            }

            auto const stem = phoxi->HardwareIdentification.GetValue() + "_" + msg.data;
            auto const root = data_dir + stem;
//            phoxi->SaveLastOutput(root + ".ply");
//            phoxi->SaveLastOutput(root + ".tif");
            phoxi->SaveLastOutput(root + ".png");

            auto pcl_cloud_ptr = boost::make_shared<PointCloudT>();
            frame->ConvertTo<PointT>(*pcl_cloud_ptr);  // This requires the  #define PHOXI_PCL_SUPPORT

            // convert from millimeters to meters
            for (int i = 0; i < pcl_cloud_ptr->points.size(); i++) {
                pcl_cloud_ptr->points[i].x = pcl_cloud_ptr->points[i].x / 1000;
                pcl_cloud_ptr->points[i].y = pcl_cloud_ptr->points[i].y / 1000;
                pcl_cloud_ptr->points[i].z = pcl_cloud_ptr->points[i].z / 1000;
            }

            auto filtered_cloud_ptr = boost::make_shared<PointCloudT>();

            // Create the filtering object
            pcl::VoxelGrid<PointT> sor;
            sor.setInputCloud(pcl_cloud_ptr);
            sor.setLeafSize(0.002f, 0.002f, 0.002f);
            sor.filter(*filtered_cloud_ptr);

            combined_pcl_cloud += *filtered_cloud_ptr;
        }

        std::cout << "Combined cloud has " << combined_pcl_cloud.size() << " points.\n";

        sensor_msgs::PointCloud2 pc_msg;
        pcl::toROSMsg(combined_pcl_cloud, pc_msg);
        pc_msg.header.frame_id = "MarkerBoard";
        pc_msg.header.stamp = ros::Time::now();
        pc_pub.publish(pc_msg);

        auto const combined_stem = "combined_scan_" + msg.data;
        auto const combined_root = data_dir + combined_stem;

        pcl::PLYWriter writer;
        writer.write(combined_root + ".ply", combined_pcl_cloud, false);
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

private:
    std::shared_ptr<ros::NodeHandle> nh;
    ros::Subscriber sub;
    ros::Publisher pc_pub;

    pho::api::PhoXiFactory Factory;
    std::vector<pho::api::PPhoXi> PhoXiDevices;
    std::vector<pho::api::PhoXiDeviceInformation> DeviceList;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "phoxi_trigger");

    auto nh = std::make_shared<ros::NodeHandle>();
    TriggerCameras tc(nh);
    tc.Setup();

    ros::spin();

    return 0;
}





