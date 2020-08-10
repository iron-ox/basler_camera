#include <ros/console.h>
#include <ros/ros.h>

#include <pylon/PylonIncludes.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>

#include <XmlRpcValue.h>

#include "basler_parameters.h"
#include "image_publisher.h"

namespace
{
const int load_camera_timeout = 5;
const int ptp_sync_timeout = 60;
const int startup_sleep_s = 5;
}

std::vector<Pylon::CBaslerGigEInstantCamera> loadCameras(const XmlRpc::XmlRpcValue& cameras_yaml)
{
    Pylon::CTlFactory& tl_factory = Pylon::CTlFactory::GetInstance();
    Pylon::DeviceInfoList_t devices;
    ros::Rate r(0.5);

    std::vector<Pylon::CBaslerGigEInstantCamera> cameras(cameras_yaml.size());
    for (size_t index = 0; index < cameras_yaml.size(); ++index)
    {
        ROS_ASSERT_MSG(cameras_yaml[index].getType() == XmlRpc::XmlRpcValue::TypeStruct, "Bad yaml for camera %zu", index);
        ROS_ASSERT_MSG(cameras_yaml[index].hasMember("serial"), "Camera %zu yaml missing serial", index);
        std::string serial = std::to_string(int(cameras_yaml[index]["serial"]));

        bool camera_found = false;
        ros::Time timeout_time = ros::Time::now() + ros::Duration(load_camera_timeout);
        while(ros::ok() && !camera_found)
        {
            tl_factory.EnumerateDevices(devices);
            for (size_t i = 0; i < devices.size(); ++i)
            {
                if (devices[i].GetSerialNumber().c_str() == serial)
                {
                    ROS_INFO_STREAM("Found camera " << index << " with matching serial " << serial);
                    cameras[index].Attach(tl_factory.CreateDevice(devices[i]));
                    camera_found = true;
                    break;
                }
            }
            if (!camera_found)
            {
                ROS_DEBUG_STREAM("Failed to find camera " << index << " with serial " << serial);
                ROS_DEBUG_STREAM( devices.size() << " other serials found:");
                for (size_t i = 0; i < devices.size(); ++i)
                {
                    ROS_DEBUG_STREAM(devices[i].GetSerialNumber());
                }
                if (ros::Time::now() > timeout_time)
                {
                    ROS_ERROR_STREAM("Timed out looking for camera " << index << " with serial " << serial);
                    return {};
                }
                r.sleep();
                ros::spinOnce();
            }
        }
    }
    return cameras;
}

// Wait for all cameras to be slaves and to have same master clock id
bool waitForPTPSlave(const std::vector<Pylon::CBaslerGigEInstantCamera>& cameras)
{
    ROS_DEBUG("Waiting for %zu cameras to be PTP slaves...", cameras.size());
    ros::Rate r(0.5);
    ros::Time timeout_time = ros::Time::now() + ros::Duration(ptp_sync_timeout);
    while (ros::ok())
    {
        int num_init = 0;
        int num_master = 0;
        int num_slave = 0;
        int64_t master_clock_id = 0;
        bool multiple_masters = false;
        for (const auto& camera : cameras)
        {
            // The GevIEEE1588 prefix is for PTP-related calls.
            // Using latched values ensures consistent query results.
            camera.GevIEEE1588DataSetLatch();
            switch(camera.GevIEEE1588StatusLatched())
            {
            case Basler_GigECamera::GevIEEE1588StatusLatched_Master:
            {
                ++num_master;
                break;
            }
            case Basler_GigECamera::GevIEEE1588StatusLatched_Slave:
            {
                // We want to make sure all cameras are synced to the same
                // master clock. For the first slave we encounter, record
                // its parent clock id, and then check if subsequent slaves
                // all have the same parent clock id.
                int64_t master_id = camera.GevIEEE1588ParentClockId();
                if (num_slave == 0)
                {
                    master_clock_id = master_id;
                }
                else if(master_id != master_clock_id)
                {
                    multiple_masters = true;
                }
                ++num_slave;
                break;
            }
            default:
            {
                ++num_init;
            }
            }
        }
        if (num_slave == cameras.size() && !multiple_masters)
        {
            ROS_DEBUG_STREAM("All camera clocks are PTP slaves to master clock " << master_clock_id);
            return true;
        }
        else if(num_slave == cameras.size())
        {
            ROS_DEBUG("All camera clocks are PTP slaves, but multiple masters are present");
        }
        else
        {
            ROS_DEBUG("Camera PTP status: %d initializing, %d masters, %d slaves", num_init, num_master, num_slave);
        }
        if (ros::Time::now() > timeout_time)
        {
            ROS_ERROR("Timed out waiting for camera clocks to become PTP camera slaves. Current status: %d initializing, %d masters, %d slaves", num_init, num_master, num_slave);
            return false;
        }
        r.sleep();
        ros::spinOnce();
    }
}

// Wait for all camera clocks to synchronize to the master clock. Synchronization is reached
// when all clocks have an offset below max_offset_ns for offset_window_s seconds.
// We do not track individual clock synchronization - rather we only consider the
// largest offset from all cameras.
bool waitForPTPClockSync(const std::vector<Pylon::CBaslerGigEInstantCamera>& cameras, int max_offset_ns, int offset_window_s)
{
    ROS_DEBUG("Waiting for clock offsets < %d ns over %d s...", max_offset_ns, offset_window_s);
    bool currently_below = false;
    ros::Time below_start;
    // Store the current offset for each camera
    std::vector<int64_t> current_offsets(cameras.size());
    // Max of the abs of each value in current_offsets
    int64_t current_max_abs_offset = 0;

    ros::Time timeout_time = ros::Time::now() + ros::Duration(ptp_sync_timeout);
    while (ros::ok())
    {
        // Get each camera's offset from master
        current_max_abs_offset = 0;
        for (size_t i = 0; i < cameras.size(); ++i)
        {
            cameras[i].GevIEEE1588DataSetLatch();
            current_offsets[i] = cameras[i].GevIEEE1588OffsetFromMaster();
            current_max_abs_offset = std::max(current_max_abs_offset, std::abs(current_offsets[i]));
        }
        if (current_max_abs_offset < max_offset_ns)
        {
            // The largest clock offset is below our threshold, so start counting time,
            // or check if we've met our time requirement
            if (!currently_below)
            {
                currently_below = true;
                below_start = ros::Time::now();
            }
            else if ((ros::Time::now() - below_start).toSec() >= offset_window_s)
            {
                ROS_DEBUG("All clocks synced");
                return true;
            }
        }
        else
        {
            // The largest clock offset is above our threshold
            currently_below = false;
        }
        if (ros::Time::now() > timeout_time)
        {
            ROS_ERROR("PTP clock synchronization timed out waiting for clocks to reach offsets < %d ns over %d s", max_offset_ns, offset_window_s);
            ROS_ERROR("Current clock offsets:");
            for (size_t i = 0; i < cameras.size(); ++i)
            {
                ROS_ERROR_STREAM(i << ": " << current_offsets[i]);
            }
            return false;
        }
        ros::spinOnce();
    }
}

void enableSyncFreeRun(Pylon::CBaslerGigEInstantCamera& camera, float frame_rate)
{
    // StartTimeLow and High specify a time offset for staggered capture.
    // We set to 0 for all cameras for simultaneous capture.
    // See https://docs.baslerweb.com/synchronous-free-run.html for more info.
    camera.SyncFreeRunTimerTriggerRateAbs.SetValue(frame_rate);
    camera.SyncFreeRunTimerStartTimeLow.SetValue(0);
    camera.SyncFreeRunTimerStartTimeHigh.SetValue(0);
    camera.SyncFreeRunTimerUpdate.Execute();
    camera.SyncFreeRunTimerEnable.SetValue(true);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "basler_synced_cameras");
    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");

    // Load in launchfile params
    int ptp_max_offset_ns;
    pnh.param("ptp_max_offset_ns", ptp_max_offset_ns, 1000);
    int ptp_offset_window_s;
    pnh.param("ptp_window_s", ptp_offset_window_s, 5);
    float frame_rate;
    pnh.param("frame_rate", frame_rate, 5.f);

    // Required to use pylon api
    Pylon::PylonAutoInitTerm auto_init_term;

    // Load in camera information and basler params from yaml
    XmlRpc::XmlRpcValue cameras_yaml;
    pnh.getParam("cameras", cameras_yaml);
    ROS_ASSERT_MSG(cameras_yaml.getType() == XmlRpc::XmlRpcValue::TypeArray, "Bad cameras yaml");
    XmlRpc::XmlRpcValue basler_params;
    pnh.getParam("basler_params", basler_params);
    ROS_ASSERT_MSG(basler_params.getType() == XmlRpc::XmlRpcValue::TypeArray, "Bad basler_params yaml");

    // Load cameras by serial
    std::vector<Pylon::CBaslerGigEInstantCamera> cameras = loadCameras(cameras_yaml);
    if (cameras.empty())
    {
        return 1;
    }

    std::vector<std::shared_ptr<camera_info_manager::CameraInfoManager>> cinfo_managers(cameras.size());
    // Set up camera infos
    for (size_t i = 0; i < cameras.size(); ++i)
    {
        ROS_ASSERT_MSG(cameras_yaml[i].hasMember("name"), "Camera %zu yaml missing name", i);
        ROS_ASSERT_MSG(cameras_yaml[i].hasMember("camera_info_url"), "Camera %zu yaml missing camera_info_url", i);
        ROS_ASSERT_MSG(cameras_yaml[i].hasMember("frame_id"), "Camera %zu yaml missing frame_id", i);
        std::string name = cameras_yaml[i]["name"];
        std::string camera_info_url = cameras_yaml[i]["camera_info_url"];
        std::string frame_id = cameras_yaml[i]["frame_id"];

        std::string camera_info_dir;
        if(pnh.getParam("camera_info_dir", camera_info_dir))
            camera_info_url = camera_info_dir + "/" + camera_info_url;

        ros::NodeHandle camera_nh(nh, name);
        cinfo_managers[i] = std::make_shared<camera_info_manager::CameraInfoManager>(camera_nh, name);
        cinfo_managers[i]->loadCameraInfo(camera_info_url);

        sensor_msgs::CameraInfo::Ptr cinfo(
            new sensor_msgs::CameraInfo(cinfo_managers[i]->getCameraInfo()));
        cameras[i].RegisterImageEventHandler(new Pylon::ImagePublisher(nh, cinfo, frame_id, name + "/", true), Pylon::RegistrationMode_Append, Pylon::Cleanup_Delete);
        cameras[i].RegisterConfiguration(new Pylon::CAcquireContinuousConfiguration, Pylon::RegistrationMode_ReplaceAll, Pylon::Cleanup_Delete);
    }

    // Open cameras so we can change settings, then set basler params and enable PTP
    for (auto& camera : cameras)
    {
        camera.Open();
        handle_basler_parameters(camera);
        // This enables PTP on the camera. (IEEE1588 is the PTP standard)
        camera.GevIEEE1588.SetValue(true);
    }

    ROS_INFO("Waiting for camera PTP clock synchronization...");
    if (!waitForPTPSlave(cameras) ||
        !waitForPTPClockSync(cameras, ptp_max_offset_ns, ptp_offset_window_s))
    {
        return 1;
    }

    ROS_INFO("All %zu cameras synced. Starting image capture in %d seconds...", cameras.size(), startup_sleep_s);
    for (auto& camera : cameras)
    {
        enableSyncFreeRun(camera, frame_rate);
        // With LatestImageOnly, each incoming buffer overwrites previous buffers, so RetrieveResult calls only return the latest frame.
        // There shouldn't be any dropped frames, since we call RetrieveResult continuously in a loop below.
        camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    }

    // For some reason, our networking hardware seems to have trouble handling the sudden increase in traffic upon starting capture,
    // resulting in incomplete buffers for a few seconds. So we sleep here, until the stream stabilizes. LatestImageOnly means that
    // the initial incomplete buffers will be flushed out. The OnImagesSkipped callback will be triggered though.
    ros::Duration(startup_sleep_s).sleep();

    ROS_INFO("Starting image capture from %zu cameras.", cameras.size());

    Pylon::CGrabResultPtr grab_result;
    while (ros::ok())
    {
        for (auto& camera : cameras)
        {
            camera.RetrieveResult(1, grab_result, Pylon::TimeoutHandling_Return);
            ros::spinOnce();
        }
    }


    return 0;
}
