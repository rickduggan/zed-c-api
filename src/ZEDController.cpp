#include "ZEDController.hpp"
#include <algorithm>
#include <cmath>

#include <sl/Camera.hpp>

inline bool checkKPvalidity(sl::float3 &kp, double render_threshold) {
    return (std::isnormal(kp.z) && kp.z > render_threshold);
}

inline bool checkKPvalidity(sl::float4 &kp, double render_threshold) {
    return (std::isnormal(kp.w) && kp.w > render_threshold);
}

ZEDController* ZEDController::instance[MAX_CAMERA_PLUGIN] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr , nullptr, nullptr, nullptr, nullptr , nullptr, nullptr, nullptr, nullptr , nullptr, nullptr, nullptr, nullptr};

ZEDController::ZEDController(int i) {
    input_type = -1;
    cameraOpened = false;
    camera_ID = i;
    activTracking = false;
    fskl = nullptr;
    previousPath = sl::Transform::identity();
	current_detection_model = sl::DETECTION_MODEL::MULTI_CLASS_BOX;
    grab_count_t = 0;
    memset(&camInformations, 0, sizeof (sl::CameraInformation));
    memset(&currentFloorPlane, 0, sizeof (SL_PlaneData));
    memset(&currentPlaneAtHit, 0, sizeof (SL_PlaneData));
    currentPlanes.clear();
}

ZEDController::~ZEDController() {
    cameraOpened = false;
    destroy();
}

void ZEDController::destroy() {
    cameraOpened = false;
    zed.close();
    activTracking = false;
}

void ZEDController::createCamera(bool verbose_file) {
    if (verbose_file) {
        char buffers[256];
        sprintf(buffers, "Create Camera with ID %d\n", camera_ID);
        verbosity = true;
    } else
        verbosity = false;
}

sl::float3 ZEDController::getGravityEstimation() {
    return mesh.getGravityEstimate();
}

int ZEDController::initFromUSB(SL_InitParameters *params, const char* output_file, const char* opt_settings_path, const char* opencv_calib_path) {

    char buffer_verbose[2048];
    if (cameraOpened) {
        sprintf(buffer_verbose, "[initFromUSB] Camera already opened %d = %d return success", params->camera_device_id, camera_ID);
        return 0;
    }
    sprintf(buffer_verbose, "ENTER ZEDController::initFromUSB %d = %d", params->camera_device_id, camera_ID);
	initParams.camera_resolution = (sl::RESOLUTION)params->resolution;
    initParams.input.setFromCameraID(params->camera_device_id);
    initParams.camera_fps = params->camera_fps;
    initParams.depth_minimum_distance = params->depth_minimum_distance;
    initParams.depth_mode = (sl::DEPTH_MODE)params->depth_mode;
    initParams.coordinate_system = (sl::COORDINATE_SYSTEM)params->coordinate_system;
    initParams.coordinate_units = (sl::UNIT)params->coordinate_unit;
    initParams.camera_image_flip = params->camera_image_flip;
    initParams.camera_disable_self_calib = params->camera_disable_self_calib;
    initParams.enable_right_side_measure = params->enable_right_side_measure;
    initParams.depth_stabilization = params->depth_stabilization;
    initParams.sdk_verbose_log_file = output_file;
    initParams.sdk_verbose = params->sdk_verbose;
    initParams.optional_settings_path = opt_settings_path;
    initParams.sensors_required = params->sensors_required;
    initParams.enable_image_enhancement = params->enable_image_enhancement;
    initParams.depth_maximum_distance = params->depth_maximum_distance;
    initParams.optional_opencv_calibration_file = opencv_calib_path;
	initParams.open_timeout_sec = params->open_timeout_sec;
    return open();

}

int ZEDController::initFromSVO(SL_InitParameters *params, const char* path_svo, const char* output_file, const char* opt_settings_path, const char* opencv_calib_path) {

    char buffer_verbose[2048];
    if (cameraOpened) {
        sprintf(buffer_verbose, "[initFromUSB] Camera already opened %d = %d return success", params->camera_device_id, camera_ID);
        return 0;
    }
    sprintf(buffer_verbose, "ENTER ZEDController::initFromSVO %d = %d", params->camera_device_id, camera_ID);


    if (sl::String(path_svo) == NULL) {
        sprintf(buffer_verbose, "Invalid SVO Path");
        return (int) sl::ERROR_CODE::INVALID_SVO_FILE;
    }

    initParams.depth_minimum_distance = params->depth_minimum_distance;
    initParams.depth_mode = (sl::DEPTH_MODE)params->depth_mode;
    initParams.coordinate_system = (sl::COORDINATE_SYSTEM)params->coordinate_system;
    initParams.coordinate_units = (sl::UNIT)params->coordinate_unit;
    initParams.camera_image_flip = params->camera_image_flip;
    initParams.camera_disable_self_calib = params->camera_disable_self_calib;
    initParams.enable_right_side_measure = params->enable_right_side_measure;
    initParams.depth_stabilization = params->depth_stabilization;
    initParams.svo_real_time_mode = params->svo_real_time_mode;
    initParams.input.setFromSVOFile(sl::String(path_svo));
    initParams.sdk_verbose_log_file = output_file;
    initParams.sdk_verbose = params->sdk_verbose;
    initParams.optional_settings_path = opt_settings_path;
    initParams.sensors_required = params->sensors_required;
    initParams.enable_image_enhancement = params->enable_image_enhancement;
    initParams.depth_maximum_distance = params->depth_maximum_distance;
    initParams.optional_opencv_calibration_file = opencv_calib_path;
	initParams.open_timeout_sec = params->open_timeout_sec;
    return open();
}

int ZEDController::initFromStream(SL_InitParameters *params, const char* ip, int port, const char* output_file, const char* opt_settings_path, const char* opencv_calib_path) {

    char buffer_verbose[2048];
    if (cameraOpened) {
        sprintf(buffer_verbose, "[initFromUSB] Camera already opened %d = %d return success", params->camera_device_id, camera_ID);
        return 0;
    }

    if (sl::String(ip) == NULL) {
        sprintf(buffer_verbose, "Invalid IP address");
        return (int) sl::ERROR_CODE::CAMERA_NOT_DETECTED;
    }

    sprintf(buffer_verbose, "ENTER ZEDController::initFromStream %s:%d  = %d", ip, port, camera_ID);
    initParams.camera_resolution = (sl::RESOLUTION)params->resolution;
    initParams.input.setFromStream(sl::String(ip), (unsigned short) port);
    initParams.camera_fps = params->camera_fps;
    initParams.depth_minimum_distance = params->depth_minimum_distance;
    initParams.depth_mode = (sl::DEPTH_MODE)params->depth_mode;
    initParams.coordinate_system = (sl::COORDINATE_SYSTEM)params->coordinate_system;
    initParams.coordinate_units = (sl::UNIT)params->coordinate_unit;
    initParams.camera_image_flip = params->camera_image_flip;
    initParams.camera_disable_self_calib = params->camera_disable_self_calib;
    initParams.enable_right_side_measure = params->enable_right_side_measure;
    initParams.depth_stabilization = params->depth_stabilization;
    initParams.sdk_verbose_log_file = output_file;
    initParams.sdk_verbose = params->sdk_verbose;
    initParams.optional_settings_path = opt_settings_path;
    initParams.sensors_required = params->sensors_required;
    initParams.enable_image_enhancement = params->enable_image_enhancement;
    initParams.depth_maximum_distance = params->depth_maximum_distance;
    initParams.optional_opencv_calibration_file = opencv_calib_path;
	initParams.open_timeout_sec = params->open_timeout_sec;
    return open();
}

int ZEDController::open() {

    globalmutex.lock();
    OpeningErrorCode = zed.open(initParams);
    globalmutex.unlock();

    if (OpeningErrorCode == sl::ERROR_CODE::SUCCESS) {
        width = zed.getCameraInformation().camera_configuration.resolution.width;
        height = zed.getCameraInformation().camera_configuration.resolution.height;
        context = zed.getCUDAContext();
        grab_count_t = 0;
        cameraOpened = true;
        input_type = (int) zed.getCameraInformation().input_type;
        char buffer_verbose[2048];
        sprintf(buffer_verbose, "FUNC ZEDController::init : [SUCCESS Open Camera ID : %d ]", (int) OpeningErrorCode);
    } else {
        width = -1;
        height = -1;
        context = 0;
        input_type = -1;
        cameraOpened = false;
        char buffer_verbose[2048];
        sprintf(buffer_verbose, "FUNC ZEDController::init : [CLOSE Camera ID : %d ]", (int) OpeningErrorCode);
        zed.close();
    }

    return (int) OpeningErrorCode;
}

SL_InitParameters* ZEDController::getInitParameters() {

    SL_InitParameters* initParams = new SL_InitParameters();
    memset(initParams, 0, sizeof (SL_InitParameters));

    sl::InitParameters initParameters = zed.getInitParameters();

    initParams->camera_fps = initParameters.camera_fps;
    initParams->resolution = (SL_RESOLUTION) initParameters.camera_resolution;
    initParams->camera_fps = initParameters.camera_fps;
    initParams->camera_device_id = camera_ID;
    initParams->camera_image_flip = (SL_FLIP_MODE) initParameters.camera_image_flip;
    initParams->camera_disable_self_calib = initParameters.camera_disable_self_calib;

    initParams->enable_right_side_measure = initParameters.enable_right_side_measure;
    initParams->svo_real_time_mode = initParameters.svo_real_time_mode;
    initParams->depth_mode = (SL_DEPTH_MODE) initParameters.depth_mode;
    initParams->depth_stabilization = initParameters.depth_stabilization;
    initParams->depth_maximum_distance = initParameters.depth_maximum_distance;
    initParams->depth_minimum_distance = initParameters.depth_minimum_distance;
    initParams->coordinate_unit = (SL_UNIT) initParameters.coordinate_units;
    initParams->coordinate_system = (SL_COORDINATE_SYSTEM) initParameters.coordinate_system;

    initParams->sdk_gpu_id = initParameters.sdk_gpu_id;
    initParams->sdk_verbose = initParameters.sdk_verbose;

    initParams->sensors_required = initParameters.sensors_required;
    initParams->enable_image_enhancement = initParameters.enable_image_enhancement;

	initParams->open_timeout_sec = initParameters.open_timeout_sec;

    return initParams;
}

SL_RuntimeParameters* ZEDController::getRuntimeParameters() {
    SL_RuntimeParameters* c_runtimeParams = new SL_RuntimeParameters();
    memset(c_runtimeParams, 0, sizeof (SL_RuntimeParameters));

    sl::RuntimeParameters runtimeParams = zed.getRuntimeParameters();
    c_runtimeParams->sensing_mode = (SL_SENSING_MODE) runtimeParams.sensing_mode;
    c_runtimeParams->reference_frame = (SL_REFERENCE_FRAME) runtimeParams.measure3D_reference_frame;
    c_runtimeParams->enable_depth = runtimeParams.enable_depth;
    c_runtimeParams->confidence_threshold = runtimeParams.confidence_threshold;
    c_runtimeParams->texture_confidence_threshold = runtimeParams.texture_confidence_threshold;
    c_runtimeParams->remove_saturated_areas = runtimeParams.remove_saturated_areas;
    return c_runtimeParams;
}

SL_PositionalTrackingParameters* ZEDController::getPositionalTrackingParameters() {
    SL_PositionalTrackingParameters* c_trackingParams = new SL_PositionalTrackingParameters();
    memset(c_trackingParams, 0, sizeof (SL_PositionalTrackingParameters));

    sl::PositionalTrackingParameters trackingParams = zed.getPositionalTrackingParameters();

    //c_trackingParams->area_file_path = trackingParams.area_file_path.c_str();
    c_trackingParams->enable_area_memory = trackingParams.enable_area_memory;
    c_trackingParams->enable_imu_fusion = trackingParams.enable_imu_fusion;
    c_trackingParams->enable_pose_smothing = trackingParams.enable_pose_smoothing;
    sl::Translation t = trackingParams.initial_world_transform.getTranslation();
    SL_Vector3 vec;
    vec.x = t.x;
    vec.y = t.y;
    vec.z = t.z;
    sl::Orientation orien = trackingParams.initial_world_transform.getOrientation();
    SL_Quaternion quat;
    quat.x = orien.x;
    quat.y = orien.y;
    quat.z = orien.z;
    quat.w = orien.w;

    c_trackingParams->initial_world_position = vec;
    c_trackingParams->initial_world_rotation = quat;
    c_trackingParams->set_as_static = trackingParams.set_as_static;
    c_trackingParams->set_floor_as_origin = trackingParams.set_floor_as_origin;
    return c_trackingParams;
}

SL_StreamingParameters* ZEDController::getStreamingParameters() {
	SL_StreamingParameters* c_streamingParams = new SL_StreamingParameters();
	memset(c_streamingParams, 0, sizeof(SL_StreamingParameters));

	sl::StreamingParameters streaming_params = zed.getStreamingParameters();

	c_streamingParams->adaptative_bitrate = streaming_params.adaptative_bitrate;
	c_streamingParams->bitrate = streaming_params.bitrate;
	c_streamingParams->chunk_size = streaming_params.chunk_size;
	c_streamingParams->codec = (SL_STREAMING_CODEC)streaming_params.codec;
	c_streamingParams->gop_size = streaming_params.gop_size;
	c_streamingParams->port = streaming_params.port;
	c_streamingParams->target_framerate = streaming_params.target_framerate;


	return c_streamingParams;
}


void ZEDController::disableTracking(const char *path) {
    if (!isNull()) {
        zed.disablePositionalTracking(sl::String(path));
        activTracking = false;
    }
}

sl::ERROR_CODE ZEDController::enableTracking(const SL_Quaternion *initial_world_rotation, const SL_Vector3 *initial_world_position, bool enable_area_memory, bool enable_pose_smoothing, bool set_floor_as_origin,
        bool set_as_static, bool enable_imu_fusion, const char* area_file_path) {
    if (!isNull()) {
        sl::PositionalTrackingParameters params;
        sl::Transform motion;

        sl::Translation trans;
        trans.tx = initial_world_position->x;
        trans.ty = initial_world_position->y;
        trans.tz = initial_world_position->z;
        motion.setTranslation(trans);

        sl::Orientation orientation;
        orientation.ox = initial_world_rotation->x;
        orientation.oy = initial_world_rotation->y;
        orientation.oz = initial_world_rotation->z;
        orientation.ow = initial_world_rotation->w;
        motion.setOrientation(orientation);

        params.initial_world_transform = motion;
        params.enable_area_memory = enable_area_memory;
        params.enable_pose_smoothing = enable_pose_smoothing;
        params.set_floor_as_origin = set_floor_as_origin;
        params.set_as_static = set_as_static;
        params.enable_imu_fusion = enable_imu_fusion;

        if (area_file_path != nullptr) {
            if (std::string(area_file_path) != "") {
                params.area_file_path = area_file_path;
            }
        }

        sdk_mutex.lock();
        sl::ERROR_CODE tracked = zed.enablePositionalTracking(params);
        if (tracked != sl::ERROR_CODE::SUCCESS)
            activTracking = false;
        else
            activTracking = true;

        sdk_mutex.unlock();
        return tracked;
    }
    return sl::ERROR_CODE::CAMERA_NOT_DETECTED;
}

sl::Translation mult(const sl::Rotation &rot, const sl::Translation &trans) {
    sl::Translation a;
    a.tx = rot.r00 * trans.tx + rot.r01 * trans.ty + rot.r02 * trans.tz;
    a.ty = rot.r10 * trans.tx + rot.r11 * trans.ty + rot.r12 * trans.tz;
    a.tz = rot.r20 * trans.tx + rot.r21 * trans.ty + rot.r22 * trans.tz;
    return a;
}

sl::POSITIONAL_TRACKING_STATE ZEDController::getPosition(SL_Quaternion *quat, SL_Vector3 *vec, SL_Vector3 *offset, SL_Quaternion *offsetRotation, int type) {
    if (isNull())
        return sl::POSITIONAL_TRACKING_STATE::OFF;

    sl::Transform result_pos;
    sl::Pose position;
    sl::POSITIONAL_TRACKING_STATE v;
    if (type == (int) sl::REFERENCE_FRAME::CAMERA) {

        sl::Transform temp_motion;
        temp_motion.setTranslation(sl::Translation(offset->x, offset->y, offset->z));
        sl::Transform temp_motion_inverse = sl::Transform(temp_motion);
        temp_motion_inverse.inverse();

        v = zed.getPosition(position);
        sl::Transform current_position_inverse = position.pose_data;
        current_position_inverse.inverse();

        sl::Transform pose = current_position_inverse * previousPath;
        sl::Rotation rotation(pose.getOrientation());
        rotation.inverse();
        sl::Transform rotMotion;
        rotMotion.setOrientation(rotation);


        sl::Translation r = mult(rotation, pose.getTranslation());
        pose.identity();

        pose.setTranslation(r);
        result_pos = (temp_motion_inverse * pose * temp_motion);

        //result_pos = (temp_motion_inverse*pose*temp_motion);
        sl::Translation trans = result_pos.getTranslation();
        vec->x = -trans.tx;
        vec->y = -trans.ty;
        vec->z = -trans.tz;

        sl::Orientation orientation = result_pos.getOrientation();
        quat->x = orientation.ox;
        quat->y = orientation.oy;
        quat->z = orientation.oz;
        quat->w = orientation.ow;

        previousPath = position.pose_data;
    }


    if (type == (int) sl::REFERENCE_FRAME::WORLD) {
        sl::Transform temp_motion;
        sl::Transform temp_orientation;
        temp_motion.setTranslation(sl::Translation(offset->x, offset->y, offset->z));
        sl::Orientation orientationDiff;
        orientationDiff.ox = offsetRotation->x;
        orientationDiff.oy = offsetRotation->y;
        orientationDiff.oz = offsetRotation->z;
        orientationDiff.ow = offsetRotation->w;

        temp_motion.setOrientation(orientationDiff);
        v = zed.getPosition(position);

        sl::Transform temp = position.pose_data*temp_motion;
        temp_motion.inverse();
        position.pose_data = temp_motion * temp;

        sl::Translation trans = position.getTranslation();
        vec->x = trans.tx;
        vec->y = trans.ty;
        vec->z = trans.tz;

        sl::Orientation orientation = position.getOrientation();
        quat->x = orientation.ox;
        quat->y = orientation.oy;
        quat->z = orientation.oz;
        quat->w = orientation.ow;
    }

    return v;
}

sl::POSITIONAL_TRACKING_STATE ZEDController::getPosition(SL_PoseData *poseData, int reference_frame) {
    if (!isNull()) {
        sl::Pose pose;
        sl::POSITIONAL_TRACKING_STATE state = zed.getPosition(pose, (sl::REFERENCE_FRAME) reference_frame);

		memset(poseData, 0, sizeof(SL_PoseData));
        poseData->pose_confidence = pose.pose_confidence;
        sl::Orientation tempOrientation = pose.pose_data.getOrientation();
        poseData->rotation.x = tempOrientation.x;
        poseData->rotation.y = tempOrientation.y;
        poseData->rotation.z = tempOrientation.z;
        poseData->rotation.w = tempOrientation.w;

		memcpy(&poseData->pose_covariance[0], &pose.pose_covariance[0], sizeof(float) * 36);
		memcpy(&poseData->twist[0], &pose.twist[0], sizeof(float) * 6);
		memcpy(&poseData->twist_covariance[0], &pose.twist_covariance[0], sizeof(float) * 36);

        poseData->translation.x = pose.pose_data.getTranslation().x;
        poseData->translation.y = pose.pose_data.getTranslation().y;
        poseData->translation.z = pose.pose_data.getTranslation().z;

        poseData->timestamp = pose.timestamp;
        poseData->valid = pose.valid;
        return state;
    }
    return sl::POSITIONAL_TRACKING_STATE::OFF;
}

sl::ERROR_CODE ZEDController::getIMUOrientation(SL_Quaternion *quaternion, int time_reference) {
    if (!isNull()) {
        //sl::IMUData tmp_imu_data;
        sl::SensorsData tmp_sensor_data;

        sl::ERROR_CODE err = zed.getSensorsData(tmp_sensor_data, (sl::TIME_REFERENCE)time_reference);
        sl::Orientation imuOrientation = tmp_sensor_data.imu.pose.getOrientation();
        quaternion->x = imuOrientation.x;
        quaternion->y = imuOrientation.y;
        quaternion->z = imuOrientation.z;
        quaternion->w = imuOrientation.w;
        return err;
    }

    return sl::ERROR_CODE::FAILURE;
}

sl::ERROR_CODE ZEDController::getSensorData(SL_SensorData *sensorData, int time_reference) {
    if (!isNull()) {
        sl::SensorsData tmp_sensor_data;
        sl::ERROR_CODE err = zed.getSensorsData(tmp_sensor_data, (sl::TIME_REFERENCE)time_reference);

        sensorData->camera_moving_state = (int) tmp_sensor_data.camera_moving_state;
        sensorData->image_sync_trigger = tmp_sensor_data.image_sync_trigger;

        ///// IMU ///////
        sensorData->imu.is_available = tmp_sensor_data.imu.is_available;
        sensorData->imu.timestamp_ns = tmp_sensor_data.imu.timestamp;

        sensorData->imu.angular_velocity.x = tmp_sensor_data.imu.angular_velocity.x;
        sensorData->imu.angular_velocity.y = tmp_sensor_data.imu.angular_velocity.y;
        sensorData->imu.angular_velocity.z = tmp_sensor_data.imu.angular_velocity.z;
        sensorData->imu.angular_velocity_unc.x = tmp_sensor_data.imu.angular_velocity_uncalibrated.x;
        sensorData->imu.angular_velocity_unc.y = tmp_sensor_data.imu.angular_velocity_uncalibrated.y;
        sensorData->imu.angular_velocity_unc.z = tmp_sensor_data.imu.angular_velocity_uncalibrated.z;

        sensorData->imu.linear_acceleration.x = tmp_sensor_data.imu.linear_acceleration.x;
        sensorData->imu.linear_acceleration.y = tmp_sensor_data.imu.linear_acceleration.y;
        sensorData->imu.linear_acceleration.z = tmp_sensor_data.imu.linear_acceleration.z;
        sensorData->imu.linear_acceleration_unc.x = tmp_sensor_data.imu.linear_acceleration_uncalibrated.x;
        sensorData->imu.linear_acceleration_unc.y = tmp_sensor_data.imu.linear_acceleration_uncalibrated.y;
        sensorData->imu.linear_acceleration_unc.z = tmp_sensor_data.imu.linear_acceleration_uncalibrated.z;

        sensorData->imu.orientation.x = tmp_sensor_data.imu.pose.getOrientation().x;
        sensorData->imu.orientation.y = tmp_sensor_data.imu.pose.getOrientation().y;
        sensorData->imu.orientation.z = tmp_sensor_data.imu.pose.getOrientation().z;
        sensorData->imu.orientation.w = tmp_sensor_data.imu.pose.getOrientation().w;

        for (int i = 0; i < 9; i++) {
            sensorData->imu.angular_velocity_convariance.p[i] = tmp_sensor_data.imu.angular_velocity_covariance.r[i];
            sensorData->imu.linear_acceleration_convariance.p[i] = tmp_sensor_data.imu.linear_acceleration_covariance.r[i];
            sensorData->imu.orientation_covariance.p[i] = tmp_sensor_data.imu.pose_covariance.r[i];
        }

        ///Barometer
        sensorData->barometer.is_available = tmp_sensor_data.barometer.is_available;
        sensorData->barometer.timestamp_ns = tmp_sensor_data.barometer.timestamp;
        sensorData->barometer.pressure = tmp_sensor_data.barometer.pressure;
        sensorData->barometer.relative_altitude = tmp_sensor_data.barometer.relative_altitude;

        ///Magneto
		sensorData->magnetometer.is_available = tmp_sensor_data.magnetometer.is_available;
		sensorData->magnetometer.timestamp_ns = tmp_sensor_data.magnetometer.timestamp;
		sensorData->magnetometer.magnetic_field_unc.x = tmp_sensor_data.magnetometer.magnetic_field_uncalibrated.x;
		sensorData->magnetometer.magnetic_field_unc.y = tmp_sensor_data.magnetometer.magnetic_field_uncalibrated.y;
		sensorData->magnetometer.magnetic_field_unc.z = tmp_sensor_data.magnetometer.magnetic_field_uncalibrated.z;
		sensorData->magnetometer.magnetic_field_c.x = tmp_sensor_data.magnetometer.magnetic_field_calibrated.x;
		sensorData->magnetometer.magnetic_field_c.y = tmp_sensor_data.magnetometer.magnetic_field_calibrated.y;
		sensorData->magnetometer.magnetic_field_c.z = tmp_sensor_data.magnetometer.magnetic_field_calibrated.z;
		sensorData->magnetometer.effective_rate = tmp_sensor_data.magnetometer.effective_rate;
		sensorData->magnetometer.magnetic_heading = tmp_sensor_data.magnetometer.magnetic_heading;
		sensorData->magnetometer.magnetic_heading_state = (SL_HEADING_STATE)tmp_sensor_data.magnetometer.magnetic_heading_state;
		sensorData->magnetometer.magnetic_heading_accuracy = tmp_sensor_data.magnetometer.magnetic_heading_accuracy;

        ///Temperature
        sensorData->temperature.barometer_temp = -100.f;
        sensorData->temperature.imu_temp = -100.f;
        sensorData->temperature.onboard_left_temp = -100.f;
        sensorData->temperature.onboard_right_temp = -100.f;
        if (tmp_sensor_data.temperature.temperature_map.count(sl::SensorsData::TemperatureData::SENSOR_LOCATION::IMU) > 0)
            sensorData->temperature.imu_temp = tmp_sensor_data.temperature.temperature_map[sl::SensorsData::TemperatureData::SENSOR_LOCATION::IMU];
        if (tmp_sensor_data.temperature.temperature_map.count(sl::SensorsData::TemperatureData::SENSOR_LOCATION::BAROMETER) > 0)
            sensorData->temperature.barometer_temp = tmp_sensor_data.temperature.temperature_map[sl::SensorsData::TemperatureData::SENSOR_LOCATION::BAROMETER];
        if (tmp_sensor_data.temperature.temperature_map.count(sl::SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_LEFT) > 0)
            sensorData->temperature.onboard_left_temp = tmp_sensor_data.temperature.temperature_map[sl::SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_LEFT];
        if (tmp_sensor_data.temperature.temperature_map.count(sl::SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_RIGHT) > 0)
            sensorData->temperature.onboard_right_temp = tmp_sensor_data.temperature.temperature_map[sl::SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_RIGHT];

        char buffers[256];
        sprintf(buffers, "internal sensors %f\n", sensorData->temperature.onboard_right_temp);

        return err;
    }

    return sl::ERROR_CODE::FAILURE;
}

sl::ERROR_CODE ZEDController::grab(SL_RuntimeParameters *runtimeParameters) {
    if (!isNull()) {
        sl::ERROR_CODE err = sl::ERROR_CODE::FAILURE;
        runtimeParams.enable_depth = runtimeParameters->enable_depth;
        runtimeParams.sensing_mode = (sl::SENSING_MODE)runtimeParameters->sensing_mode;
        runtimeParams.confidence_threshold = runtimeParameters->confidence_threshold;
        runtimeParams.measure3D_reference_frame = (sl::REFERENCE_FRAME)runtimeParameters->reference_frame;
        runtimeParams.texture_confidence_threshold = runtimeParameters->texture_confidence_threshold;
        runtimeParams.remove_saturated_areas = runtimeParameters->remove_saturated_areas;
        sdk_mutex.lock();
        err = zed.grab(runtimeParams);
        sdk_mutex.unlock();
        if (grab_count_t == 0 && err == sl::ERROR_CODE::SUCCESS) {
            initial_Timestamp = zed.getTimestamp(sl::TIME_REFERENCE::IMAGE);
            grab_count_t++;
        } else if (err == sl::ERROR_CODE::SUCCESS && grab_count_t > 0) {
            unsigned long long current_Timestamp_grab = zed.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds();
            unsigned long long diff_Timestamp_since_start = current_Timestamp_grab - initial_Timestamp.getNanoseconds();
            float camera_fps = zed.getCameraInformation().camera_configuration.fps;
            int number_frames_since_start = diff_Timestamp_since_start / (1000.0 * 1000.0 * (1000.0 / camera_fps));
        }
        return err;
    } else {
        char buffers[256];
        sprintf(buffers, "Grab called but zedcontroller is null %d", camera_ID);
        return sl::ERROR_CODE::CAMERA_NOT_DETECTED;
    }
}

sl::ERROR_CODE ZEDController::enableRecording(const char* path, sl::SVO_COMPRESSION_MODE compressionMode, unsigned int bitrate, int targetFPS, bool transcode) {
    sl::ERROR_CODE err = sl::ERROR_CODE::CAMERA_NOT_DETECTED;
    if (!isNull()) {
        sdk_mutex.lock();
        sl::RecordingParameters rec_params;
        rec_params.video_filename = sl::String(path);
        rec_params.compression_mode = compressionMode;
        rec_params.bitrate = bitrate;
        rec_params.target_framerate = targetFPS;
        rec_params.transcode_streaming_input = transcode;
        err = zed.enableRecording(rec_params);
        sdk_mutex.unlock();
    }
    return err;
}

void ZEDController::disableRecording() {
    if (!isNull()) {
        sdk_mutex.lock();
        zed.disableRecording();
        sdk_mutex.unlock();
    }
}

struct SL_RecordingParameters* ZEDController::getRecordingParameters() {
	SL_RecordingParameters* c_recording_params = new SL_RecordingParameters();
	memset(c_recording_params, 0, sizeof(SL_RecordingParameters));

	sl::RecordingParameters recording_params = zed.getRecordingParameters();

	c_recording_params->bitrate = recording_params.bitrate;
	c_recording_params->compression_mode = (SL_SVO_COMPRESSION_MODE)recording_params.compression_mode;
	c_recording_params->target_framerate = recording_params.target_framerate;
	c_recording_params->transcode_streaming_input = recording_params.transcode_streaming_input;
	
	sl::String video_filename = recording_params.video_filename;

	if (video_filename.size() < 256) {
		memcpy(&c_recording_params->video_filename[0], video_filename, video_filename.size() * sizeof(char));
	}

	return c_recording_params;
}

SL_RecordingStatus* ZEDController::getRecordingStatus() {
	if (!isNull()) {
		SL_RecordingStatus* c_recording_status = new SL_RecordingStatus();
		memset(c_recording_status, 0, sizeof(SL_RecordingStatus));
		
		sl::RecordingStatus recStatus = zed.getRecordingStatus();

		c_recording_status->average_compression_ratio = recStatus.average_compression_ratio;
		c_recording_status->average_compression_time = recStatus.average_compression_time;
		c_recording_status->current_compression_ratio = recStatus.current_compression_ratio;
		c_recording_status->current_compression_time = recStatus.current_compression_time;
		c_recording_status->is_paused = recStatus.is_paused;
		c_recording_status->is_recording = recStatus.is_recording;

		return c_recording_status;
	}
	else
		return nullptr;
}

SL_PlaneData* ZEDController::findFloorPlane(SL_Quaternion *resetQuaternion, SL_Vector3* resetTranslation, SL_Quaternion priorQuaternion, SL_Vector3 priorTranslation) {
    if (!isNull()) {
        sdk_mutex.lock();
        sl::Transform resetTransform;
        sl::Rotation prior_rotation;
        prior_rotation.setOrientation(sl::Orientation(sl::float4(priorQuaternion.x, priorQuaternion.y, priorQuaternion.z, priorQuaternion.w)));
        sl::ERROR_CODE res = zed.findFloorPlane(currentFloorPlaneSDK, resetTransform/*, priorTranslation.y, prior_rotation*/);
        currentFloorPlane.error_code = (int) res;

        if (res == sl::ERROR_CODE::SUCCESS) {
            currentFloorPlane.type = UNITY_PLAN_TYPE::UNITY_PLAN_TYPE_FLOOR;
            currentFloorPlane.extents.x = currentFloorPlaneSDK.getExtents().x;
            currentFloorPlane.extents.y = currentFloorPlaneSDK.getExtents().y;

            currentFloorPlane.plane_center.x = currentFloorPlaneSDK.getCenter().x;
            currentFloorPlane.plane_center.y = currentFloorPlaneSDK.getCenter().y;
            currentFloorPlane.plane_center.z = currentFloorPlaneSDK.getCenter().z;

            currentFloorPlane.plane_normal.x = currentFloorPlaneSDK.getNormal().x;
            currentFloorPlane.plane_normal.y = currentFloorPlaneSDK.getNormal().y;
            currentFloorPlane.plane_normal.z = currentFloorPlaneSDK.getNormal().z;

            sl::Transform plane_pose = currentFloorPlaneSDK.getPose();
            currentFloorPlane.plane_transform_position.x = plane_pose.getTranslation().x;
            currentFloorPlane.plane_transform_position.y = plane_pose.getTranslation().y;
            currentFloorPlane.plane_transform_position.z = plane_pose.getTranslation().z;
            currentFloorPlane.plane_transform_orientation.x = plane_pose.getOrientation().x;
            currentFloorPlane.plane_transform_orientation.y = plane_pose.getOrientation().y;
            currentFloorPlane.plane_transform_orientation.z = plane_pose.getOrientation().z;
            currentFloorPlane.plane_transform_orientation.w = plane_pose.getOrientation().w;

            currentFloorPlane.plane_equation.x = currentFloorPlaneSDK.getPlaneEquation().x;
            currentFloorPlane.plane_equation.y = currentFloorPlaneSDK.getPlaneEquation().y;
            currentFloorPlane.plane_equation.z = currentFloorPlaneSDK.getPlaneEquation().z;
            currentFloorPlane.plane_equation.w = currentFloorPlaneSDK.getPlaneEquation().w;

            std::vector<sl::float3> plane_target_bounds = currentFloorPlaneSDK.getBounds();
            currentFloorPlane.bounds_size = (int) plane_target_bounds.size();
            for (int p = 0; p < std::min((int) plane_target_bounds.size(), 256); p++) {
                currentFloorPlane.bounds[p].x = plane_target_bounds.at(p).x;
                currentFloorPlane.bounds[p].y = plane_target_bounds.at(p).y;
                currentFloorPlane.bounds[p].z = plane_target_bounds.at(p).z;
            }

            resetQuaternion->w = resetTransform.getOrientation().w;
            resetQuaternion->x = resetTransform.getOrientation().x;
            resetQuaternion->y = resetTransform.getOrientation().y;
            resetQuaternion->z = resetTransform.getOrientation().z;

            resetTranslation->x = resetTransform.getTranslation().x;
            resetTranslation->y = resetTransform.getTranslation().y;
            resetTranslation->z = resetTransform.getTranslation().z;

        }

        sdk_mutex.unlock();
    } else {
        memset(&currentFloorPlane, 0, sizeof (SL_PlaneData));
        currentFloorPlane.error_code = (int) sl::ERROR_CODE::FAILURE;
    }

    return &currentFloorPlane;
}

SL_PlaneData* ZEDController::findPlaneAtHit(SL_Vector2 pixels, bool thres) {
    if (!isNull()) {
        sdk_mutex.lock();
        sl::Transform resetTransform;
        sl::uint2 pos;
        pos.x = (unsigned int) pixels.x;
        pos.y = (unsigned int) pixels.y;

        memset(&currentPlaneAtHitSDK, 0, sizeof (sl::Plane));
        sl::ERROR_CODE res = zed.findPlaneAtHit(pos, currentPlaneAtHitSDK);

        currentPlaneAtHit.error_code = (int) res;

        if (res == sl::ERROR_CODE::SUCCESS) {
            switch (currentPlaneAtHitSDK.type) {
                case sl::PLANE_TYPE::HORIZONTAL:
                    currentPlaneAtHit.type = UNITY_PLAN_TYPE::UNITY_PLAN_TYPE_HIT_HORIZONTAL;
                    break;
                case sl::PLANE_TYPE::VERTICAL:
                    currentPlaneAtHit.type = UNITY_PLAN_TYPE::UNITY_PLAN_TYPE_HIT_VERTICAL;
                    break;
                case sl::PLANE_TYPE::UNKNOWN:
                    currentPlaneAtHit.type = UNITY_PLAN_TYPE::UNITY_PLAN_TYPE_HIT_UNKNOWN;
                    break;
                default:
                    currentPlaneAtHit.type = UNITY_PLAN_TYPE::UNITY_PLAN_TYPE_HIT_UNKNOWN;
                    break;
            }

            currentPlaneAtHit.extents.x = currentPlaneAtHitSDK.getExtents().x;
            currentPlaneAtHit.extents.y = currentPlaneAtHitSDK.getExtents().y;

            currentPlaneAtHit.plane_center.x = currentPlaneAtHitSDK.getCenter().x;
            currentPlaneAtHit.plane_center.y = currentPlaneAtHitSDK.getCenter().y;
            currentPlaneAtHit.plane_center.z = currentPlaneAtHitSDK.getCenter().z;

            currentPlaneAtHit.plane_normal.x = currentPlaneAtHitSDK.getNormal().x;
            currentPlaneAtHit.plane_normal.y = currentPlaneAtHitSDK.getNormal().y;
            currentPlaneAtHit.plane_normal.z = currentPlaneAtHitSDK.getNormal().z;

            sl::Transform plane_pose = currentPlaneAtHitSDK.getPose();
            currentPlaneAtHit.plane_transform_position.x = plane_pose.getTranslation().x;
            currentPlaneAtHit.plane_transform_position.y = plane_pose.getTranslation().y;
            currentPlaneAtHit.plane_transform_position.z = plane_pose.getTranslation().z;
            currentPlaneAtHit.plane_transform_orientation.x = plane_pose.getOrientation().x;
            currentPlaneAtHit.plane_transform_orientation.y = plane_pose.getOrientation().y;
            currentPlaneAtHit.plane_transform_orientation.z = plane_pose.getOrientation().z;
            currentPlaneAtHit.plane_transform_orientation.w = plane_pose.getOrientation().w;

            currentPlaneAtHit.plane_equation.x = currentPlaneAtHitSDK.getPlaneEquation().x;
            currentPlaneAtHit.plane_equation.y = currentPlaneAtHitSDK.getPlaneEquation().y;
            currentPlaneAtHit.plane_equation.z = currentPlaneAtHitSDK.getPlaneEquation().z;
            currentPlaneAtHit.plane_equation.w = currentPlaneAtHitSDK.getPlaneEquation().w;

            std::vector<sl::float3> plane_target_bounds = currentPlaneAtHitSDK.getBounds();
            currentPlaneAtHit.bounds_size = (int) plane_target_bounds.size();
            for (int p = 0; p < std::min((int) plane_target_bounds.size(), 256); p++) {
                currentPlaneAtHit.bounds[p].x = plane_target_bounds.at(p).x;
                currentPlaneAtHit.bounds[p].y = plane_target_bounds.at(p).y;
                currentPlaneAtHit.bounds[p].z = plane_target_bounds.at(p).z;
            }


            //Check if area is enough for Unity
            if (thres) {
                if (currentPlaneAtHit.extents.x * currentPlaneAtHit.extents.y < 0.5 || currentPlaneAtHit.extents.x < 0.25 || currentPlaneAtHit.extents.y < 0.25)
                    currentPlaneAtHit.error_code = (int) sl::ERROR_CODE::FAILURE;
            }
        } else {
            memset(&currentPlaneAtHit, 0, sizeof (SL_PlaneData));
            currentPlaneAtHit.error_code = (int) sl::ERROR_CODE::FAILURE;
        }

        sdk_mutex.unlock();
    } else {
        memset(&currentPlaneAtHit, 0, sizeof (SL_PlaneData));
        currentPlaneAtHit.error_code = (int) sl::ERROR_CODE::FAILURE;
    }

    return &currentPlaneAtHit;
}

sl::ERROR_CODE ZEDController::convertCurrentFloorPlaneToChunk(float* vertices, int* triangles, int* numVerticesTot, int* numTrianglesTot) {
    sl::Mesh mesh = currentFloorPlaneSDK.extractMesh();
    if (mesh.vertices.size() > 0 && mesh.triangles.size() > 0) {
        memcpy(vertices, mesh.vertices.data(), sizeof (sl::float3) * int(mesh.vertices.size()));
        memcpy(triangles, mesh.triangles.data(), sizeof (sl::uint3) * int(mesh.triangles.size()));
        *numVerticesTot = 3 * mesh.vertices.size();
        *numTrianglesTot = 3 * int(mesh.triangles.size()); // mesh.triangles.size();
        return sl::ERROR_CODE::SUCCESS;
    } else {
        numVerticesTot = 0;
        numTrianglesTot = 0;
        return sl::ERROR_CODE::FAILURE;
    }
}

sl::ERROR_CODE ZEDController::convertCurrentHitPlaneToChunk(float* vertices, int* triangles, int* numVerticesTot, int* numTrianglesTot) {
    sl::Mesh mesh = currentPlaneAtHitSDK.extractMesh();
    if (mesh.vertices.size() > 0 && mesh.triangles.size() > 15 && currentPlaneAtHit.error_code == (int) sl::ERROR_CODE::SUCCESS) {
        memcpy(vertices, mesh.vertices.data(), sizeof (sl::float3) * int(mesh.vertices.size()));
        memcpy(triangles, mesh.triangles.data(), sizeof (sl::uint3) * int(mesh.triangles.size()));
        *numVerticesTot = 3 * mesh.vertices.size();
        *numTrianglesTot = 3 * int(mesh.triangles.size());
        return sl::ERROR_CODE::SUCCESS;
    } else {
        numVerticesTot = 0;
        numTrianglesTot = 0;
        return sl::ERROR_CODE::FAILURE;
    }
}

sl::POSITIONAL_TRACKING_STATE ZEDController::getPosition(SL_Quaternion *quat, SL_Vector3 *vec, sl::REFERENCE_FRAME mat_type) {
    if (!isNull()) {
        sl::Pose pose;

        memset(quat, 0, sizeof (SL_Quaternion));
        memset(vec, 0, sizeof (SL_Vector3));

        sl::POSITIONAL_TRACKING_STATE v = zed.getPosition(pose, mat_type);
        vec->x = pose.getTranslation().x;
        vec->y = pose.getTranslation().y;
        vec->z = pose.getTranslation().z;

        sl::Orientation orientation = pose.getOrientation();

        quat->x = orientation.x;
        quat->y = orientation.y;
        quat->z = orientation.z;
        quat->w = orientation.w;
        return v;
    }
    return sl::POSITIONAL_TRACKING_STATE::OFF;
}

sl::POSITIONAL_TRACKING_STATE ZEDController::getPoseArray(float* pose, int mat_type) {
    if (!isNull()) {
        sl::Pose p;
        sl::POSITIONAL_TRACKING_STATE v = zed.getPosition(p, (sl::REFERENCE_FRAME)mat_type);
        std::copy(p.pose_data.m, p.pose_data.m + 16, pose);
        return v;
    } else
        return sl::POSITIONAL_TRACKING_STATE::OFF;
}

sl::CameraInformation* ZEDController::getSLCameraInformation() {
    if (!isNull()) {
        //sdk_mutex.lock();
        sl::CameraInformation params = zed.getCameraInformation();
        memcpy(&camInformations, &params, sizeof (sl::CameraInformation));
        //sdk_mutex.unlock();
        return &camInformations;
    }

    return nullptr;
}

SL_CameraParameters convertCamParameters(sl::CameraParameters input) {
	SL_CameraParameters output;
	memset(&output, 0, sizeof(SL_CameraParameters));

	output.cx = input.cx;
	output.cy = input.cy;
	output.fx = input.fx;
	output.fy = input.fy;
	memcpy(&output.disto[0], &input.disto[0], sizeof(double) * 5);

	output.d_fov = input.d_fov;
	output.h_fov = input.h_fov;
	output.v_fov = input.v_fov;
	output.image_size.height = input.image_size.height;
	output.image_size.width = input.image_size.width;

	return output;
}

SL_CalibrationParameters* ZEDController::getCalibrationParameters(bool raw) {
    SL_CalibrationParameters* params = new SL_CalibrationParameters();
    memset(params, 0, sizeof (SL_CalibrationParameters));
    if (!isNull()) {
        sl::CalibrationParameters calib_;
        if (raw)
            calib_ = zed.getCameraInformation().camera_configuration.calibration_parameters_raw;
        else
            calib_ = zed.getCameraInformation().camera_configuration.calibration_parameters;

        params->left_cam = convertCamParameters(calib_.left_cam);
        params->right_cam = convertCamParameters(calib_.right_cam);
        params->rotation.x = calib_.stereo_transform.getOrientation().x;
        params->rotation.y = calib_.stereo_transform.getOrientation().y;
        params->rotation.z = calib_.stereo_transform.getOrientation().z;
        params->rotation.w = calib_.stereo_transform.getOrientation().w;

        params->translation.x = calib_.stereo_transform.getTranslation().x;
        params->translation.y = calib_.stereo_transform.getTranslation().y;
        params->translation.z = calib_.stereo_transform.getTranslation().z;
    }
    return params;
}

SL_SensorParameters convertSensorsParam(sl::SensorParameters input) {
    SL_SensorParameters output;

    output.is_available = input.isAvailable;
    output.noise_density = input.noise_density;
    output.random_walk = input.random_walk;
    output.range.x = input.range.x;
    output.range.y = input.range.y;
    output.resolution = input.resolution;
    output.sampling_rate = input.sampling_rate;
    output.sensor_unit = (SL_SENSORS_UNIT) input.sensor_unit;
    output.type = (SL_SENSOR_TYPE) input.type;

    return output;
}

SL_SensorsConfiguration* ZEDController::getSensorsConfiguration() {
    SL_SensorsConfiguration* params = new SL_SensorsConfiguration();
    memset(params, 0, sizeof (SL_SensorsConfiguration));
    if (!isNull()) {
        sl::SensorsConfiguration sensorConfig;
        sensorConfig = zed.getCameraInformation().sensors_configuration;

        params->firmware_version = sensorConfig.firmware_version;
        params->accelerometer_parameters = convertSensorsParam(sensorConfig.accelerometer_parameters);
        params->gyroscope_parameters = convertSensorsParam(sensorConfig.gyroscope_parameters);
        params->magnetometer_parameters = convertSensorsParam(sensorConfig.magnetometer_parameters);
        params->barometer_parameters = convertSensorsParam(sensorConfig.barometer_parameters);

        params->camera_imu_translation.x = sensorConfig.camera_imu_transform.getTranslation().x;
        params->camera_imu_translation.y = sensorConfig.camera_imu_transform.getTranslation().y;
        params->camera_imu_translation.z = sensorConfig.camera_imu_transform.getTranslation().z;

        params->camera_ium_rotation.x = sensorConfig.camera_imu_transform.getOrientation().x;
        params->camera_ium_rotation.y = sensorConfig.camera_imu_transform.getOrientation().y;
        params->camera_ium_rotation.z = sensorConfig.camera_imu_transform.getOrientation().z;
        params->camera_ium_rotation.w = sensorConfig.camera_imu_transform.getOrientation().w;

		params->ium_magnetometer_translation.x = sensorConfig.imu_magnetometer_transform.getTranslation().x;
		params->ium_magnetometer_translation.y = sensorConfig.imu_magnetometer_transform.getTranslation().y;
		params->ium_magnetometer_translation.z = sensorConfig.imu_magnetometer_transform.getTranslation().z;

		params->ium_magnetometer_rotation.x = sensorConfig.imu_magnetometer_transform.getOrientation().x;
		params->ium_magnetometer_rotation.y = sensorConfig.imu_magnetometer_transform.getOrientation().y;
		params->ium_magnetometer_rotation.z = sensorConfig.imu_magnetometer_transform.getOrientation().z;
		params->ium_magnetometer_rotation.w = sensorConfig.imu_magnetometer_transform.getOrientation().w;
    }
    return params;
}

SL_CameraInformation* ZEDController::getCameraInformation(int width, int height) {
	SL_CameraInformation* params = new SL_CameraInformation();
	memset(params, 0, sizeof(SL_CameraInformation));
	
	sl::CameraInformation sl_camera_info = zed.getCameraInformation(sl::Resolution(width, height));

	SL_CameraConfiguration camera_config;
	camera_config.calibration_parameters = *getCalibrationParameters(false);
	camera_config.calibration_parameters_raw = *getCalibrationParameters(true);
	camera_config.firmware_version = sl_camera_info.camera_configuration.firmware_version;
	camera_config.fps = sl_camera_info.camera_configuration.fps;

	SL_Resolution res;
	res.width = sl_camera_info.camera_configuration.resolution.width;
	res.height = sl_camera_info.camera_configuration.resolution.height;
	camera_config.resolution = res;

	params->camera_configuration = camera_config;
	params->camera_model = (SL_MODEL)sl_camera_info.camera_model;
	params->input_type = (SL_INPUT_TYPE)sl_camera_info.input_type;
	params->serial_number = sl_camera_info.serial_number;
	params->sensors_configuration = *getSensorsConfiguration();

	return params;
}

sl::MODEL ZEDController::getCameraModel() {
    if (!isNull()) {
        return zed.getCameraInformation().camera_model;
    }

    return sl::MODEL::LAST;
}

sl::ERROR_CODE ZEDController::resetTracking(SL_Quaternion rotation, SL_Vector3 translation) {
    if (!isNull()) {
        sl::Transform t;
        t.setOrientation(sl::Orientation(sl::float4(rotation.x, rotation.y, rotation.z, rotation.w)));
        t.setTranslation(sl::Translation(translation.x, translation.y, translation.z));
        return zed.resetPositionalTracking(t);

    }
    return sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
}

sl::ERROR_CODE ZEDController::resetTrackingWithOffset(SL_Quaternion rotation, SL_Vector3 translation, SL_Quaternion OffsetRotation, SL_Vector3 OffsetTranslation) {
    if (!isNull()) {
        sl::Transform t, tout;
        t.setOrientation(sl::Orientation(sl::float4(rotation.x, rotation.y, rotation.z, rotation.w)));
        t.setTranslation(sl::Translation(translation.x, translation.y, translation.z));

        sl::Transform passage;
        passage.setOrientation(sl::Orientation(sl::float4(OffsetRotation.x, OffsetRotation.y, OffsetRotation.z, OffsetRotation.w)));
        passage.setTranslation(sl::Translation(OffsetTranslation.x, OffsetTranslation.y, OffsetTranslation.z));
        tout = t * passage;

        return zed.resetPositionalTracking(tout);
    }
    return sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
}

sl::ERROR_CODE ZEDController::setIMUPriorOrientation(SL_Quaternion rotation) {
    if (!isNull()) {
		sl::Orientation prior_quat = sl::Orientation(sl::float4(rotation.x, rotation.y, rotation.z, rotation.w));
		sl::Translation prior_trans = sl::Translation(0, 0, 0);
		return zed.setIMUPrior(sl::Transform(prior_quat, prior_trans));

    }
    return sl::ERROR_CODE::CAMERA_NOT_INITIALIZED;
}

//////////////////////////////////////////////////
/////////// Spatial Mapping MESH /////////////////
//////////////////////////////////////////////////

sl::ERROR_CODE ZEDController::enableSpatialMapping(struct SL_SpatialMappingParameters mapping_param) {
    if (!isNull()) {
        isTextured = false;
        isMeshUpdated = false;
        areTextureReady = false;
        sl::SpatialMappingParameters params;
        params.map_type = (sl::SpatialMappingParameters::SPATIAL_MAP_TYPE)mapping_param.map_type;
        params.resolution_meter = mapping_param.resolution_meter;
        params.range_meter = mapping_param.range_meter;
        params.use_chunk_only = mapping_param.use_chunk_only;
		params.reverse_vertex_order = mapping_param.reverse_vertex_order;

        if (mapping_param.map_type == SL_SPATIAL_MAP_TYPE_MESH) {
            params.save_texture = mapping_param.save_texture;
            params.max_memory_usage = mapping_param.max_memory_usage;
#ifdef __unix__
            if (params.max_memory_usage > 4095)
                params.max_memory_usage = 4095;
#endif
            this->saveTexture = mapping_param.save_texture;
        } else {
            this->saveTexture = false;
        }

        sl::ERROR_CODE v = sl::ERROR_CODE::FAILURE;
        sdk_mutex.lock();
        try {
            v = zed.enableSpatialMapping(params);
        } catch (std::exception& e) {
        }
        sdk_mutex.unlock();
        return v;
    }
    return sl::ERROR_CODE::CAMERA_NOT_DETECTED;
}

void ZEDController::requestMeshAsync() {
    if (!isNull()) {
        zed.requestSpatialMapAsync();
    }
}

sl::ERROR_CODE ZEDController::updateMesh(int* numVertices, int* numTriangles, int* numUpdatedSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, const int maxSubmesh) {
    if (!isNull() && !isTextured) {
        //LOG(verbosity, "ENTER --> updateMesh");
        if (zed.getSpatialMapRequestStatusAsync() == sl::ERROR_CODE::SUCCESS) {
            //LOG(verbosity, "FUNC updateMesh : call getMeshRequestStatusAsync success");
            sl::ERROR_CODE v = zed.retrieveSpatialMapAsync(mesh);
            if (v != sl::ERROR_CODE::SUCCESS)
                return v;

            //LOG(verbosity, "FUNC updateMesh : call retrieveMeshAsync success : " + std::to_string(mesh.chunks.size()));

            *numUpdatedSubmeshes = 0;
            *numVerticesTot = 0;
            *numTrianglesTot = 0;

            for (int i = 0; i < std::min(maxSubmesh, int(mesh.chunks.size())); i++) {
                if (mesh.chunks[i].has_been_updated) {
                    numVertices[*numUpdatedSubmeshes] = mesh.chunks[i].vertices.size();
                    *numVerticesTot += mesh.chunks[i].vertices.size();
                    *numTrianglesTot += mesh.chunks[i].triangles.size();
                    numTriangles[*numUpdatedSubmeshes] = mesh.chunks[i].triangles.size();

                    updatedIndices[*numUpdatedSubmeshes] = i;
                    (*numUpdatedSubmeshes)++;
                }
            }

            //LOG(verbosity, "FUNC updateMesh : fill vertices " + std::to_string(numUpdatedSubmeshes) + " " + std::to_string(numVerticesTot));
            isMeshUpdated = true;
            return v;
        }
        return sl::ERROR_CODE::FAILURE;
    }
    return sl::ERROR_CODE::CAMERA_NOT_DETECTED;
}

sl::ERROR_CODE ZEDController::updateChunks(int* numVertices, int* numTriangles, int* numUpdatedSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, const int maxSubmesh) {
    if (!isNull() && !isTextured) {
        if (zed.getSpatialMapRequestStatusAsync() == sl::ERROR_CODE::SUCCESS) {
            sl::ERROR_CODE v = zed.retrieveSpatialMapAsync(mesh);
            if (v != sl::ERROR_CODE::SUCCESS)
                return v;

            *numUpdatedSubmeshes = 0;
            *numVerticesTot = 0;
            *numTrianglesTot = 0;

            for (int i = 0; i < std::min(maxSubmesh, int(mesh.chunks.size())); i++) {
				//if (mesh.chunks[i].has_been_updated) 
				{
					numVertices[*numUpdatedSubmeshes] = mesh.chunks[i].vertices.size();
					*numVerticesTot += mesh.chunks[i].vertices.size();
					*numTrianglesTot += mesh.chunks[i].triangles.size();
					numTriangles[*numUpdatedSubmeshes] = mesh.chunks[i].triangles.size();

					updatedIndices[*numUpdatedSubmeshes] = i;
					(*numUpdatedSubmeshes)++;
				}
            }

            isMeshUpdated = true;
            return v;
        }
        return sl::ERROR_CODE::FAILURE;
    }
    return sl::ERROR_CODE::CAMERA_NOT_DETECTED;
}

sl::ERROR_CODE ZEDController::retrieveChunks(const int maxSubmesh, float* vertices, int* triangles) {
    if (!isNull() && !isTextured) {
        if (isMeshUpdated) {
            int offsetVertices = 0, offsetTriangles = 0, offsetUvs = 0;

            for (int i = 0; i < std::min(maxSubmesh, int(mesh.chunks.size())); i++) {
                //if (mesh.chunks[i].has_been_updated)
                {
                    memcpy(&vertices[offsetVertices], mesh.chunks[i].vertices.data(), sizeof (sl::float3) * int(mesh.chunks[i].vertices.size()));
                    memcpy(&triangles[offsetTriangles], mesh.chunks[i].triangles.data(), sizeof (sl::uint3) * int(mesh.chunks[i].triangles.size()));
                    offsetVertices += int(3 * mesh.chunks[i].vertices.size());
                    offsetTriangles += int(3 * mesh.chunks[i].triangles.size());
                }
            }

            return sl::ERROR_CODE::SUCCESS;
        }
    }
    return sl::ERROR_CODE::CAMERA_NOT_DETECTED;
}

sl::ERROR_CODE ZEDController::retrieveMesh(float* vertices, int* triangles, const int maxSubmesh, float* uvs, unsigned char* texturePtr) {
    if (!isNull() && !isTextured) {
        if (isMeshUpdated) {
            int offsetVertices = 0, offsetTriangles = 0, offsetUvs = 0;

            bool isTextureCalled = areTextureReady && uvs != nullptr && texturePtr != nullptr;
            cudaGraphicsResource_t pcuImageRes = nullptr;
            if (isTextureCalled) {

				texturePtr = mesh.texture.getPtr<sl::uchar1>(sl::MEM::CPU);
#if 0
#ifdef _WIN32
                cudaGraphicsD3D11RegisterResource(&pcuImageRes, (ID3D11Texture2D*) texturePtr, cudaGraphicsMapFlags::cudaGraphicsMapFlagsNone);
                cudaError_t error = cudaGraphicsMapResources(1, &pcuImageRes, 0);
                cudaArray_t ArrIm;

                error = cudaGraphicsSubResourceGetMappedArray(&ArrIm, pcuImageRes, 0, 0);
                sl::Mat texture = mesh.texture;

                error = cudaMemcpy2DToArray(ArrIm, 0, 0,
                        texture.getPtr<sl::uchar1>(sl::MEM::CPU), texture.getStepBytes(sl::MEM::CPU),
                        texture.getWidthBytes(),
                        texture.getHeight(), cudaMemcpyHostToDevice);
                cudaGraphicsUnmapResources(1, &pcuImageRes, 0);
                cudaGraphicsUnregisterResource(pcuImageRes);

#elif __unix__
                cudaGraphicsGLRegisterImage(&pcuImageRes, texturePtr, GL_TEXTURE_2D, cudaGraphicsRegisterFlagsNone);
                cudaError_t error = cudaGraphicsMapResources(1, &pcuImageRes, 0);
                cudaArray_t ArrIm;

                error = cudaGraphicsSubResourceGetMappedArray(&ArrIm, pcuImageRes, 0, 0);
                sl::Mat texture = mesh.texture;

                error = cudaMemcpy2DToArray(ArrIm, 0, 0,
                        texture.getPtr<sl::uchar1>(sl::MEM::CPU), texture.getStepBytes(sl::MEM::CPU),
                        texture.getWidthBytes(),
                        texture.getHeight(), cudaMemcpyHostToDevice);
                cudaGraphicsUnmapResources(1, &pcuImageRes, 0);
                cudaGraphicsUnregisterResource(pcuImageRes);

#endif
#endif
            }
            int startIndexUV = 0;
            for (int i = 0; i < std::min(maxSubmesh, int(mesh.chunks.size())); i++) {
                if (mesh.chunks[i].has_been_updated) {
                    memcpy(&vertices[offsetVertices], mesh.chunks[i].vertices.data(), sizeof (sl::float3) * int(mesh.chunks[i].vertices.size()));
                    memcpy(&triangles[offsetTriangles], mesh.chunks[i].triangles.data(), sizeof (sl::uint3) * int(mesh.chunks[i].triangles.size()));
                    offsetVertices += int(3 * mesh.chunks[i].vertices.size());
                    offsetTriangles += int(3 * mesh.chunks[i].triangles.size());
                    if (isTextureCalled) {
                        memcpy(&uvs[offsetUvs], &mesh.uv.data()[startIndexUV], sizeof (sl::float2) * int(mesh.chunks[i].uv.size()));
                        offsetUvs += int(2 * mesh.chunks[i].uv.size());
                        startIndexUV += int(mesh.chunks[i].uv.size());
                    }
                }
            }
            if (areTextureReady && uvs != nullptr && texturePtr != nullptr) {
                isTextured = true;
            }
            return sl::ERROR_CODE::SUCCESS;
        }
    }
    return sl::ERROR_CODE::CAMERA_NOT_DETECTED;
}

bool ZEDController::filterMesh(sl::MeshFilterParameters::MESH_FILTER filterParams, int* numVertices, int* numTriangles, int* numUpdatedSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, const int maxSubmesh) {
    if (!isNull() && !isTextured) {
        if (mesh.filter(sl::MeshFilterParameters(filterParams), false)) {

            *numUpdatedSubmeshes = 0;
            *numVerticesTot = 0;
            *numTrianglesTot = 0;

            for (int i = 0; i < std::min(maxSubmesh, int(mesh.chunks.size())); i++) {
                mesh.chunks[i].has_been_updated = true;
                numVertices[*numUpdatedSubmeshes] = mesh.chunks[i].vertices.size();
                *numVerticesTot += mesh.chunks[i].vertices.size();
                *numTrianglesTot += mesh.chunks[i].triangles.size();
                numTriangles[*numUpdatedSubmeshes] = mesh.chunks[i].triangles.size();
                updatedIndices[*numUpdatedSubmeshes] = i;
                (*numUpdatedSubmeshes)++;
            }
            isMeshUpdated = true;
            return true;
        }
        return false;
    }
    return false;
}

sl::ERROR_CODE ZEDController::updateFusedPointCloud(int* numPointsTot) {
    if (!isNull()) {
        //LOG(verbosity, "ENTER updateFusedPointCloud");
        if (zed.getSpatialMapRequestStatusAsync() == sl::ERROR_CODE::SUCCESS) {
            sl::ERROR_CODE v = zed.retrieveSpatialMapAsync(pointCloudFused);
            if (v != sl::ERROR_CODE::SUCCESS)
                return v;
            //zed.extractWholeSpatialMap(pointCloudFused);
            *numPointsTot = pointCloudFused.vertices.size();
            return v;
        }
        return sl::ERROR_CODE::FAILURE;
    }
    return sl::ERROR_CODE::CAMERA_NOT_DETECTED;
}

sl::ERROR_CODE ZEDController::retrieveFusedPointCloud(float* vertices) {
    if (!isNull()) {
        //LOG(verbosity, "ENTER retrieveFusedPointCloud");
        int numPointsTot = pointCloudFused.vertices.size();
        if (numPointsTot > 0) {
            memcpy(&vertices[0], pointCloudFused.vertices.data(), sizeof (sl::float4) * numPointsTot);
            return sl::ERROR_CODE::SUCCESS;
        }
    }
    return sl::ERROR_CODE::CAMERA_NOT_DETECTED;
}

void ZEDController::disableSpatialMapping() {
    if (!isNull()) {
        isTextured = false;
        isMeshUpdated = false;
        areTextureReady = false;

        zed.disableSpatialMapping();
    }

}

SL_SpatialMappingParameters* ZEDController::getSpatialMappingParameters() {
	SL_SpatialMappingParameters* c_mappingParams = new SL_SpatialMappingParameters();
	memset(c_mappingParams, 0, sizeof(SL_SpatialMappingParameters));

	sl::SpatialMappingParameters mappingParams = zed.getSpatialMappingParameters();

	c_mappingParams->map_type = (SL_SPATIAL_MAP_TYPE)mappingParams.map_type;
	c_mappingParams->max_memory_usage = mappingParams.max_memory_usage;
	c_mappingParams->range_meter = mappingParams.range_meter;
	c_mappingParams->resolution_meter = mappingParams.resolution_meter;
	c_mappingParams->reverse_vertex_order = mappingParams.reverse_vertex_order;
	c_mappingParams->save_texture = mappingParams.save_texture;
	c_mappingParams->use_chunk_only = mappingParams.use_chunk_only;

	return c_mappingParams;
}

void ZEDController::mergeChunks(int numberFaces, int* numVertices, int* numTriangles, int* numUpdatedSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, const int maxSubmesh) {
    if (!isNull()) {
        mesh.mergeChunks(numberFaces);

        *numUpdatedSubmeshes = 0;
        *numVerticesTot = 0;
        *numTrianglesTot = 0;
		
        for (int i = 0; i < std::min(maxSubmesh, int(mesh.chunks.size())); i++) {
            mesh.chunks[i].has_been_updated = true;
            numVertices[*numUpdatedSubmeshes] = mesh.chunks[i].vertices.size();
            *numVerticesTot += mesh.chunks[i].vertices.size();
            *numTrianglesTot += mesh.chunks[i].triangles.size();
            numTriangles[*numUpdatedSubmeshes] = mesh.chunks[i].triangles.size();
            updatedIndices[*numUpdatedSubmeshes] = i;
            (*numUpdatedSubmeshes)++;
        }
    }
}

sl::ERROR_CODE ZEDController::extractWholeSpatialMap() {

    sl::ERROR_CODE v = sl::ERROR_CODE::FAILURE;
    if (!isNull()) {
        v = zed.extractWholeSpatialMap(mesh);
    }
    return v;
}

bool ZEDController::saveMesh(const char* filename, sl::MESH_FILE_FORMAT format) {
    if (!isNull()) {
        return mesh.save(filename, format);
    }
    return false;
}

bool ZEDController::savePointCloud(const char* filename, sl::MESH_FILE_FORMAT format) {
    if (!isNull()) {
        return pointCloudFused.save(filename, format);
    }
    return false;
}

bool ZEDController::loadMesh(const char* filename, int* numVertices, int* numTriangles, int* numUpdatedSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, const int maxSubmesh, int* texturesSize) {
    if (mesh.load(filename, false)) {

		*numUpdatedSubmeshes = 0;
        *numVerticesTot = 0;
        *numTrianglesTot = 0;

        if (mesh.texture.isInit() && texturesSize != nullptr) {
            texturesSize[1] = mesh.texture.getHeight();
            texturesSize[0] = mesh.texture.getWidth();
        }

        for (int i = 0; i < std::min(maxSubmesh, int(mesh.chunks.size())); i++) {
            numVertices[i] = mesh.chunks[i].vertices.size();
            *numVerticesTot += mesh.chunks[i].vertices.size();
            *numTrianglesTot += mesh.chunks[i].triangles.size();
            numTriangles[i] = mesh.chunks[i].triangles.size();
            updatedIndices[i] = i;

            (*numUpdatedSubmeshes)++;
            mesh.chunks[i].has_been_updated = true;
        }
        isMeshUpdated = true;

        if (mesh.texture.isInit() && texturesSize != nullptr) {
            areTextureReady = true;
        } else {
            texturesSize[0] = -1;
        }
        return true;
    } else {
        return false;
    }
}

bool ZEDController::applyTexture(int* numVertices, int* numTriangles, int* numUpdatedSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, int* texturesSize, const int maxSubmesh) {

    if (mesh.applyTexture(sl::MESH_TEXTURE_FORMAT::RGBA)) {

        *numUpdatedSubmeshes = 0;
        *numVerticesTot = 0;
        *numTrianglesTot = 0;

        for (int i = 0; i < std::min(maxSubmesh, int(mesh.chunks.size())); i++) {
            numVertices[*numUpdatedSubmeshes] = mesh.chunks[i].vertices.size();
            *numVerticesTot += mesh.chunks[i].vertices.size();
            *numTrianglesTot += mesh.chunks[i].triangles.size();
            numTriangles[*numUpdatedSubmeshes] = mesh.chunks[i].triangles.size();
            updatedIndices[*numUpdatedSubmeshes] = i;
            (*numUpdatedSubmeshes)++;

            mesh.chunks[i].has_been_updated = true;
        }
        texturesSize[0] = mesh.texture.getWidth();
        texturesSize[1] = mesh.texture.getHeight();
        isMeshUpdated = true;
        areTextureReady = true;
        return true;
    }

    return false;

}

sl::ERROR_CODE ZEDController::enableStreaming(sl::STREAMING_CODEC codec, unsigned int bitrate, unsigned short port, int gopSize, bool adaptativeBitrate, int chunk_size, int target_framerate) {
    if (!isNull()) {
        sl::ERROR_CODE v;
        sdk_mutex.lock();
        sl::StreamingParameters params;
        params.codec = codec;
        params.bitrate = bitrate;
        params.gop_size = gopSize;
        params.port = port;
        params.adaptative_bitrate = adaptativeBitrate;
        params.chunk_size = chunk_size;
        params.target_framerate = target_framerate;
        v = zed.enableStreaming(params);
        sdk_mutex.unlock();
        return v;
    }
    return sl::ERROR_CODE::FAILURE;
}

bool ZEDController::isStreamingEnabled() {
    if (!isNull()) {
        return zed.isStreamingEnabled();
    }
    return false;
}

void ZEDController::disableStreaming() {
    if (!isNull()) {
        zed.disableStreaming();
    }
    return;
}

sl::ERROR_CODE ZEDController::saveCurrentImage(sl::VIEW view, const char* filename) {
    sl::ERROR_CODE v = sl::ERROR_CODE::FAILURE;
    if (!isNull()) {
        sdk_mutex.lock();
        sl::Mat image;
        v = zed.retrieveImage(image, view);
        if (v == sl::ERROR_CODE::SUCCESS)
            v = image.write(sl::String(filename));
        sdk_mutex.unlock();
    }
    return v;
}

sl::ERROR_CODE ZEDController::saveCurrentDepth(int side, const char* filename) {
    sl::ERROR_CODE v = sl::ERROR_CODE::FAILURE;
    if (!isNull()) {
        sdk_mutex.lock();
        sl::Mat meas;
        if (side == 0)
            v = zed.retrieveMeasure(meas, sl::MEASURE::DEPTH);
        else
            v = zed.retrieveMeasure(meas, sl::MEASURE::DEPTH_RIGHT);
        if (v == sl::ERROR_CODE::SUCCESS)
            v = meas.write(sl::String(filename));
        sdk_mutex.unlock();
    }
    return v;
}

sl::ERROR_CODE ZEDController::saveCurrentPointCloud(int side, const char* filename) {
    sl::ERROR_CODE v = sl::ERROR_CODE::FAILURE;
    if (!isNull()) {
        sdk_mutex.lock();
        sl::Mat meas;
        if (side == 0)
            v = zed.retrieveMeasure(meas, sl::MEASURE::XYZRGBA);
        else
            v = zed.retrieveMeasure(meas, sl::MEASURE::XYZRGBA_RIGHT);
        if (v == sl::ERROR_CODE::SUCCESS)
            v = meas.write(sl::String(filename));
        sdk_mutex.unlock();
    }
    return v;
}

int ZEDController::getInputType() {
    return input_type;
}


#if WITH_OBJECT_DETECTION

sl::ERROR_CODE ZEDController::enableObjectDetection(SL_ObjectDetectionParameters* obj_params) {

    if (!isNull()) {
        sl::ERROR_CODE v;
        sl::ObjectDetectionParameters params;
        params.image_sync = obj_params->image_sync;
        params.enable_tracking = obj_params->enable_tracking;
        params.enable_mask_output = obj_params->enable_mask_output;
        params.enable_body_fitting = obj_params->enable_body_fitting;
		params.body_format = (sl::BODY_FORMAT)obj_params->body_format;
        params.detection_model = (sl::DETECTION_MODEL)obj_params->model;
		params.filtering_mode = (sl::OBJECT_FILTERING_MODE)obj_params->filtering_mode;
        if (obj_params->max_range > 0)
            params.max_range = obj_params->max_range;

        sl::BatchParameters batch_parameters;
        batch_parameters.enable = obj_params->batch_parameters.enable;
        if (obj_params->batch_parameters.id_retention_time > 0) {
            batch_parameters.id_retention_time = obj_params->batch_parameters.id_retention_time;
        }
        if (obj_params->batch_parameters.latency > 0) {
            batch_parameters.latency = obj_params->batch_parameters.latency;
        }
        params.batch_parameters = batch_parameters;
        current_detection_model = params.detection_model;
        sdk_mutex.lock();
        v = zed.enableObjectDetection(params);
        sdk_mutex.unlock();

        isTrajectoriesUpdated = false;
        return v;
    }
    return sl::ERROR_CODE::CAMERA_NOT_DETECTED;
}

SL_ObjectDetectionParameters* ZEDController::getObjectDetectionParameters() {
	SL_ObjectDetectionParameters* c_odParams = new SL_ObjectDetectionParameters();
	memset(c_odParams, 0, sizeof(SL_ObjectDetectionParameters));

	sl::ObjectDetectionParameters odParams = zed.getObjectDetectionParameters();

	sl::BatchParameters batchParams = odParams.batch_parameters;

	c_odParams->batch_parameters.enable = batchParams.enable;
	c_odParams->batch_parameters.id_retention_time = batchParams.id_retention_time;
	c_odParams->batch_parameters.latency = batchParams.latency;

	c_odParams->body_format = (SL_BODY_FORMAT)odParams.body_format;
	c_odParams->enable_body_fitting = odParams.enable_body_fitting;
	c_odParams->enable_mask_output = odParams.enable_mask_output;
	c_odParams->enable_tracking = odParams.enable_tracking;
	c_odParams->filtering_mode = (SL_OBJECT_FILTERING_MODE)odParams.filtering_mode;
	c_odParams->image_sync = odParams.image_sync;
	c_odParams->max_range = odParams.max_range;
	c_odParams->model = (SL_DETECTION_MODEL)odParams.detection_model;

	return c_odParams;
}

void ZEDController::pauseObjectDetection(bool status) {
    if (!isNull()) {
        sdk_mutex.lock();
        zed.pauseObjectDetection(status);
        sdk_mutex.unlock();
    }
}

void ZEDController::disableObjectDetection() {
    if (!isNull()) {
        sdk_mutex.lock();
        zed.disableObjectDetection();
        sdk_mutex.unlock();
        isTrajectoriesUpdated = false;
    }
}

sl::ERROR_CODE ZEDController::ingestCustomBoxObjectData(int nb_objects, SL_CustomBoxObjectData* objects_in)
{
	if (!isNull()) {
		std::vector<sl::CustomBoxObjectData> objs;
		for (int i = 0; i < nb_objects; i++) 
		{			
			SL_CustomBoxObjectData obj = objects_in[i];
			sl::CustomBoxObjectData tmp;

			tmp.unique_object_id = sl::String(obj.unique_object_id);
			tmp.label = obj.label;
			tmp.probability = obj.probability;
			tmp.is_grounded = obj.is_grounded;
			for (int l = 0; l < 4; l++) {
				sl::uint2 value;
				value.x = obj.bounding_box_2d[l].x;
				value.y = obj.bounding_box_2d[l].y;

				tmp.bounding_box_2d.push_back(value);
			}
			objs.push_back(tmp);
		}
		sl::ERROR_CODE err = zed.ingestCustomBoxObjects(objs);
		return err;
	}
	return sl::ERROR_CODE::CAMERA_NOT_DETECTED;
}

sl::ERROR_CODE ZEDController::retrieveObjectDetectionData(SL_ObjectDetectionRuntimeParameters* _objruntimeparams, SL_Objects* data) {
    memset(data, 0, sizeof (SL_Objects));

    if (!isNull()) {
        sl::Objects objects;
        cuCtxSetCurrent(zed.getCUDAContext());
        sl::ObjectDetectionRuntimeParameters runtime_params;
        runtime_params.detection_confidence_threshold = _objruntimeparams->detection_confidence_threshold;

        runtime_params.object_class_filter = std::vector<sl::OBJECT_CLASS>{};
        runtime_params.object_class_detection_confidence_threshold = std::map<sl::OBJECT_CLASS, float>{};

        for (int k = 0; k < (int) sl::OBJECT_CLASS::LAST; k++) {
            if (_objruntimeparams->object_class_filter[k]) {
                runtime_params.object_class_filter.push_back(static_cast<sl::OBJECT_CLASS> (k));
            }

            if (_objruntimeparams->object_confidence_threshold[k]) {
                runtime_params.object_class_detection_confidence_threshold.insert({static_cast<sl::OBJECT_CLASS> (k), _objruntimeparams->object_confidence_threshold[k]});
            }
        }
        sl::ERROR_CODE v = zed.retrieveObjects(objects, runtime_params);

        if (v == sl::ERROR_CODE::SUCCESS) {
            //LOG(verbosity, "retrieve objects :" + std::to_string(objects.object_list.size()));
            data->is_new = objects.is_new;
            data->is_tracked = objects.is_tracked;
            data->detection_model = (SL_DETECTION_MODEL) current_detection_model;
            int size_objects = objects.object_list.size();
            data->image_ts = objects.timestamp;
            data->nb_object = size_objects;

            int count = 0;

            for (auto &p : objects.object_list) {
                if (count < MAX_NUMBER_OBJECT) {
                    //data->data_object[count].valid = true;
                    data->object_list[count].label = (SL_OBJECT_CLASS) p.label;
                    data->object_list[count].sublabel = (SL_OBJECT_SUBCLASS) p.sublabel;
                    data->object_list[count].tracking_state = (SL_OBJECT_TRACKING_STATE) p.tracking_state;
                    data->object_list[count].action_state = (SL_OBJECT_ACTION_STATE) p.action_state;
                    data->object_list[count].id = p.id;
                    data->object_list[count].confidence = p.confidence;
					data->object_list[count].raw_label = p.raw_label;

					memcpy(data->object_list[count].unique_object_id, p.unique_object_id, 37 * sizeof(char));


                    for (int k = 0; k < 6; k++)
                        data->object_list[count].position_covariance[k] = p.position_covariance[k];

					data->object_list[count].mask = (int*)(new sl::Mat(p.mask));

                    for (int l = 0; l < 4; l++) {
                        data->object_list[count].bounding_box_2d[l].x = (float) p.bounding_box_2d.at(l).x;
                        data->object_list[count].bounding_box_2d[l].y = (float) p.bounding_box_2d.at(l).y;
                    }

                    // World data
                    data->object_list[count].position.x = p.position.x;
                    data->object_list[count].position.y = p.position.y;
                    data->object_list[count].position.z = p.position.z;

                    data->object_list[count].velocity.x = p.velocity.x;
                    data->object_list[count].velocity.y = p.velocity.y;
                    data->object_list[count].velocity.z = p.velocity.z;

                    // 3D Bounding box in world frame
                    for (int m = 0; m < 8; m++) {
                        if (m < p.bounding_box.size()) {
                            data->object_list[count].bounding_box[m].x = p.bounding_box.at(m).x;
                            data->object_list[count].bounding_box[m].y = p.bounding_box.at(m).y;
                            data->object_list[count].bounding_box[m].z = p.bounding_box.at(m).z;
                        }
                    }

                    // if skeleton
                    if (data->detection_model == SL_DETECTION_MODEL_HUMAN_BODY_FAST || data->detection_model == SL_DETECTION_MODEL_HUMAN_BODY_ACCURATE || data->detection_model == SL_DETECTION_MODEL_HUMAN_BODY_MEDIUM) {
                        for (int i = 0; i < (int)p.keypoint.size(); i++) {
                            data->object_list[count].keypoint_2d[i].x = p.keypoint_2d.at(i).x;
                            data->object_list[count].keypoint_2d[i].y = p.keypoint_2d.at(i).y;

                            data->object_list[count].keypoint[i].x = p.keypoint.at(i).x;
                            data->object_list[count].keypoint[i].y = p.keypoint.at(i).y;
                            data->object_list[count].keypoint[i].z = p.keypoint.at(i).z;
                            data->object_list[count].keypoint_confidence[i] = p.keypoint_confidence.at(i);
                        }

                        data->object_list[count].head_position.x = p.head_position.x;
                        data->object_list[count].head_position.y = p.head_position.y;
                        data->object_list[count].head_position.z = p.head_position.z;

                        for (int m = 0; m < 8; m++) {
                            if (m < p.head_bounding_box.size()) {
                                data->object_list[count].head_bounding_box[m].x = p.head_bounding_box.at(m).x;
                                data->object_list[count].head_bounding_box[m].y = p.head_bounding_box.at(m).y;
                                data->object_list[count].head_bounding_box[m].z = p.head_bounding_box.at(m).z;
                            }
                        }

						data->object_list[count].global_root_orientation.x = p.global_root_orientation.x;
						data->object_list[count].global_root_orientation.y = p.global_root_orientation.y;
						data->object_list[count].global_root_orientation.z = p.global_root_orientation.z;
						data->object_list[count].global_root_orientation.w = p.global_root_orientation.w;

						for (int i = 0; i < p.local_orientation_per_joint.size(); i++) { // 18 or 34

							data->object_list[count].local_orientation_per_joint[i].x = p.local_orientation_per_joint[i].x;
							data->object_list[count].local_orientation_per_joint[i].y = p.local_orientation_per_joint[i].y;
							data->object_list[count].local_orientation_per_joint[i].z = p.local_orientation_per_joint[i].z;
							data->object_list[count].local_orientation_per_joint[i].w = p.local_orientation_per_joint[i].w;

							data->object_list[count].local_position_per_joint[i].x = p.local_position_per_joint[i].x;
							data->object_list[count].local_position_per_joint[i].y = p.local_position_per_joint[i].y;
							data->object_list[count].local_position_per_joint[i].z = p.local_position_per_joint[i].z;
						}
                    }
                    count++;
                }
            }
        }
        return v;
    }
    return sl::ERROR_CODE::CAMERA_NOT_DETECTED;
}

sl::ERROR_CODE ZEDController::updateObjectsBatch(int* nb_batches) {
    if (!isNull()) {
        cuCtxSetCurrent(zed.getCUDAContext());
        sl::ERROR_CODE v = zed.getObjectsBatch(trajectories);
        if (v != sl::ERROR_CODE::SUCCESS)
            return v;

        *nb_batches = trajectories.size();
        isTrajectoriesUpdated = true;

        return v;
    }
    return sl::ERROR_CODE::CAMERA_NOT_DETECTED;
}

sl::ERROR_CODE ZEDController::getObjectsBatchData(int index, struct SL_ObjectsBatch *objs) {

	memset(objs, 0, sizeof(SL_ObjectsBatch));

    if (!isNull()) {
        if (isTrajectoriesUpdated) {
            if (index < trajectories.size() && index < MAX_NUMBER_OBJECT) {

                sl::ObjectsBatch obj_batch = trajectories[index];
				objs->nb_data = obj_batch.positions.size();
				objs->id = obj_batch.id;

				objs->label = (SL_OBJECT_CLASS)obj_batch.label;
				objs->sublabel = (SL_OBJECT_SUBCLASS)obj_batch.sublabel;
				objs->tracking_state = (SL_OBJECT_TRACKING_STATE)obj_batch.tracking_state;

                for (int i = 0; i < obj_batch.positions.size(); i++) {

					objs->positions[i].x = obj_batch.positions[i].x;
					objs->positions[i].y = obj_batch.positions[i].y;
					objs->positions[i].z = obj_batch.positions[i].z;
					objs->confidences[i] = obj_batch.confidences[i];

					objs->velocities[i].x = obj_batch.velocities[i].x;
					objs->velocities[i].y = obj_batch.velocities[i].y;
					objs->velocities[i].z = obj_batch.velocities[i].z;

					objs->timestamps[i] = obj_batch.timestamps[i];

					objs->action_states[i] = (SL_OBJECT_ACTION_STATE)obj_batch.action_states[i];

                    for (int j = 0; j < 6; j++) {
						objs->position_covariances[i][j] = obj_batch.position_covariances.at(i).at(j);
                    }

                    for (int k = 0; k < 4; k++) {
						objs->bounding_boxes_2d[i][k].x = (float) obj_batch.bounding_boxes_2d.at(i).at(k).x;
						objs->bounding_boxes_2d[i][k].y = (float) obj_batch.bounding_boxes_2d.at(i).at(k).y;
                    }

                    for (int l = 0; l < 8; l++) {
						objs->bounding_boxes[i][l].x = obj_batch.bounding_boxes.at(i).at(l).x;
						objs->bounding_boxes[i][l].y = obj_batch.bounding_boxes.at(i).at(l).y;
						objs->bounding_boxes[i][l].z = obj_batch.bounding_boxes.at(i).at(l).z;

                    }

                    if (current_detection_model == sl::DETECTION_MODEL::HUMAN_BODY_FAST || current_detection_model == sl::DETECTION_MODEL::HUMAN_BODY_ACCURATE || current_detection_model == sl::DETECTION_MODEL::HUMAN_BODY_MEDIUM) {

						objs->head_positions[i].x = obj_batch.head_positions[i].x;
						objs->head_positions[i].y = obj_batch.head_positions[i].y;
						objs->head_positions[i].z = obj_batch.head_positions[i].z;

                        for (int m = 0; m < 18; m++) {
							objs->keypoints[i][m].x = obj_batch.keypoints.at(i).at(m).x;
							objs->keypoints[i][m].y = obj_batch.keypoints.at(i).at(m).y;
							objs->keypoints[i][m].z = obj_batch.keypoints.at(i).at(m).z;

							objs->keypoints_2d[i][m].x = obj_batch.keypoints_2d.at(i).at(m).x;
							objs->keypoints_2d[i][m].y = obj_batch.keypoints_2d.at(i).at(m).y;

							objs->keypoints_confidences[i][m] = obj_batch.keypoint_confidences.at(i).at(m);
                        }
                        for (int n = 0; n < 8; n++) {
							objs->head_bounding_boxes[i][n].x = obj_batch.head_bounding_boxes.at(i).at(n).x;
							objs->head_bounding_boxes[i][n].y = obj_batch.head_bounding_boxes.at(i).at(n).y;
							objs->head_bounding_boxes[i][n].z = obj_batch.head_bounding_boxes.at(i).at(n).z;
                        }
                        for (int p = 0; p < 8; p++) {
							objs->head_bounding_boxes_2d[i][p].x = (float) obj_batch.head_bounding_boxes_2d.at(i).at(p).x;
							objs->head_bounding_boxes_2d[i][p].y = (float) obj_batch.head_bounding_boxes_2d.at(i).at(p).y;
                        }
                    }
                }
            }
        }
        return sl::ERROR_CODE::SUCCESS;
    }
    return sl::ERROR_CODE::CAMERA_NOT_DETECTED;
}

sl::ERROR_CODE ZEDController::getObjectsBatchDataCSharp(int index, int* num_data, int* id, int* label, int* sublabel, int* tracking_state,
	struct SL_Vector3 positions[MAX_TRAJECTORY_SIZE], float position_covariances[MAX_TRAJECTORY_SIZE][6], struct SL_Vector3 velocities[MAX_TRAJECTORY_SIZE], unsigned long long timestamps[MAX_TRAJECTORY_SIZE],
	struct SL_Vector2 bounding_boxes_2d[MAX_TRAJECTORY_SIZE][4], struct SL_Vector3 bounding_boxes[MAX_TRAJECTORY_SIZE][8], float confidences[MAX_TRAJECTORY_SIZE], int action_states[MAX_TRAJECTORY_SIZE],
	struct SL_Vector2 keypoints_2d[MAX_TRAJECTORY_SIZE][18], struct SL_Vector3 keypoints[MAX_TRAJECTORY_SIZE][18], struct SL_Vector2 head_bounding_boxes_2d[MAX_TRAJECTORY_SIZE][4], struct SL_Vector3 head_bounding_boxes[MAX_TRAJECTORY_SIZE][8],
	struct SL_Vector3 head_positions[MAX_TRAJECTORY_SIZE], float keypoints_confidences[MAX_TRAJECTORY_SIZE][18]) {

	if (!isNull())
	{
		if (isTrajectoriesUpdated)
		{
			if (index < trajectories.size() && index < MAX_NUMBER_OBJECT) {

				sl::ObjectsBatch obj_batch = trajectories[index];
				*num_data = obj_batch.positions.size();
				*id = (int)obj_batch.id;
				*label = (int)obj_batch.label;
				*sublabel = (int)obj_batch.sublabel;
				*tracking_state = (int)obj_batch.tracking_state;

				for (int i = 0; i < obj_batch.positions.size(); i++) {
					positions[i].x = obj_batch.positions[i].x;
					positions[i].y = obj_batch.positions[i].y;
					positions[i].z = obj_batch.positions[i].z;

					confidences[i] = obj_batch.confidences[i];

					velocities[i].x = obj_batch.velocities[i].x;
					velocities[i].y = obj_batch.velocities[i].y;
					velocities[i].z = obj_batch.velocities[i].z;

					timestamps[i] = obj_batch.timestamps[i];

					action_states[i] = (int)obj_batch.action_states[i];

					head_positions[i].x = obj_batch.head_positions[i].x;
					head_positions[i].y = obj_batch.head_positions[i].y;
					head_positions[i].z = obj_batch.head_positions[i].z;

					for (int j = 0; j < 6; j++) {
						position_covariances[i][j] = obj_batch.position_covariances.at(i).at(j);
					}

					for (int k = 0; k < 4; k++) {
						bounding_boxes_2d[i][k].x = (float)obj_batch.bounding_boxes_2d.at(i).at(k).x;
						bounding_boxes_2d[i][k].y = (float)obj_batch.bounding_boxes_2d.at(i).at(k).y;
					}

					for (int l = 0; l < 8; l++) {
						bounding_boxes[i][l].x = obj_batch.bounding_boxes.at(i).at(l).x;
						bounding_boxes[i][l].y = obj_batch.bounding_boxes.at(i).at(l).y;
						bounding_boxes[i][l].z = obj_batch.bounding_boxes.at(i).at(l).z;

					}

					if (current_detection_model == sl::DETECTION_MODEL::HUMAN_BODY_FAST || current_detection_model == sl::DETECTION_MODEL::HUMAN_BODY_ACCURATE || current_detection_model == sl::DETECTION_MODEL::HUMAN_BODY_MEDIUM) {

						for (int m = 0; m < 18; m++) {
							keypoints[i][m].x = obj_batch.keypoints.at(i).at(m).x;
							keypoints[i][m].y = obj_batch.keypoints.at(i).at(m).y;
							keypoints[i][m].z = obj_batch.keypoints.at(i).at(m).z;

							keypoints_2d[i][m].x = obj_batch.keypoints_2d.at(i).at(m).x;
							keypoints_2d[i][m].y = obj_batch.keypoints_2d.at(i).at(m).y;

							keypoints_confidences[i][m] = obj_batch.keypoint_confidences.at(i).at(m);
						}
						for (int n = 0; n < 8; n++) {
							head_bounding_boxes[i][n].x = obj_batch.head_bounding_boxes.at(i).at(n).x;
							head_bounding_boxes[i][n].y = obj_batch.head_bounding_boxes.at(i).at(n).y;
							head_bounding_boxes[i][n].z = obj_batch.head_bounding_boxes.at(i).at(n).z;
						}
						for (int p = 0; p < 8; p++) {
							head_bounding_boxes_2d[i][p].x = (float)obj_batch.head_bounding_boxes_2d.at(i).at(p).x;
							head_bounding_boxes_2d[i][p].y = (float)obj_batch.head_bounding_boxes_2d.at(i).at(p).y;
						}
					}
				}
			}
		}
		return sl::ERROR_CODE::SUCCESS;
	}
	return sl::ERROR_CODE::CAMERA_NOT_DETECTED;
}


#endif
