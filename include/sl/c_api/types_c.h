#ifndef __TYPES_C_H__
#define __TYPES_C_H__

/**
 * @file
 * */

#define WITH_OBJECT_DETECTION 1
#ifdef WITH_UNLIMITED_CAMERA
#define MAX_CAMERA_PLUGIN 20
#else
#define MAX_CAMERA_PLUGIN 4
#endif

#define MAX_SUBMESH 1000

#include <stdbool.h>

 /**
 * \brief Quaternion
 */
struct SL_Quaternion {
    float x;
    float y;
    float z;
    float w;
};

/**
* \brief Vector2
*/
struct SL_Vector2 {
    float x;
    float y;
};

/**
* \brief Vector3
*/
struct SL_Vector3 {
    float x;
    float y;
    float z;
};

/**
* \brief Vector4
*/
struct SL_Vector4 {
    float x;
    float y;
    float z;
    float w;
};

/**
* \brief uchar2
*/
struct SL_Uchar2 {
    unsigned char x;
    unsigned char y;
};

/**
* \brief uchar3
*/
struct SL_Uchar3 {
    unsigned char x;
    unsigned char y;
    unsigned char z;
};

/**
* \brief uchar4
*/
struct SL_Uchar4 {
    unsigned char x;
    unsigned char y;
    unsigned char z;
    unsigned char w;
};

/**
* \brief Matric4x4
*/
struct SL_Matrix4f {
    float p[16];
};

/**
* \brief Matrix3x3
*/
struct SL_Matrix3f {
    float p[9];
};

struct SL_PoseData {
    bool valid;
    unsigned long long timestamp;
    struct SL_Quaternion rotation;
    struct SL_Vector3 translation;
    int pose_confidence;
	float pose_covariance[36];
	float twist[6];
	float twist_covariance[36];
};

enum UNITY_PLAN_TYPE {
    UNITY_PLAN_TYPE_FLOOR,
    UNITY_PLAN_TYPE_HIT_HORIZONTAL,
    UNITY_PLAN_TYPE_HIT_VERTICAL,
    UNITY_PLAN_TYPE_HIT_UNKNOWN
};

struct SL_PlaneData {
    int error_code;
    enum UNITY_PLAN_TYPE type;
    struct SL_Vector3 plane_normal;
    struct SL_Vector3 plane_center;
    struct SL_Vector3 plane_transform_position;
    struct SL_Quaternion plane_transform_orientation;
    struct SL_Vector4 plane_equation;
    struct SL_Vector2 extents;
    int bounds_size;
    struct SL_Vector3 bounds[256]; //max 256 points but only BoundsSize are actually filled
};

/**
* \brief IMU Data structure
*/
struct SL_IMUData {
    bool is_available;
    unsigned long long timestamp_ns;
    struct SL_Vector3 angular_velocity;
    struct SL_Vector3 linear_acceleration;
    struct SL_Vector3 angular_velocity_unc; //uncalibrated
    struct SL_Vector3 linear_acceleration_unc; //uncalibrated
    struct SL_Quaternion orientation;
    struct SL_Matrix3f orientation_covariance;
    struct SL_Matrix3f angular_velocity_convariance;
    struct SL_Matrix3f linear_acceleration_convariance;
};

/**
* \brief Barometer Data structure
*/
struct SL_BarometerData {
    bool is_available;
    unsigned long long timestamp_ns;
    float pressure;
    float relative_altitude;
};

/**
* \brief Heading state enum
*/
enum SL_HEADING_STATE {
	SL_HEADING_STATE_GOOD,/**< The heading is reliable and not affected by iron interferences. */
	SL_HEADING_STATE_OK,/**< The heading is reliable, but affected by slight iron interferences. */
	SL_HEADING_STATE_NOT_GOOD,/**< The heading is not reliable because affected by strong iron interferences. */
	SL_HEADING_STATE_NOT_CALIBRATED,/**< The magnetometer has not been calibrated. */
	SL_HEADING_STATE_MAG_NOT_AVAILABLE,/**< The magnetomer sensor is not available. */
	SL_HEADING_STATE_LAST
};

/**
* \brief Magnometer Data structure
*/
struct SL_MagnetometerData {
	bool is_available;
	unsigned long long  timestamp_ns;
	struct SL_Vector3 magnetic_field_c; //calibrated
	struct SL_Vector3 magnetic_field_unc; //uncalibrated
	float magnetic_heading;
	enum SL_HEADING_STATE magnetic_heading_state;
	float magnetic_heading_accuracy;
	float effective_rate;
};

/**
* \brief Temperature Data structure
*/
struct SL_TemperatureData {
    float imu_temp;
    float barometer_temp;
    float onboard_left_temp;
    float onboard_right_temp;
};

/**
* \brief Sensor Data structure
*/
struct SL_SensorData {
    //IMU data
    struct SL_IMUData imu;
    struct SL_BarometerData barometer;
    struct SL_MagnetometerData magnetometer;
    struct SL_TemperatureData temperature;
    int camera_moving_state;
    int image_sync_trigger;
};

enum USB_DEVICE {
	USB_DEVICE_OCULUS,
	USB_DEVICE_HTC,
	USB_DEVICE_STEREOLABS
};

struct USB_product {
	int id_vendor;
	int id_product;
};

/**
\brief Lists error codes in the ZED SDK.
 */
enum SL_ERROR_CODE {
	SL_ERROR_CODE_SUCCESS, /**< Standard code for successful behavior.*/
	SL_ERROR_CODE_FAILURE, /**< Standard code for unsuccessful behavior.*/
	SL_ERROR_CODE_NO_GPU_COMPATIBLE, /**< No GPU found or CUDA capability of the device is not supported.*/
	SL_ERROR_CODE_NOT_ENOUGH_GPU_MEMORY, /**< Not enough GPU memory for this depth mode, try a different mode (such as PERFORMANCE), or increase the minimum depth value (see InitParameters::depth_minimum_distance).*/
	SL_ERROR_CODE_CAMERA_NOT_DETECTED, /**< The ZED camera is not plugged or detected.*/
	SL_ERROR_CODE_SENSORS_NOT_INITIALIZED, /**< The MCU that controls the sensors module has an invalid Serial Number. You can try to recover it launching the 'ZED Diagnostic' tool from the command line with the option '-r'.*/
	SL_ERROR_CODE_SENSORS_NOT_AVAILABLE, /**< a ZED-M or ZED2/2i camera is detected but the sensors (imu,barometer...) cannot be opened. Only for ZED-M or ZED2/2i devices. Unplug/replug is required*/
	SL_ERROR_CODE_INVALID_RESOLUTION, /**< In case of invalid resolution parameter, such as a upsize beyond the original image size in Camera::retrieveImage */
	SL_ERROR_CODE_LOW_USB_BANDWIDTH, /**< This issue can occurs when you use multiple ZED or a USB 2.0 port (bandwidth issue).*/
	SL_ERROR_CODE_CALIBRATION_FILE_NOT_AVAILABLE, /**< ZED calibration file is not found on the host machine. Use ZED Explorer or ZED Calibration to get one.*/
	SL_ERROR_CODE_INVALID_CALIBRATION_FILE, /**< ZED calibration file is not valid, try to download the factory one or recalibrate your camera using 'ZED Calibration'.*/
	SL_ERROR_CODE_INVALID_SVO_FILE, /**< The provided SVO file is not valid.*/
	SL_ERROR_CODE_SVO_RECORDING_ERROR, /**< An recorder related error occurred (not enough free storage, invalid file).*/
	SL_ERROR_CODE_SVO_UNSUPPORTED_COMPRESSION, /**< An SVO related error when NVIDIA based compression cannot be loaded.*/
	SL_ERROR_CODE_END_OF_SVOFILE_REACHED, /**<SVO end of file has been reached, and no frame will be available until the SVO position is reset.*/
	SL_ERROR_CODE_INVALID_COORDINATE_SYSTEM, /**< The requested coordinate system is not available.*/
	SL_ERROR_CODE_INVALID_FIRMWARE, /**< The firmware of the ZED is out of date. Update to the latest version.*/
	SL_ERROR_CODE_INVALID_FUNCTION_PARAMETERS, /**< An invalid parameter has been set for the function. */
	SL_ERROR_CODE_CUDA_ERROR, /**< In grab() or retrieveXXX() only, a CUDA error has been detected in the process. Activate verbose in SL_Camera::open for more info.*/
	SL_ERROR_CODE_CAMERA_NOT_INITIALIZED, /**< In grab() only, ZED SDK is not initialized. Probably a missing call to SL_Camera::open.*/
	SL_ERROR_CODE_NVIDIA_DRIVER_OUT_OF_DATE, /**< Your NVIDIA driver is too old and not compatible with your current CUDA version. */
	SL_ERROR_CODE_INVALID_FUNCTION_CALL, /**< The call of the function is not valid in the current context. Could be a missing call of SL_Camera::open. */
	SL_ERROR_CODE_CORRUPTED_SDK_INSTALLATION, /**< The SDK wasn't able to load its dependencies or somes assets are missing, the installer should be launched. */
	SL_ERROR_CODE_INCOMPATIBLE_SDK_VERSION, /**< The installed SDK is incompatible SDK used to compile the program. */
	SL_ERROR_CODE_INVALID_AREA_FILE, /**< The given area file does not exist, check the path. */
	SL_ERROR_CODE_INCOMPATIBLE_AREA_FILE, /**< The area file does not contain enought data to be used or the SL_DEPTH_MODE used during the creation of the area file is different from the one currently set. */
	SL_ERROR_CODE_CAMERA_FAILED_TO_SETUP, /**< Failed to open the camera at the proper resolution. Try another resolution or make sure that the UVC driver is properly installed.*/
	SL_ERROR_CODE_CAMERA_DETECTION_ISSUE, /**< Your ZED can not be opened, try replugging it to another USB port or flipping the USB-C connector.*/
	SL_ERROR_CODE_CANNOT_START_CAMERA_STREAM, /**< Cannot start camera stream. Make sure your camera is not already used by another process or blocked by firewall or antivirus.*/
	SL_ERROR_CODE_NO_GPU_DETECTED, /**< No GPU found, CUDA is unable to list it. Can be a driver/reboot issue.*/
	SL_ERROR_CODE_PLANE_NOT_FOUND, /**< Plane not found, either no plane is detected in the scene, at the location or corresponding to the floor, or the floor plane doesn't match the prior given*/
	SL_ERROR_CODE_MODULE_NOT_COMPATIBLE_WITH_CAMERA, /**< The Object detection module is only compatible with the ZED 2*/
	SL_ERROR_CODE_MOTION_SENSORS_REQUIRED, /**< The module needs the sensors to be enabled (see InitParameters::disable_sensors) */
	SL_ERROR_CODE_MODULE_NOT_COMPATIBLE_WITH_CUDA_VERSION, /**< The module needs a newer version of CUDA */
};

/**
\brief Represents the available resolution defined in the \ref cameraResolution list.
\note The VGA resolution does not respect the 640*480 standard to better fit the camera sensor (672*376 is used).
 */
enum SL_RESOLUTION {
	SL_RESOLUTION_HD2K, /**< 2208*1242, available framerates: 15 fps.*/
	SL_RESOLUTION_HD1080, /**< 1920*1080, available framerates: 15, 30 fps.*/
	SL_RESOLUTION_HD720, /**< 1280*720, available framerates: 15, 30, 60 fps.*/
	SL_RESOLUTION_VGA /**< 672*376, available framerates: 15, 30, 60, 100 fps.*/
};

/**
\brief Lists available unit for measures.
 */
enum SL_UNIT {
	SL_UNIT_MILLIMETER, /**< International System, 1/1000 METER. */
	SL_UNIT_CENTIMETER, /**< International System, 1/100 METER. */
	SL_UNIT_METER, /**< International System, 1 METER */
	SL_UNIT_INCH, /**< Imperial Unit, 1/12 FOOT */
	SL_UNIT_FOOT /**< Imperial Unit, 1 FOOT */
};

/**
\brief Lists available coordinates systems for positional tracking and 3D measures.

\image html CoordinateSystem.png
 */
enum SL_COORDINATE_SYSTEM {
	SL_COORDINATE_SYSTEM_IMAGE, /**< Standard coordinates system in computer vision. Used in OpenCV : see here : http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html */
	SL_COORDINATE_SYSTEM_LEFT_HANDED_Y_UP, /**< Left-Handed with Y up and Z forward. Used in Unity with DirectX. */
	SL_COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP, /**< Right-Handed with Y pointing up and Z backward. Used in OpenGL. */
	SL_COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP, /**< Right-Handed with Z pointing up and Y forward. Used in 3DSMax. */
	SL_COORDINATE_SYSTEM_LEFT_HANDED_Z_UP, /**< Left-Handed with Z axis pointing up and X forward. Used in Unreal Engine. */
	SL_COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP_X_FWD /**< Right-Handed with Z pointing up and X forward. Used in ROS (REP 103). */
};

/**
\brief List of possible camera state
 */
enum SL_CAMERA_STATE {
	SL_CAMERA_STATE_AVAILABLE, /**< Defines if the camera can be opened by the SDK */
	SL_CAMERA_STATE_NOT_AVAILABLE, /**<  Defines if the camera is already opened and unavailable*/
};

/**
\brief Lists compatible ZED Camera model
 */
enum SL_MODEL {
	SL_MODEL_ZED, /**< Defines ZED Camera model */
	SL_MODEL_ZED_M, /**<  Defines ZED Mini (ZED-M) Camera model */
	SL_MODEL_ZED2, /**< Defines ZED 2 Camera model */
	SL_MODEL_ZED2i /**< Defines ZED 2i Camera model */
};

/**
\brief List available memory type
 */
enum SL_MEM
{
	SL_MEM_CPU, /**< CPU Memory (Processor side).*/
	SL_MEM_GPU  /**< GPU Memory (Graphic card side).*/
};

/**
\brief List of the available onboard sensors
 */
enum SL_SENSOR_TYPE {
	SL_SENSOR_TYPE_ACCELEROMETER, /**< Three axis Accelerometer sensor to measure the inertial accelerations. */
	SL_SENSOR_TYPE_GYROSCOPE, /**< Three axis Gyroscope sensor to measure the angular velocitiers. */
	SL_SENSOR_TYPE_MAGNETOMETER, /**< Three axis Magnetometer sensor to measure the orientation of the device respect to the earth magnetic field. */
	SL_SENSOR_TYPE_BAROMETER, /**< Barometer sensor to measure the atmospheric pressure. */
};

/**
\brief List of the available onboard sensors measurement units
 */
enum SL_SENSORS_UNIT {
	SL_SENSORS_UNIT_M_SEC_2, /**< Acceleration [m/s²]. */
	SL_SENSORS_UNIT_DEG_SEC, /**< Angular velocity [deg/s]. */
	SL_SENSORS_UNIT_U_T, /**< MAgnetic Fiels [uT]. */
	SL_SENSORS_UNIT_HPA, /**< Atmospheric pressure [hPa]. */
	SL_SENSORS_UNIT_CELSIUS, /**< Temperature [°C]. */
	SL_SENSORS_UNIT_HERTZ /**< Frequency [Hz]. */
};

enum SL_SIDE {
	SL_SIDE_LEFT, /**< Left side only.*/
	SL_SIDE_RIGHT, /**< Right side only.*/
	SL_SIDE_BOTH /**< Left and Right side.*/
};

enum SL_INPUT_TYPE {
	SL_INPUT_TYPE_USB,
	SL_INPUT_TYPE_SVO,
	SL_INPUT_TYPE_STREAM
};

/**
\brief Defines which type of position matrix is used to store camera path and pose.
 */
enum SL_REFERENCE_FRAME
{
	SL_REFERENCE_FRAME_WORLD,  /**< The transform of SL_Pose will contains the motion with reference to the world frame (previously called PATH).*/
	SL_REFERENCE_FRAME_CAMERA, /**< The transform of SL_Pose will contains the motion with reference to the previous camera frame (previously called POSE).*/
};

/**
\brief Lists specific and particular timestamps
 */
enum SL_TIME_REFERENCE {
	SL_TIME_REFERENCE_IMAGE, /** Defines the timestamp at the time the frame has been extracted from USB stream. */
	SL_TIME_REFERENCE_CURRENT, /**  Defines the timestamp at the time of the function call. */
};

/**
\brief List of codec
 */
enum SL_STREAMING_CODEC {
	SL_STREAMING_CODEC_H264, /**< AVCHD/H264 encoding used in image streaming.*/
	SL_STREAMING_CODEC_H265, /**<  HEVC/H265 encoding used in image streaming.*/
};

/**
\brief Lists available camera settings for the ZED camera (contrast, hue, saturation, gain...).
\warning GAIN and EXPOSURE are linked in auto/default mode (see \ref SL_Camera::setCameraSettings).
 */
enum SL_VIDEO_SETTINGS {
	SL_VIDEO_SETTINGS_BRIGHTNESS, /**< Defines the brightness control. Affected value should be between 0 and 8.*/
	SL_VIDEO_SETTINGS_CONTRAST, /**< Defines the contrast control. Affected value should be between 0 and 8.*/
	SL_VIDEO_SETTINGS_HUE, /**< Defines the hue control. Affected value should be between 0 and 11.*/
	SL_VIDEO_SETTINGS_SATURATION, /**< Defines the saturation control. Affected value should be between 0 and 8.*/
	SL_VIDEO_SETTINGS_SHARPNESS, /**< Defines the digital sharpening control. Affected value should be between 0 and 8.*/
	SL_VIDEO_SETTINGS_GAMMA, /**< Defines the ISP gamma control. Affected value should be between 1 and 9.*/
	SL_VIDEO_SETTINGS_GAIN, /**< Defines the gain control. Affected value should be between 0 and 100 for manual control.*/
	SL_VIDEO_SETTINGS_EXPOSURE, /**< Defines the exposure control. Affected value should be between 0 and 100 for manual control.\n The exposition is mapped linearly in a percentage of the following max values. Special case for the setExposure(0) that corresponds to 0.17072ms.\n The conversion to milliseconds depends on the framerate: <ul><li>15fps setExposure(100) -> 19.97ms</li><li>30fps setExposure(100) -> 19.97ms</li><li>60fps setExposure(100) -> 10.84072ms</li><li>100fps setExposure(100) -> 10.106624ms</li></ul>*/
	SL_VIDEO_SETTINGS_AEC_AGC, /**< Defines if the Gain and Exposure are in automatic mode or not. Setting a Gain or Exposure through @GAIN or @EXPOSURE values will automatically set this value to 0.*/
	SL_VIDEO_SETTINGS_AEC_AGC_ROI, /**< Defines the region of interest for automatic exposure/gain computation. To be used with overloaded @setCameraSettings/@getCameraSettings functions.*/
	SL_VIDEO_SETTINGS_WHITEBALANCE_TEMPERATURE, /**< Defines the color temperature value. Setting a value will automatically set @WHITEBALANCE_AUTO to 0. Affected value should be between 2800 and 6500 with a step of 100.*/
	SL_VIDEO_SETTINGS_WHITEBALANCE_AUTO, /**< Defines if the White balance is in automatic mode or not*/
	SL_VIDEO_SETTINGS_LED_STATUS, /**< Defines the status of the camera front LED. Set to 0 to disable the light, 1 to enable the light. Default value is on. Requires Camera FW 1523 at least.*/
	SL_VIDEO_SETTINGS_LAST
};

/**
\brief Lists retrievable measures.
*/
enum SL_MEASURE {
	SL_MEASURE_DISPARITY, /** Disparity map. Each pixel contains 1 float. SL_MAT_TYPE_F32_C1.*/
	SL_MEASURE_DEPTH, /** Depth map. In SL_UNIT defined in SL_InitParameters. Each pixel contains 1 float. SL_MAT_TYPE_F32_C1.*/
	SL_MEASURE_CONFIDENCE, /** Certainty/confidence of the depth map. Each pixel contains 1 float. SL_MAT_TYPE_F32_C1.*/
	SL_MEASURE_XYZ, /** Point cloud. Each pixel contains 4 float (X, Y, Z, not used). SL_MAT_TYPE_F32_C4.*/
	SL_MEASURE_XYZRGBA, /** Colored point cloud. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the RGBA color.  SL_MAT_TYPE_F32_C4.*/
	SL_MEASURE_XYZBGRA, /** Colored point cloud. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the BGRA color.  SL_MAT_TYPE_F32_C4.*/
	SL_MEASURE_XYZARGB, /** Colored point cloud. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the ARGB color.  SL_MAT_TYPE_F32_C4.*/
	SL_MEASURE_XYZABGR, /** Colored point cloud. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the ABGR color.  SL_MAT_TYPE_F32_C4.*/
	SL_MEASURE_NORMALS, /** Normals vector. Each pixel contains 4 float (X, Y, Z, 0).  SL_MAT_TYPE_F32_C4.*/
	SL_MEASURE_DISPARITY_RIGHT, /**< Disparity map for right sensor. Each pixel contains 1 float. SL_MAT_TYPE_F32_C1.*/
	SL_MEASURE_DEPTH_RIGHT, /** Depth map for right sensor. Each pixel contains 1 float. SL_MAT_TYPE_F32_C1.*/
	SL_MEASURE_XYZ_RIGHT, /** Point cloud for right sensor. Each pixel contains 4 float (X, Y, Z, not used). SL_MAT_TYPE_F32_C4.*/
	SL_MEASURE_XYZRGBA_RIGHT, /** Colored point cloud for right sensor. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the RGBA color. SL_MAT_TYPE_F32_C4.*/
	SL_MEASURE_XYZBGRA_RIGHT, /** Colored point cloud for right sensor. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the BGRA color. SL_MAT_TYPE_F32_C4.*/
	SL_MEASURE_XYZARGB_RIGHT, /** Colored point cloud for right sensor. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the ARGB color. SL_MAT_TYPE_F32_C4.*/
	SL_MEASURE_XYZABGR_RIGHT, /** Colored point cloud for right sensor. Each pixel contains 4 float (X, Y, Z, color). The color need to be read as an usigned char[4] representing the ABGR color. SL_MAT_TYPE_F32_C4.*/
	SL_MEASURE_NORMALS_RIGHT, /** Normals vector for right view. Each pixel contains 4 float (X, Y, Z, 0).  SL_MAT_TYPE_F32_C4.*/
	SL_MEASURE_DEPTH_U16_MM, /** Depth map in millimeter whatever the SL_UNIT defined in SL_InitParameters. Invalid values are set to 0, depth values are clamped at 65000.  Each pixel  contains 1 unsigned short. SL_MAT_TYPE_U16_C1.*/
	SL_MEASURE_DEPTH_U16_MM_RIGHT, /** Depth map in millimeter for right sensor. Each pixel  contains 1 unsigned short. SL_MAT_TYPE_U16_C1.*/
};

/**
\brief Lists available views.
 */
enum SL_VIEW {
	SL_VIEW_LEFT, /** Left BGRA image. Each pixel contains 4 usigned char (B,G,R,A). SL_MAT_TYPE_U8_C4.  */
	SL_VIEW_RIGHT, /** Right BGRA image. Each pixel contains 4 usigned char (B,G,R,A). SL_MAT_TYPE_U8_C4. */
	SL_VIEW_LEFT_GRAY, /** Left GRAY image. Each pixel contains 1 usigned char. SL_MAT_TYPE_U8_C1. */
	SL_VIEW_RIGHT_GRAY, /** Right GRAY image. Each pixel contains 1 usigned char. SL_MAT_TYPE_U8_C1. */
	SL_VIEW_LEFT_UNRECTIFIED, /** Left BGRA unrectified image. Each pixel contains 4 usigned char (B,G,R,A). SL_MAT_TYPE_U8_C4. */
	SL_VIEW_RIGHT_UNRECTIFIED, /** Right BGRA unrectified image. Each pixel contains 4 usigned char (B,G,R,A). SL_MAT_TYPE_U8_C4. */
	SL_VIEW_LEFT_UNRECTIFIED_GRAY, /** Left GRAY unrectified image. Each pixel contains 1 usigned char. SL_MAT_TYPE_U8_C1. */
	SL_VIEW_RIGHT_UNRECTIFIED_GRAY, /** Right GRAY unrectified image. Each pixel contains 1 usigned char. SL_MAT_TYPE_U8_C1. */
	SL_VIEW_SIDE_BY_SIDE, /** Left and right image (the image width is therefore doubled). Each pixel contains 4 usigned char (B,G,R,A). SL_MAT_TYPE_U8_C4. */
	SL_VIEW_DEPTH, /** Color rendering of the depth. Each pixel contains 4 usigned char (B,G,R,A). SL_MAT_TYPE_U8_C4. Use \ref MEASURE "MEASURE_DEPTH" with \ref Camera.retrieveMeasure() to get depth values.*/
	SL_VIEW_CONFIDENCE, /** Color rendering of the depth confidence. Each pixel contains 4 usigned char (B,G,R,A). SL_MAT_TYPE_U8_C4. */
	SL_VIEW_NORMALS, /** Color rendering of the normals. Each pixel contains 4 usigned char (B,G,R,A). SL_MAT_TYPE_U8_C4. */
	SL_VIEW_DEPTH_RIGHT, /** Color rendering of the right depth mapped on right sensor. Each pixel contains 4 usigned char (B,G,R,A). SL_MAT_TYPE_U8_C4. */
	SL_VIEW_NORMALS_RIGHT, /** Color rendering of the normals mapped on right sensor. Each pixel contains 4 usigned char (B,G,R,A). SL_MAT_TYPE_U8_C4. */
};

/**
\brief Lists available object tracking state
 */
enum SL_OBJECT_TRACKING_STATE
{
	SL_OBJECT_TRACKING_STATE_OFF,       /**< The tracking is not yet initialized, the object ID is not usable */
	SL_OBJECT_TRACKING_STATE_OK,        /**< The object is tracked */
	SL_OBJECT_TRACKING_STATE_SEARCHING, /**< The object couldn't be detected in the image and is potentially occluded, the trajectory is estimated */
	SL_OBJECT_TRACKING_STATE_TERMINATE /**< This is the last searching state of the track, the track will be deleted in the next retreiveObject */
};

/**
\brief Lists the different states of positional tracking.
 */
enum SL_POSITIONAL_TRACKING_STATE {
	SL_POSITIONAL_TRACKING_STATE_SEARCHING, /** The camera is searching for a previously known position to locate itself.*/
	SL_POSITIONAL_TRACKING_STATE_OK, /** Positional tracking is working normally.*/
	SL_POSITIONAL_TRACKING_STATE_OFF, /** Positional tracking is not enabled.*/
	SL_POSITIONAL_TRACKING_STATE_FPS_TOO_LOW, /** Effective FPS is too low to give proper results for motion tracking. Consider using PERFORMANCES parameters (DEPTH_MODE_PERFORMANCE, low camera resolution (VGA,HD720))*/
};

/**
\brief Lists the different states of spatial memory area export.
 */
enum SL_AREA_EXPORTING_STATE {
	SL_AREA_EXPORTING_STATE_SUCCESS, /**< The spatial memory file has been successfully created.*/
	SL_AREA_EXPORTING_STATE_RUNNING, /**< The spatial memory is currently written.*/
	SL_AREA_EXPORTING_STATE_NOT_STARTED, /**< The spatial memory file exportation has not been called.*/
	SL_AREA_EXPORTING_STATE_FILE_EMPTY, /**< The spatial memory contains no data, the file is empty.*/
	SL_AREA_EXPORTING_STATE_FILE_ERROR, /**< The spatial memory file has not been written because of a wrong file name.*/
	SL_AREA_EXPORTING_STATE_SPATIAL_MEMORY_DISABLED, /**< The spatial memory learning is disable, no file can be created.*/
};

/**
\brief Gives the spatial mapping state.
 */
enum SL_SPATIAL_MAPPING_STATE {
	SL_SPATIAL_MAPPING_STATE_INITIALIZING, /** The spatial mapping is initializing.*/
	SL_SPATIAL_MAPPING_STATE_OK, /** The depth and tracking data were correctly integrated in the fusion algorithm.*/
	SL_SPATIAL_MAPPING_STATE_NOT_ENOUGH_MEMORY, /** The maximum memory dedicated to the scanning has been reach, the mesh will no longer be updated.*/
	SL_SPATIAL_MAPPING_STATE_NOT_ENABLED, /** Camera::enableSpatialMapping() wasn't called (or the scanning was stopped and not relaunched).*/
	SL_SPATIAL_MAPPING_STATE_FPS_TOO_LOW, /** Effective FPS is too low to give proper results for spatial mapping. Consider using PERFORMANCES parameters (DEPTH_MODE_PERFORMANCE, low camera resolution (VGA,HD720), spatial mapping low resolution)*/
};

/**
\brief Lists the types of spatial maps that can be created.
 */
enum SL_SPATIAL_MAP_TYPE {
	SL_SPATIAL_MAP_TYPE_MESH, /**< Represents a surface with faces, 3D points are linked by edges, no color information.*/
	SL_SPATIAL_MAP_TYPE_FUSED_POINT_CLOUD /**< Geometry is represented by a set of 3D colored points.*/
};

/**
\brief Lists available mesh filtering intensity.
 */
enum SL_MESH_FILTER {
	SL_MESH_FILTER_LOW, /**< Clean the mesh by closing small holes and removing isolated faces.*/
	SL_MESH_FILTER_MEDIUM, /**< Soft decimation and smoothing.*/
	SL_MESH_FILTER_HIGH, /**< Decimate the number of triangles and apply a soft smooth.*/
};

/**
\brief Lists available mesh file formats.
 */
enum SL_MESH_FILE_FORMAT {
	SL_MESH_FILE_FORMAT_PLY, /**< Contains only vertices and faces.*/
	SL_MESH_FILE_FORMAT_PLY_BIN, /**< Contains only vertices and faces, encoded in binary.*/
	SL_MESH_FILE_FORMAT_OBJ, /**< Contains vertices, normals, faces and textures informations if possible.*/

};

/**
\brief Lists available depth sensing modes.
 */
enum SL_SENSING_MODE {
	/** This mode outputs ZED standard depth map that preserves edges and depth accuracy.
	* Applications example: Obstacle detection, Automated navigation, People detection, 3D reconstruction, measurements.*/
	SL_SENSING_MODE_STANDARD,
	/** This mode outputs a smooth and fully dense depth map.
	* Applications example: AR/VR, Mixed-reality capture, Image post-processing.*/
	SL_SENSING_MODE_FILL
};

/**
\brief Lists available depth computation modes.
 */
enum SL_DEPTH_MODE {
	SL_DEPTH_MODE_NONE, /** This mode does not compute any depth map. Only rectified stereo images will be available.*/
	SL_DEPTH_MODE_PERFORMANCE, /** Computation mode optimized for speed.*/
	SL_DEPTH_MODE_QUALITY, /** Computation mode designed for challenging areas with untextured surfaces.*/
	SL_DEPTH_MODE_ULTRA, /** Computation mode favorising edges and sharpness. Requires more GPU memory and computation power.*/
	SL_DEPTH_MODE_NEURAL /**< End to End Neural disparity estimation, requires AI module */
};

/**
\brief Gives the camera flip mode
 */
enum SL_FLIP_MODE {
	SL_FLIP_MODE_OFF, /**  default behavior.*/
	SL_FLIP_MODE_ON, /** Images and camera sensors data are flipped useful when your camera is mounted upside down.*/
	SL_FLIP_MODE_AUTO, /** Live mode: use the camera orientation (if an IMU is available) to set the flip mode. SVO mode: read the state of this enum when recorded.*/
};

/**
\brief List available copy operation on Mat
 */
enum SL_COPY_TYPE
{
	SL_COPY_TYPE_CPU_CPU, /**< copy data from CPU to CPU.*/
	SL_COPY_TYPE_CPU_GPU, /**< copy data from CPU to GPU.*/
	SL_COPY_TYPE_GPU_GPU, /**< copy data from GPU to GPU.*/
	SL_COPY_TYPE_GPU_CPU  /**< copy data from GPU to CPU.*/
};

/**
\brief Lists available compression modes for SVO recording.
\brief SL_SVO_COMPRESSION_MODE_LOSSLESS is an improvement of previous lossless compression (used in ZED Explorer), even if size may be bigger, compression time is much faster.
 */
enum SL_SVO_COMPRESSION_MODE {
	SL_SVO_COMPRESSION_MODE_LOSSLESS, /**< PNG/ZSTD (lossless) CPU based compression : avg size = 42% (of RAW).*/
	SL_SVO_COMPRESSION_MODE_H264, /**< H264(AVCHD) GPU based compression : avg size = 1% (of RAW). Requires a NVIDIA GPU*/
	SL_SVO_COMPRESSION_MODE_H265, /**< H265(HEVC) GPU based compression: avg size = 1% (of RAW). Requires a NVIDIA GPU, Pascal architecture or newer*/
	SL_SVO_COMPRESSION_MODE_H264_LOSSLESS, /**< H264 Lossless GPU/Hardware based compression: avg size = 25% (of RAW). Provides a SSIM/PSNR result (vs RAW) >= 99.9%. Requires a NVIDIA GPU */
	SL_SVO_COMPRESSION_MODE_H265_LOSSLESS, /**< H265 Lossless GPU/Hardware based compression: avg size = 25% (of RAW). Provides a SSIM/PSNR result (vs RAW) >= 99.9%. Requires a NVIDIA GPU */
};

/**
\brief List available Mat formats.
 */
enum SL_MAT_TYPE
{
	SL_MAT_TYPE_F32_C1, /**< float 1 channel.*/
	SL_MAT_TYPE_F32_C2, /**< float 2 channels.*/
	SL_MAT_TYPE_F32_C3, /**< float 3 channels.*/
	SL_MAT_TYPE_F32_C4, /**< float 4 channels.*/
	SL_MAT_TYPE_U8_C1,  /**< unsigned char 1 channel.*/
	SL_MAT_TYPE_U8_C2,  /**< unsigned char 2 channels.*/
	SL_MAT_TYPE_U8_C3,  /**< unsigned char 3 channels.*/
	SL_MAT_TYPE_U8_C4,  /**< unsigned char 4 channels.*/
	SL_MAT_TYPE_U16_C1  /**< unsigned short 1 channel.*/
};

/**
\brief Available object subclass, given as hint, when using object tracking an object can change of OBJECT_SUBCLASS while keeping the same OBJECT_CLASS and id (i.e: frame M: MOTORBIKE, frame N:BICYCLE)
  */
enum SL_OBJECT_SUBCLASS
{
	SL_OBJECT_SUBCLASS_PERSON,       /**< PERSON / PERSON_BODY */
	SL_OBJECT_SUBCLASS_BICYCLE,      /**< VEHICLE */
	SL_OBJECT_SUBCLASS_CAR,          /**< VEHICLE */
	SL_OBJECT_SUBCLASS_MOTORBIKE,    /**< VEHICLE */
	SL_OBJECT_SUBCLASS_BUS,          /**< VEHICLE */
	SL_OBJECT_SUBCLASS_TRUCK,        /**< VEHICLE */
	SL_OBJECT_SUBCLASS_BOAT,         /**< VEHICLE */
	SL_OBJECT_SUBCLASS_BACKPACK,     /**< BAG */
	SL_OBJECT_SUBCLASS_HANDBAG,      /**< BAG */
	SL_OBJECT_SUBCLASS_SUITCASE,     /**< BAG */
	SL_OBJECT_SUBCLASS_BIRD,        /**< ANIMAL */
	SL_OBJECT_SUBCLASS_CAT,         /**< ANIMAL */
	SL_OBJECT_SUBCLASS_DOG,         /**< ANIMAL */
	SL_OBJECT_SUBCLASS_HORSE,       /**< ANIMAL */
	SL_OBJECT_SUBCLASS_SHEEP,       /**< ANIMAL */
	SL_OBJECT_SUBCLASS_COW,         /**< ANIMAL */
	SL_OBJECT_SUBCLASS_CELLPHONE,   /**< ELECTRONIC */
	SL_OBJECT_SUBCLASS_LAPTOP,      /**< ELECTRONIC */
	SL_OBJECT_SUBCLASS_BANANA,      /**< FRUIT/VEGETABLE */
	SL_OBJECT_SUBCLASS_APPLE,       /**< FRUIT/VEGETABLE */
	SL_OBJECT_SUBCLASS_ORANGE,      /**< FRUIT/VEGETABLE */
	SL_OBJECT_SUBCLASS_CARROT,      /**< FRUIT/VEGETABLE */
	SL_OBJECT_SUBCLASS_PERSON_HEAD, /**< PERSON */
	SL_OBJEC_SUBCLASS_SPORTSBALL  /**< SPORTSBALL >*/

};

/**
\brief Lists available object action state
 */
enum SL_OBJECT_ACTION_STATE
{
	SL_OBJECT_ACTION_STATE_IDLE,   /**< The object is staying static. */
	SL_OBJECT_ACTION_STATE_MOVING, /**< The object is moving. */
};

/**
\brief List available models for detection
 */
enum SL_DETECTION_MODEL {
	SL_DETECTION_MODEL_MULTI_CLASS_BOX, /**< Any objects, bounding box based */
	SL_DETECTION_MODEL_MULTI_CLASS_BOX_ACCURATE, /**< Any objects, bounding box based, more accurate but slower than the base model */
	SL_DETECTION_MODEL_HUMAN_BODY_FAST, /**<  Keypoints based, specific to human skeleton, real time performance even on Jetson or low end GPU cards */
	SL_DETECTION_MODEL_HUMAN_BODY_ACCURATE, /**<  Keypoints based, specific to human skeleton, state of the art accuracy, requires powerful GPU */
	SL_DETECTION_MODEL_MULTI_CLASS_BOX_MEDIUM, /**< Any objects, bounding box based, compromise between accuracy and speed */
	SL_DETECTION_MODEL_HUMAN_BODY_MEDIUM, /**<  Keypoints based, specific to human skeleton, compromise between accuracy and speed */
	SL_DETECTION_MODEL_PERSON_HEAD_BOX, /**<  Bounding Box detector specialized in person heads, particulary well suited for crowded environement, the person localization is also improved */
	SL_DETECTION_MODEL_CUSTOM_BOX_OBJECTS /**< For external inference, using your own custom model and/or frameworks. This mode disable the internal inference engine, the 2D bounding box detection must be provided */
};

/**
   \brief Lists available  AI moles.
   */
enum SL_AI_MODELS {
	SL_AI_MODELS_MULTI_CLASS_DETECTION, // related to sl::DETECTION_MODEL::MULTI_CLASS_BOX
	SL_AI_MODELS_MULTI_CLASS_MEDIUM_DETECTION, // related to sl::DETECTION_MODEL::MULTI_CLASS_BOX_MEDIUM
	SL_AI_MODELS_MULTI_CLASS_ACCURATE_DETECTION, // related to sl::DETECTION_MODEL::MULTI_CLASS_BOX_ACCURATE
	SL_AI_MODELS_HUMAN_BODY_FAST_DETECTION, // related to sl::DETECTION_MODEL::HUMAN_BODY_FAST
	SL_AI_MODELS_HUMAN_BODY_MEDIUM_DETECTION, // related to sl::DETECTION_MODEL::HUMAN_BODY_MEDIUM
	SL_AI_MODELS_HUMAN_BODY_ACCURATE_DETECTION, // related to sl::DETECTION_MODEL::HUMAN_BODY_ACCURATE
	SL_AI_MODELS_PERSON_HEAD_DETECTION, // related to sl::DETECTION_MODEL::PERSON_HEAD_BOX
	SL_AI_MODELS_REID_ASSOCIATION, // related to sl::BatchParameters::enable
	SL_AI_MODELS_NEURAL_DEPTH, // related to sl::DEPTH_MODE::NEURAL
	SL_AI_MODELS_LAST
};

/**
\brief Lists of supported bounding box preprocessing
 */
enum SL_OBJECT_FILTERING_MODE {
	/**
	 * \brief SDK will not apply any preprocessing to the detected objects
	 */
	SL_OBJECT_FILTERING_MODE_NONE,
    /**
     * \brief SDK will remove objects that are in the same 3D position as an already tracked object (independant of class ID)
     */
    SL_OBJECT_FILTERING_MODE_NMS_3D,
    /**
     * \brief SDK will remove objects that are in the same 3D position as an already tracked object of the same class ID
     */
    SL_OBJECT_FILTERING_MODE_NMS_3D_PER_CLASS
};


/**
\brief Lists of supported skeleton body model
*/
enum SL_BODY_FORMAT
{
	/**
	 * \brief 18  keypoint model of COCO 18.
	 * \note local keypoint angle and position are not available with this format
	 */
	SL_BODY_FORMAT_POSE_18,

	/**
	 * \brief 34 keypoint model.
	 * \note local keypoint angle and position are available
	 * \warning The SDK will automatically enable fitting.
	 */
	SL_BODY_FORMAT_POSE_34,
};

/**
 * \brief semantic of human body parts and order of \ref ObjectData::keypoint.
 */
enum SL_BODY_PARTS_POSE_18
{
	SL_BODY_PARTS_POSE_18_NOSE,
	SL_BODY_PARTS_POSE_18_NECK,
	SL_BODY_PARTS_POSE_18_RIGHT_SHOULDER,
	SL_BODY_PARTS_POSE_18_RIGHT_ELBOW,
	SL_BODY_PARTS_POSE_18_RIGHT_WRIST,
	SL_BODY_PARTS_POSE_18_LEFT_SHOULDER,
	SL_BODY_PARTS_POSE_18_LEFT_ELBOW,
	SL_BODY_PARTS_POSE_18_LEFT_WRIST,
	SL_BODY_PARTS_POSE_18_RIGHT_HIP,
	SL_BODY_PARTS_POSE_18_RIGHT_KNEE,
	SL_BODY_PARTS_POSE_18_RIGHT_ANKLE,
	SL_BODY_PARTS_POSE_18_LEFT_HIP,
	SL_BODY_PARTS_POSE_18_LEFT_KNEE,
	SL_BODY_PARTS_POSE_18_LEFT_ANKLE,
	SL_BODY_PARTS_POSE_18_RIGHT_EYE,
	SL_BODY_PARTS_POSE_18_LEFT_EYE,
	SL_BODY_PARTS_POSE_18_RIGHT_EAR,
	SL_BODY_PARTS_POSE_18_LEFT_EAR,
	SL_BODY_PARTS_POSE_18_LAST
};

/**
 * \brief semantic of human body parts and order of \ref ObjectData::keypoint for BODY_FORMAT::POSE_34.
 */
enum SL_BODY_PARTS_POSE_34
{
	SL_BODY_PARTS_POSE_34_PELVIS,
	SL_BODY_PARTS_POSE_34_NAVAL_SPINE,
	SL_BODY_PARTS_POSE_34_CHEST_SPINE,
	SL_BODY_PARTS_POSE_34_NECK,
	SL_BODY_PARTS_POSE_34_LEFT_CLAVICLE,
	SL_BODY_PARTS_POSE_34_LEFT_SHOULDER,
	SL_BODY_PARTS_POSE_34_LEFT_ELBOW,
	SL_BODY_PARTS_POSE_34_LEFT_WRIST,
	SL_BODY_PARTS_POSE_34_LEFT_HAND,
	SL_BODY_PARTS_POSE_34_LEFT_HANDTIP,
	SL_BODY_PARTS_POSE_34_LEFT_THUMB,
	SL_BODY_PARTS_POSE_34_RIGHT_CLAVICLE,
	SL_BODY_PARTS_POSE_34_RIGHT_SHOULDER,
	SL_BODY_PARTS_POSE_34_RIGHT_ELBO,
	SL_BODY_PARTS_POSE_34_RIGHT_WRIST,
	SL_BODY_PARTS_POSE_34_RIGHT_HAND,
	SL_BODY_PARTS_POSE_34_RIGHT_HANDTIP,
	SL_BODY_PARTS_POSE_34_RIGHT_THUMB,
	SL_BODY_PARTS_POSE_34_LEFT_HIP,
	SL_BODY_PARTS_POSE_34_LEFT_KNEE,
	SL_BODY_PARTS_POSE_34_LEFT_ANKLE,
	SL_BODY_PARTS_POSE_34_LEFT_FOOT,
	SL_BODY_PARTS_POSE_34_RIGHT_HIP,
	SL_BODY_PARTS_POSE_34_RIGHT_KNEE,
	SL_BODY_PARTS_POSE_34_RIGHT_ANKLE,
	SL_BODY_PARTS_POSE_34_RIGHT_FOOT,
	SL_BODY_PARTS_POSE_34_HEAD,
	SL_BODY_PARTS_POSE_34_NOSE,
	SL_BODY_PARTS_POSE_34_LEFT_EYE,
	SL_BODY_PARTS_POSE_34_LEFT_EAR,
	SL_BODY_PARTS_POSE_34_RIGHT_EYE,
	SL_BODY_PARTS_POSE_34_RIGHT_EAR,
	SL_BODY_PARTS_POSE_34_LEFT_HEEL,
	SL_BODY_PARTS_POSE_34_RIGHT_HEEL,
	SL_BODY_PARTS_POSE_34_LAST
};


/**
* \brief Resolution
*/
struct SL_Resolution {
	long long width;
	long long height;
};

/**
Struct containing all parameters passed to the SDK when initializing the ZED.
These parameters will be fixed for the whole execution life time of the camera.
For more details, see the InitParameters class in the SDK API documentation:
https://www.stereolabs.com/docs/api/structsl_1_1InitParameters.html
*/
struct SL_InitParameters
{
	/**
	The SDK can handle different input types:
	  - Select a camera by its ID (/dev/video<i>X</i> on Linux, and 0 to N cameras connected on Windows)
	  - Select a camera by its serial number
	  - Open a recorded sequence in the SVO file format
	  - Open a streaming camera from its IP address and port

	This parameter allows you to select to desired input.
	*/
	enum  SL_INPUT_TYPE input_type;
	/**
	Define the chosen camera resolution. Small resolutions offer higher framerate and lower computation time (SL_RESOLUTION).\n
	In most situations, the \ref RESOLUTION "RESOLUTION_HD720" at 60 fps is the best balance between image quality and framerate.\n
	Available resolutions are listed here: \ref RESOLUTION.
	\n default : \ref RESOLUTION "RESOLUTION_HD720"
	 */
	enum  SL_RESOLUTION resolution;
	/**
	Requested camera frame rate. If set to 0, the highest FPS of the specified \ref camera_resolution will be used.\n
	See \ref RESOLUTION for a list of supported framerates.
	\n default : 0
	\note If the requested camera_fps is unsupported, the closest available FPS will be used.
	 */
	int camera_fps;
	/**
	Id of the Camera.
	*/
	int camera_device_id;
	/**
	If you are using the camera upside down, setting this parameter to FLIP_MODE_ON will cancel its rotation. The images will be horizontally flipped.
	\n default : FLIP_MODE_AUTO
	 * From ZED SDK 3.2 a new FLIP_MODE enum was introduced to add the automatic flip mode detection based on the IMU gravity detection. This only works for ZED-M or ZED2 cameras.
	 */
	enum  SL_FLIP_MODE camera_image_flip;
	/**
	At initialization, the \ref Camera runs a self-calibration process that corrects small offsets from the device's factory calibration.\n
	A drawback is that calibration parameters will slightly change from one run to another, which can be an issue for repeatability.\n
	If set to true, self-calibration will be disabled and calibration parameters won't be optimized.\n
	default : false
	\note In most situations, self calibration should remain enabled.
	\note You can also trigger the self-calibration at anytime after open() by calling \ref Camera::UpdateSelfCalibration(), even if this parameter is set to true.

	 */
	bool camera_disable_self_calib;
	/**
	By default, the SDK only computes a single depth map, aligned with the left camera image.\n
	This parameter allows you to enable the \ref MEASURE "MEASURE_DEPTH_RIGHT" and other \ref MEASURE "MEASURE_<XXX>_RIGHT" at the cost of additional computation time.\n

	For example, mixed reality pass-through applications require one depth map per eye, so this parameter can be activated.
	\n default : false
	 */
	bool enable_right_side_measure;

	/**
	When playing back an SVO file, each call to \ref Camera::grab() will extract a new frame and use it.\n
	However, this ignores the real capture rate of the images saved in the SVO file.\n
	Enabling this parameter will bring the SDK closer to a real simulation when playing back a file by using the images' timestamps. However, calls to \ref Camera::grab() will return an error when trying to play too fast, and frames will be dropped when playing too slowly.

	\n default : false
	 */
	bool svo_real_time_mode;
	/**
	The SDK offers several \ref DEPTH_MODE options offering various levels of performance and accuracy.
	\n This parameter allows you to set the \ref DEPTH_MODE that best matches your needs.
	\n default : \ref DEPTH_MODE "DEPTH_MODE_PERFORMANCE"
	 */
	enum SL_DEPTH_MODE depth_mode;
	/**
	Regions of the generated depth map can oscillate from one frame to another. These oscillations result from a lack of texture (too homogeneous) on an object and by image noise.
	\n This parameter enables a stabilization filter that reduces these oscillations.
	\n default : true
	\note The stabilization uses the positional tracking to increase its accuracy, so the Positional Tracking module will be enabled automatically when set to true.\n
	 */
	bool depth_stabilization;
	/**
	This parameter allows you to specify the minimum depth value (from the camera) that will be computed, measured in the \ref UNIT you define.
	\n In stereovision (the depth technology used by the camera), looking for closer depth values can have a slight impact on performance and memory consumption.
	\n On most of modern GPUs, performance impact will be low. However, the impact of memory footprint will be visible.
	\n In cases of limited computation power, increasing this value can provide better performance.
	\n default : (-1) corresponding to 700 mm for a ZED/ZED2 and 300 mm for ZED Mini.

	\note With a ZED camera you can decrease this value to 300 mm whereas you can set it to 100 mm using a ZED Mini and 200 mm for a ZED2. In any case this value cannot be greater than 3 meters.
	\note Specific value (0) : This will set the depth minimum distance to the minimum authorized value :
								- 300mm for ZED
								- 100mm for ZED-M
								- 200mm for ZED2
	*/
	float depth_minimum_distance;
	/**
	Defines the current maximum distance that can be computed in the defined \ref UNIT.
	When estimating the depth, the SDK uses this upper limit to turn higher values into \ref TOO_FAR ones.

	\note Changing this value has no impact on performance and doesn't affect the positional tracking nor the spatial mapping. (Only the depth, point cloud, normals)
	 */
	float depth_maximum_distance;
	/**
	This parameter allows you to select the unit to be used for all metric values of the SDK. (depth, point cloud, tracking, mesh, and others) (SL_UNIT).
	\n default : \ref UNIT "UNIT_MILLIMETER"
	 */
	enum  SL_UNIT coordinate_unit;
	/**
	Positional tracking, point clouds and many other features require a given \ref COORDINATE_SYSTEM to be used as reference.
	This parameter allows you to select the \ref COORDINATE_SYSTEM used by the \ref Camera to return its measures.
	\n This defines the order and the direction of the axis of the coordinate system.
	\n default : \ref COORDINATE_SYSTEM "COORDINATE_SYSTEM_IMAGE"
	 */
	enum  SL_COORDINATE_SYSTEM coordinate_system;
	/**
	By default the SDK will use the most powerful NVIDIA graphics card found.
	However, when running several applications, or using several cameras at the same time, splitting the load over available GPUs can be useful.
	This parameter allows you to select the GPU used by the \ref Camera using an ID from 0 to n-1 GPUs in your PC.
	\n default : -1
	\note A non-positive value will search for all CUDA capable devices and select the most powerful.
	 */
	int sdk_gpu_id;
	/**
	This parameter allows you to enable the verbosity of the SDK to get a variety of runtime information in the console.
	When developing an application, enabling verbose mode can help you understand the current SDK behavior.
	\n However, this might not be desirable in a shipped version.
	\n default : false
	\note The verbose messages can also be exported into a log file. See \ref sdk_verbose_log_file for more.
	 */
	int  sdk_verbose;
	/**
	Force the motion sensors opening of the ZED 2 / ZED-M to open the camera.
	\n default : false.
	\n If set to false, the SDK will try to <b>open and use</b> the IMU (second USB device on USB2.0) and will open the camera successfully even if the sensors failed to open.
	\n This can be used for example when using a USB3.0 only extension cable (some fiber extension for example).
	\n This parameter only impacts the LIVE mode.
	\n If set to true, the camera will fail to open if the sensors cannot be opened. This parameter should be used when the IMU data must be available, such as Object Detection module or when the gravity is needed.
	\note This setting is not taken into account for ZED camera since it does not include sensors.
	 */
	bool sensors_required;
	/**
	Enable or Disable the Enhanced Contrast Technology, to improve image quality.
	\n default : true.
	\n If set to true, image enhancement will be activated in camera ISP. Otherwise, the image will not be enhanced by the IPS.
	\n This only works for firmware version starting from 1523 and up.
	 */
	bool enable_image_enhancement;

	/**
	Define a timeout in seconds after which an error is reported if the \ref open() command fails.
	Set to '-1' to try to open the camera endlessly without returning error in case of failure.
	Set to '0' to return error in case of failure at the first attempt.
	\n This parameter only impacts the LIVE mode.
	 */
	float open_timeout_sec;
};

/**
Parameters that define the behavior of the grab.
*/
struct SL_RuntimeParameters
{
	/**
	Defines the algorithm used for depth map computation, more info : \ref SENSING_MODE definition.
	\n default : \ref SENSING_MODE_STANDARD
	*/
	enum SL_SENSING_MODE sensing_mode;
	/**
	Provides 3D measures (point cloud and normals) in the desired reference frame (default is REFERENCE_FRAME_CAMERA)
	\n default : \ref REFERENCE_FRAME_CAMERA
	 */
	enum  SL_REFERENCE_FRAME reference_frame;
	/**
	Defines if the depth map should be computed.
	\n If false, only the images are available.
	\n default : true
	 */
	bool enable_depth;
	/**
	Threshold to reject depth values based on their confidence.
	\n Each depth pixel has a corresponding confidence. (\ref MEASURE "MEASURE_CONFIDENCE"), the confidence range is [1,100].
	\n By default, the confidence threshold is set at 100, meaning that no depth pixel will be rejected.
	\n Decreasing this value will remove depth data from both objects edges and low textured areas, to keep only confident depth estimation data.
	 */
	int confidence_threshold;
	/**
	Threshold to reject depth values based on their texture confidence.
	\n The texture confidence range is [1,100].
	\n By default, the texture confidence threshold is set at 100, meaning that no depth pixel will be rejected.
	\n Decreasing this value will remove depth data from image areas which are uniform.
	 */
	int texture_confidence_threshold;

    /**
     Defines if the saturated area (Luminance>=255) must be removed from depth map estimation
     \n True by default
     \n It is recommended to keep this parameter at true because saturated area can create false detection.
     */
    bool remove_saturated_areas;
};

/**
Properties of a camera.
*/
struct SL_DeviceProperties {
	/**
	the camera state
	 */
	enum  SL_CAMERA_STATE camera_state;
	/**
	the camera id
	 */
	int id;
	/**
	the camera model
	 */
	enum  SL_MODEL camera_model;
	/**
	the camera serial number
	\n Not provided for Windows
	 */
	unsigned int sn;
};

struct SL_CameraParameters {
	float fx; /**< Focal length in pixels along x axis. */
	float fy; /**< Focal length in pixels along y axis. */
	float cx; /**< Optical center along x axis, defined in pixels (usually close to width/2). */
	float cy; /**< Optical center along y axis, defined in pixels (usually close to height/2). */
	double disto[5]; /**< Distortion factor : [ k1, k2, p1, p2, k3 ]. Radial (k1,k2,k3) and Tangential (p1,p2) distortion.*/
	float v_fov; /**< Vertical field of view, in degrees. */
	float h_fov; /**< Horizontal field of view, in degrees.*/
	float d_fov; /**< Diagonal field of view, in degrees.*/
	struct SL_Resolution image_size; /** size in pixels of the images given by the camera.*/
};

/**
Holds calibration information about the current ZED's hardware, including per-sensor calibration and offsets between the two sensors.
*/
struct SL_CalibrationParameters
{
	/**
	Intrinsic parameters of the left camera
	*/
	struct SL_CameraParameters left_cam;
	/**
	Intrinsic parameters of the right camera
	*/
	struct SL_CameraParameters right_cam;
	/**
	Left to Right camera rotation, expressed in user coordinate system and unit (defined by InitParameters).
	*/
	struct SL_Vector4 rotation;
	/**
	Left to Right camera translation, expressed in user coordinate system and unit (defined by InitParameters).
	*/
	struct SL_Vector3 translation;
};

/**
\brief Structure containing information about a single sensor available in the current device
\note This object is meant to be used as a read - only container, editing any of its fields won't impact the SDK.
*/
struct SL_SensorParameters {
	enum  SL_SENSOR_TYPE type; /** The type of the sensor as \ref DEVICE_SENSORS (SL_SENSOR_TYPE)*/
	float resolution; /** The resolution of the sensor. */
	float sampling_rate; /** The sampling rate (or ODR) of the sensor. */
	struct SL_Vector2 range; /** The range values of the sensor. MIN: "range.x", MAX: "range.y" */
	float noise_density; /** also known as white noise, given as continous (frequency independant). Units will be expressed in sensor_unit/squared_root(Hz). "NAN" if the information is not available */
	float random_walk; /** derived from the Allan Variance, given as continous (frequency independant). Units will be expressed in sensor_unit/s/squared_root(Hz)."NAN" if the information is not available */
	enum  SL_SENSORS_UNIT sensor_unit; /** The string relative to the measurement unit of the sensor (SL_SENSORS_UNIT). */
	bool is_available;
};


/**
\brief Structure containing information about the camera sensor
\note This object is meant to be used as a read-only container, editing any of its field won't impact the SDK.
\warning CalibrationParameters are returned in SL_COORDINATE_SYSTEM_IMAGE, they are not impacted by the InitParameters::coordinate_system
 */
struct SL_CameraConfiguration {
	/**< Intrinsic and Extrinsic stereo parameters for rectified/undistorded images (default).  */
	struct SL_CalibrationParameters calibration_parameters;
	/**< Intrinsic and Extrinsic stereo parameters for original images (unrectified/distorded). */
	struct SL_CalibrationParameters calibration_parameters_raw;
	/**< The internal firmware version of the camera. */
	unsigned int firmware_version;
	/**< The camera capture FPS */
	float fps;
	/**< The camera resolution */
	struct SL_Resolution resolution;
};

/**
Structure containing information about all the sensors available in the current device.
*/
struct SL_SensorsConfiguration
{
	/**
	The firmware version of the sensor module, 0 if no sensors are available (ZED camera model).
	*/
	unsigned int firmware_version;
	/**
	IMU to Left camera rotation (quaternion).
	*/
	struct SL_Vector4 camera_ium_rotation;
	/**
	IMU to Left camera translation (SL_float3).
	*/
	struct SL_Vector3 camera_imu_translation;
	/**
	Magnetometer to IMU rotation. That contains rotation between IMU frame and magnetometer frame.
	*/
	struct SL_Vector4 ium_magnetometer_rotation;
	/**
	Magnetometer to IMU translation. That contains translation between IMU frame and magnetometer frame.
	*/
	struct SL_Vector3 ium_magnetometer_translation;
	/**
	Configuration of the accelerometer device
	*/
	struct SL_SensorParameters accelerometer_parameters;
	/**
	Configuration of the gyroscope device
	*/
	struct SL_SensorParameters gyroscope_parameters;
	/**
	Configuration of the magnetometer device
	*/
	struct SL_SensorParameters magnetometer_parameters;
	/**
	Configuration of the barometer device
	*/
	struct SL_SensorParameters barometer_parameters;
};


/**
\brief Structure containing information of a single camera (serial number, model, input type, etc.)

That information about the camera will be returned by sl_get_camera_information().

\note This object is meant to be used as a read-only container, editing any of its field won't impact the SDK.
 */
struct SL_CameraInformation {

	/**< The serial number of the camera.  */
	unsigned int serial_number;
	/**< The model of the camera (ZED, ZED-M or ZED2). */
	enum SL_MODEL camera_model;
	/**< Input type used in SDK. */
	enum SL_INPUT_TYPE input_type;
	/**< Camera configuration as defined in \ref CameraConfiguration. */
	struct SL_CameraConfiguration camera_configuration;
	/**< Device Sensors configuration as defined in \ref SensorsConfiguration. */
	struct SL_SensorsConfiguration sensors_configuration;
};


/**
Parameters for positional tracking initialization.
*/
struct SL_PositionalTrackingParameters
{
	/**
	Rotation of the camera in the world frame when the camera is started. By default, it should be identity.
	*/
	struct SL_Quaternion initial_world_rotation;
	/**
	Position of the camera in the world frame when the camera is started. By default, it should be identity.
	*/
	struct SL_Vector3 initial_world_position;
	/**
	This mode enables the camera to remember its surroundings. This helps correct positional tracking drift, and can be helpful for positioning
	different cameras relative to one other in space.
	\n default: true
	\warning: This mode requires more resources to run, but greatly improves tracking accuracy. We recommend leaving it on by default.
		*/
	bool enable_area_memory;
	/**
	This mode enables smooth pose correction for small drift correction.
		*/
	bool enable_pose_smothing;
	/**
	This mode initializes the tracking to be aligned with the floor plane to better position the camera in space.
	\n default: false
	\note This launches floor plane detection in the background until a suitable floor plane is found.
	The tracking is in POSITIONAL_TRACKING_STATE_SEARCHING state.

	\warning This features work best with the ZED-M since it needs an IMU to classify the floor.
		* The ZED needs to look at the floor during initialization for optimum results.
		*/
	bool set_floor_as_origin;
	/**
	This setting allows you define the camera as static. If true, it will not move in the environment. This allows you to set its position using initial_world_transform.
	\n All SDK functionalities requiring positional tracking will be enabled.
	\n Camera::getPosition() will return the value set as initial_world_transform for the PATH, and identity as the POSE.
		*/
	bool set_as_static;
	/**
	This setting allows you to enable or disable IMU fusion. When set to false, only the optical odometry will be used.
	\n default: true
	\note This setting has no impact on the tracking of a ZED camera; only the ZED Mini uses a built-in IMU.
		*/
	bool enable_imu_fusion;
};

/**
\brief Recording structure that contains information about SVO.
 */
struct SL_RecordingStatus {
	/**< Recorder status, true if enabled */
	bool is_recording; 
	/**< Recorder status, true if the pause is enabled */
	bool is_paused; 
	/**< Status of current frame. True for success or false if the frame couldn't be written in the SVO file.*/
	bool status; 
	/**< Compression time for the current frame in ms.*/
	double current_compression_time; 
	/**< Compression ratio (% of raw size) for the current frame.*/
	double current_compression_ratio; 
	/**< Average compression time in ms since beginning of recording.*/
	double average_compression_time;
	/**< Average compression ratio (% of raw size) since beginning of recording.*/
	double average_compression_ratio; 
};

/**
\brief Sets the recording parameters.
 */
struct SL_RecordingParameters {
	/**
	\brief filename of the SVO file.
	 */
	unsigned char video_filename[256];

	/**
	\brief can be one of the \ref SL_SVO_COMPRESSION_MODE enum
	 */
	enum SL_SVO_COMPRESSION_MODE compression_mode;

	/**
	 \brief overrides default bitrate of the SVO file, in KBits/s. Only works if \ref SVO_COMPRESSION_MODE is H264 or H265.
	 \n default : 0 means default values (depends on the resolution)
	 \note Available range : 0 or [1000 - 60000]
	 */
	unsigned int bitrate;

	/**
	 \brief defines the target framerate for the recording module.
	 \warning This framerate must be below or equal to the camera framerate and camera framerate must be a multiple of the target framerate. It means that
	 it must respect camera_framerate%target_framerate==0
	 Allowed framerates are 15,30, 60 or 100 if possible.
	 Any other values will be discarded and camera FPS will be taken.
	 \n default : 0, meaning that the camera framerate will be taken
	 */
	unsigned int target_framerate;

	/**
	\brief In case of streaming input, if set to false, it will avoid decoding/re-encoding and convert directly streaming input into a SVO file.
	This saves a encoding session and can be especially useful on NVIDIA Geforce cards where the number of encoding session is limited.
	\note compression_mode, target_framerate and bitrate will be ignored in this mode.
	 */
	bool transcode_streaming_input;
};

/**
\brief Sets the streaming parameters.
\note Parameters can be user adjusted.
 */
struct SL_StreamingParameters {

	/**
	\brief Defines the codec used for streaming.
	\warning If HEVC is used, make sure the receiving host is compatible with H265 decoding (Pascal NVIDIA card or newer). If not, prefer to use H264 since every compatible NVIDIA card supports H264 decoding
		*/
	enum SL_STREAMING_CODEC codec;

	/**
	\brief Defines the port used for streaming.
	\warning Port must be an even number. Any odd number will be rejected.
	\warning Port must be opened.
	 */
	unsigned short port;

	/**
	\brief Defines the streaming bitrate in Kbits/s
	 *
	 *
	 *  | STREAMING_CODEC  | Resolution   | FPS   | bitrate (kbps) |
	 *  |------------------|--------------|-------|----------------|
	 *  | H264             |  HD2K        |   15  |     8500       |
	 *  | H264             |  HD1080      |   30  |    12500       |
	 *  | H264             |  HD720       |   60  |     7000       |
	 *  | H265             |  HD2K        |   15  |     7000       |
	 *  | H265             |  HD1080      |   30  |    11000       |
	 *  | H265             |  HD720       |   60  |     6000       |

	\note Available range : [1000 - 60000]
	 */
	unsigned int bitrate;

	/**
	\brief Defines the gop size in number of frames.
	\note if value is set to -1, the gop size will last at maximum 2 seconds, depending on camera fps.
	\note The gop size determines the maximum distance between IDR/I-frames. Very high GOP size will result in slightly more efficient compression, especially on static scenes. But latency will increase.
	\note maximum value: 256
	 */
	int gop_size;

	/**
	\brief Enable/Disable adaptive bitrate
	\note Bitrate will be adjusted depending the number of packet dropped during streaming.
	\note if activated, bitrate can vary between [bitrate/4, bitrate]
	\warning Currently, the adaptive bitrate only works when "sending" device is a NVIDIA Jetson (X1,X2,Xavier,Nano)
	 */
	bool adaptative_bitrate;

	/**
	\brief Defines a single chunk size
	\note Stream buffers are divided in X number of chunk where each chunk is "chunk_size" bytes long.
	\note Default value is 16084. You can lower this value if network generates a lot of packet lost : this will
	generates more chunk for a single image, but each chunk sent will be lighter to avoid inside-chunk corruption.
	Increasing this value can decrease latency.
	\note Available range : [4096 - 65000]
	 */
	unsigned short chunk_size;


	/**
	 \brief defines the target framerate for the streaming output.
	 \warning This framerate must be below or equal to the camera framerate. Allowed framerates are 15,30, 60 or 100 if possible.
	 Any other values will be discarded and camera FPS will be taken.
	 \ndefault : 0, meaning that the camera framerate will be taken
	 */
	unsigned int target_framerate;
};

/**
\struct StreamingProperties
\brief Properties of a streaming device
  */
struct SL_StreamingProperties {
	/**
	the streaming IP of the device
	 */
	unsigned char ip[16];

	/**
	the streaming port
	 */
	unsigned short port;

	/**
	the serial number of the streaming device
	 */
	unsigned int serial_number;

	/**
	the current bitrate of encoding of the streaming device
	 */
	int current_bitrate;

	/**
	the current codec used for compression in streaming device
	 */
	enum SL_STREAMING_CODEC codec;
};


struct SL_SpatialMappingParameters {
	/**
	\brief Spatial mapping resolution in meters. Should fit \ref allowed_resolution (Default is 0.05f).
	 */
	float resolution_meter;
	/**
	\brief Depth range in meters.
	Can be different from the value set by \ref setDepthMaxRangeValue.
	Set to 0 by default. In this case, the range is computed from resolution_meter
	and from the current internal parameters to fit your application.
	 */
	float range_meter;
	/**
	\brief Set to true if you want to be able to apply the texture to your mesh after its creation.

	\note This option will consume more memory.
	\note This option is only available for \ref SPATIAL_MAP_TYPE_MESH
	 */
	bool save_texture;
	/**
	\brief Set to false if you want to ensure consistency between the mesh and its inner chunk data (default is false).

	\note Updating the mesh is time-consuming. Setting this to true results in better performance.
	 */
	bool use_chunk_only;
	/**
	\brief The maximum CPU memory (in MB) allocated for the meshing process (default is 2048).
	 */
	int max_memory_usage;
	/**
	\brief Specify if the order of the vertices of the triangles needs to be inverted. If your display process does not handle front and back face culling, you can use this to correct it.

	\note This option is only available for \ref SPATIAL_MAP_TYPE_MESH
	 */
	bool reverse_vertex_order;
	/**
	\brief The type of spatial map to be created. This dictates the format that will be used for the mapping(e.g. mesh, point cloud). See \ref SPATIAL_MAP_TYPE
	 */
	enum SL_SPATIAL_MAP_TYPE map_type;
};


#if WITH_OBJECT_DETECTION
#define MAX_NUMBER_OBJECT 75
#define MAX_TRAJECTORY_SIZE 200

/** Set the batch trajectory parameters*/
struct SL_BatchParameters
{
	/**
	\brief Defines if the Batch option in the object detection module is enabled. Batch queueing system provides:
	 *  - Deep-Learning based re-identification
	 *  - Trajectory smoothing and filtering
	 \note To activate this option, enable must be set to true.
	 */
	bool enable;
	/**
	\brief Max retention time in seconds of a detected object. After this time, the same object will mostly have a different ID.
	 */
	float id_retention_time;
	/**
	\brief Trajectories will be output in batch with the desired latency in seconds.
	During this waiting time, re-identification of objects is done in the background.
	Specifying a short latency will limit the search ( falling in timeout) for previously seen object IDs but will be closer to real time output.
	Specifying a long latency will reduce the change of timeout in Re-ID but increase difference with live output.
	 */
	float latency;
};

/**
\brief contains AI model status.
*/
struct SL_AI_Model_status {
	bool downloaded; // the model file is currently present on the host
	bool optimized; // an engine file with the expected architecure is found.
};

/**
Sets the object detection parameters.
*/
struct SL_ObjectDetectionParameters
{
	/**
	\brief Defines if the object detection is synchronized to the image or runs in a separate thread.
	If set to true, the detection is run for every grab, otherwise, the thread runs at its own speed, which can lead to new detection once in a while.
	*/
	bool image_sync; // data synchronized
	/**
	\brief Defines if the object detection will track objects across images flow
	 */
	bool enable_tracking;
	/**
	\brief Defines if the mask object will be computed
	 */
	bool enable_mask_output;
	/**
	\brief Enable human pose estimation with skeleton keypoints output (SL_DETECTION_MODEL).
	 */
	enum  SL_DETECTION_MODEL model;
	/**
	\brief Defines if the body fitting will be applied
	 */
	bool enable_body_fitting;
	/**
	 * \brief Defines the body format outputed by the sdk when \ref retrieveObjects is called.
	 *
	*/
	enum SL_BODY_FORMAT body_format;
	/**
	\brief Defines a upper depth range for detections.
	  * \n Defined in \ref UNIT set at \ref SL_Camera::open.
	  * \n Default value is set to \ref SL_Initparameters::depth_maximum_distance (can not be higher).
	  */
	float max_range;
	/**
	 \brief Batching system parameters.
	 Batching system (introduced in 3.5) performs short-term re-identification with deep learning and trajectories filtering.
	 * \n BatchParameters::enable need to be true to use this feature (by default disabled)
	 */
	struct SL_BatchParameters batch_parameters;
	/**
	\brief Defines the filtering mode that should be applied to raw detections.
	*/
	enum SL_OBJECT_FILTERING_MODE filtering_mode;
};

/**
\brief Lists available object class
 */
enum SL_OBJECT_CLASS
{
	SL_OBJECT_CLASS_PERSON,          /**< For people detection */
	SL_OBJECT_CLASS_VEHICLE,         /**< For vehicle detection. It can be cars, trucks, buses, motorcycles etc */
	SL_OBJECT_CLASS_BAG,             /**< For bag detection (backpack, handbag, suitcase) */
	SL_OBJECT_CLASS_ANIMAL,          /**< For animal detection (cow, sheep, horse, dog, cat, bird, etc) */
	SL_OBJECT_CLASS_ELECTRONICS,     /**< For electronic device detection (cellphone, laptop, etc) */
	SL_OBJECT_CLASS_FRUIT_VEGETABLE, /**<  For fruit and vegetable detection (banana, apple, orange, carrot, etc) */
	SL_OBJECT_CLASS_SPORT,			 /**<  For sport related objects (sports ball etc) */
	SL_OBJECT_CLASS_LAST
};


/**
Sets the object detection runtime parameters.
*/
struct SL_ObjectDetectionRuntimeParameters
{
	/**
	\brief Defines the confidence threshold: interval between 1 and 99. A confidence of 1 meaning a low
	 *  threshold, more uncertain objects and 99 very few but very precise objects.
	 * If the scene contains a lot of objects, increasing the confidence can slightly speed up the process, since every object instances are tracked.
	 *
	 * Default confidence threshold value, used as a fallback when ObjectDetectionRuntimeParameters::object_class_detection_confidence_threshold is partially set
	 */
	float detection_confidence_threshold;
	/**
	\brief Select which object types to detect and track. By default all classes are tracked.
	 * Fewer object types can slightly speed up the process, since every objects are tracked.
	 * Only the selected classes in the vector will be output.

	 In order to get all the available classes, the filter vector must be empty.
	 */
	int object_class_filter[(int)SL_OBJECT_CLASS_LAST];
	/**
	\brief Defines a detection threshold for each classes, can be empty for some classes,
	 * ObjectDetectionRuntimeParameters::detection_confidence_threshold will be taken as fallback/default value
	 */
	int object_confidence_threshold[(int)SL_OBJECT_CLASS_LAST];
};

/**
Contains data of a detected object such as its bounding_box, label, id and its 3D position.
*/
struct SL_ObjectData
{
	/**
		\brief Object identification number, used as a reference when tracking the object through the frames
		\note Only available if \ref ObjectDetectionParameters::enable_tracking is activated else set to -1.
		 */
	int id;
	/**
	\brief Unique ID to help identify and track AI detections. Can be either generated externally, or using \ref sl_generate_unique_id() or left empty
	*/
	unsigned char unique_object_id[37];

	/**
	\brief Object label, forwarded from \ref CustomBoxObjects when using \ref SL_DETECTION_MODEL_CUSTOM_BOX_OBJECTS
	*/
	int raw_label;
	/**
	\brief Object category. Identify the object type (sl::OBJECT_CLASS)
	 */
	enum  SL_OBJECT_CLASS label;
	/**
	\brief Object subclass (sl::OBJECT_SUBCLASS)
	 */
	enum  SL_OBJECT_SUBCLASS sublabel;
	/**
	\brief Defines the object tracking state (sl::OBJECT_TRACKING_STATE).
	 */
	enum  SL_OBJECT_TRACKING_STATE tracking_state;
	/**
	\brief Defines the object action state (sl::OBJECT_ACTION_STATE).
	 */
	enum  SL_OBJECT_ACTION_STATE action_state;
	/**
	\brief Defines the detection confidence value of the object.
	 * From 0 to 100, a low value means the object might not be localized perfectly or the label (OBJECT_CLASS) is uncertain.
	 */
	float confidence;

	//Mask
	int* mask;
	//int* mask; //IntPtr to an sl::Mat object.

	//Image
	struct SL_Vector2 bounding_box_2d[4];
	/**
	\brief Defines the object 3D centroid.
	 * Defined in \ref sl:InitParameters::UNIT, expressed in \ref RuntimeParameters::measure3D_reference_frame.
	 */
	struct SL_Vector3 position;
	/**
	 * \brief 3D head centroid.
	 * Defined in \ref sl:InitParameters::UNIT, expressed in \ref RuntimeParameters::measure3D_reference_frame.
	  \note Not available with DETECTION_MODEL::MULTI_CLASS_BOX.
	 */
	struct SL_Vector3 head_position;
	/**
	\brief Defines the object 3D velocity
	 * Defined in \ref sl:InitParameters::UNIT / seconds, expressed in \ref RuntimeParameters::measure3D_reference_frame.
	 */
	struct SL_Vector3 velocity;
	/**
	 * \brief 3D bounding box of the person represented as eight 3D points
	 * Defined in \ref sl:InitParameters::UNIT, expressed in \ref RuntimeParameters::measure3D_reference_frame.
	 *
	 * \code
		   1 ------ 2
		  /        /|
		 0 ------ 3 |
		 | Object | 6
		 |        |/
		 4 ------ 7
	 \endcode
	 */
	struct SL_Vector3 bounding_box[8];
	/**
	 * \brief bounds the head with eight 3D points.
	 * Defined in \ref sl:InitParameters::UNIT, expressed in \ref RuntimeParameters::measure3D_reference_frame.
	  \note Not available with DETECTION_MODEL::MULTI_CLASS_BOX.
	 */
	struct SL_Vector3 head_bounding_box[8];
	/**
	 * \brief A set of useful points representing the human body, expressed in 2D, respect to the original image resolution.
	 * We use a classic 18 points representation, the points semantic and order is given by BODY_PARTS.
	 * Expressed in pixels on the original image resolution, [0,0] is the top left corner.
	  \note Not available with DETECTION_MODEL::MULTI_CLASS_BOX.
	  \warning in some cases, eg. body partially out of the image, some keypoint can not be detected, they will have negatives coordinates.
	 */
	struct SL_Vector2 keypoint_2d[34];
	/**
	 * \brief A set of useful points representing the human body, expressed in 3D.
	 * We use a classic 18 points representation, the points semantic and order is given by BODY_PARTS.
	 * Defined in \ref sl:InitParameters::UNIT, expressed in \ref RuntimeParameters::measure3D_reference_frame.
	  \note Not available with DETECTION_MODEL::MULTI_CLASS_BOX.
	  \warning in some cases, eg. body partially out of the image or missing depth data, some keypoint can not be detected, they will have non finite values.
	 */
	struct SL_Vector3 keypoint[34];
	/**
	\brief the covariance matrix of the 3d position, represented by its upper triangular matrix value
	 * \code
		 = [p0, p1, p2]
		   [p1, p3, p4]
		   [p2, p4, p5]
	  \endcode
	  where pi is position_covariance[i]
	 */
	float position_covariance[6];
	/**
	 * \brief Per keypoint detection confidence, can not be lower than the \ref ObjectDetectionRuntimeParameters::detection_confidence_threshold.
	  \note Not available with DETECTION_MODEL::MULTI_CLASS_BOX.
	  \warning in some cases, eg. body partially out of the image or missing depth data, some keypoint can not be detected, they will have non finite values.
	 */
	float keypoint_confidence[34];
	/**
	\brief Per keypoint local position (the position of the child keypoint with respect to its parent expressed in its parent coordinate frame)
	\note it is expressed in sl::REFERENCE_CAMERA or sl::REFERENCE_WORLD
	Not available with DETECTION_MODEL::MULTI_CLASS_BOX.
	*/
	struct SL_Vector3 local_position_per_joint[34];
	/**
		\brief Per keypoint local orientation
		\note the orientation is represented by a quaternion which is stored in sl::float4 (sl::float4 q = sl::float4(qx,qy,qz,qw);)
		Not available with DETECTION_MODEL::MULTI_CLASS_BOX.
	*/
	struct SL_Quaternion local_orientation_per_joint[34];
	/**
		\brief global root orientation of the skeleton. The orientation is also represented by a quaternion with the same format as \ref local_orientation_per_joint
	*/
	struct SL_Quaternion global_root_orientation;
};

/**
\brief Container to store the externally detected objects. The objects can be ingested
 * using \ref sl_ingest_custom_box_objects() functions to extract 3D information and tracking over time
 */

struct SL_CustomBoxObjectData {
	/**
	\brief Unique ID to help identify and track AI detections. Can be either generated externally, or using \ref generate_unique_id() or left empty
	*/
	char unique_object_id[37];
	/**
	 * \brief 2D bounding box represented as four 2D points starting at the top left corner and rotation clockwise.
	 * Expressed in pixels on the original image resolution, [0,0] is the top left corner.
	 * \code
		A ------ B
		| Object |
		D ------ C
	\endcode
	*/
	struct SL_Vector2 bounding_box_2d[4];

	/**
	\brief Object label, this information is passed-through and can be used to improve object tracking
	*/
	int label;

	/**
	\brief Detection confidence. Should be [0-1]. It can be used to improve the object tracking
	*/
	float probability;
	/**
	\brief Provide hypothesis about the object movements(degrees of freedom) to improve the object tracking
		* true: means 2 DoF projected alongside the floor plane, the default for object standing on the ground such as person, vehicle, etc
		* false : 6 DoF full 3D movements are allowed
    */
	bool is_grounded;
};

/**
Contains the result of the object detection module.
*/
struct SL_Objects
{
	/**
	\brief Number of detected objects. Used to iterate through the object_list array.
	 */
	int nb_object;
	/**
	\brief Defines the timestamp corresponding to the frame acquisition.
	 * This value is especially useful for the async mode to synchronize the data.
	 */
	unsigned long long image_ts;
	/**
	\brief Defined if the object list has already been retrieved or not.
	 */
	int is_new;
	/**
	\brief Defined if both the object tracking and the world orientation has been setup.
	 */
	int is_tracked;
	/**
	\brief Detection model used (SL_DETECTION_MODEL).
	 */
	enum  SL_DETECTION_MODEL detection_model;
	/**
	\brief The list of detected objects
	 */
	struct SL_ObjectData object_list[MAX_NUMBER_OBJECT];
};

/**
* \brief Objects batch structure
*/
struct SL_ObjectsBatch {
	int nb_data;
	int id;
	enum SL_OBJECT_CLASS label;
	enum SL_OBJECT_SUBCLASS sublabel;
	enum SL_OBJECT_TRACKING_STATE tracking_state;
	struct SL_Vector3 positions[MAX_TRAJECTORY_SIZE];
	float position_covariances[MAX_TRAJECTORY_SIZE][6];
	struct SL_Vector3 velocities[MAX_TRAJECTORY_SIZE];
	unsigned long long timestamps[MAX_TRAJECTORY_SIZE];
	struct SL_Vector2 bounding_boxes_2d[MAX_TRAJECTORY_SIZE][4];
	struct SL_Vector3 bounding_boxes[MAX_TRAJECTORY_SIZE][8];
	float confidences[MAX_TRAJECTORY_SIZE];
	enum SL_OBJECT_ACTION_STATE action_states[MAX_TRAJECTORY_SIZE];
	struct SL_Vector2 keypoints_2d[MAX_TRAJECTORY_SIZE][18];
	struct SL_Vector3 keypoints[MAX_TRAJECTORY_SIZE][18];
	struct SL_Vector2 head_bounding_boxes_2d[MAX_TRAJECTORY_SIZE][4];
	struct SL_Vector3 head_bounding_boxes[MAX_TRAJECTORY_SIZE][8];
	struct SL_Vector3 head_positions[MAX_TRAJECTORY_SIZE];
	float keypoints_confidences[MAX_TRAJECTORY_SIZE][18];
};

/*
Defines a 2D rectangle with top-left corner coordinates and width/height in pixels.
*/
struct SL_Rect
{
	int x;
	int y;
	int width;
	int height;
};

#endif

#endif
