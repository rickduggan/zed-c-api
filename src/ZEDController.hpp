#ifndef __ZED_CONTROLLER_H__
#define __ZED_CONTROLLER_H__

#include <memory>
#include <sl/Camera.hpp>
#ifdef _WIN32
#include <Windows.h>
#endif
#include <cuda.h>
#include "cuda_runtime.h"
#include "sl/c_api/types_c.h"

enum TRACKING_TYPE {
    PATH,
    POSE
};

static std::mutex globalmutex;

class ZEDController {
public:
    ZEDController(int i);
    ~ZEDController();

    static ZEDController* get(int i) {
        if (!instance[i]) // Only allow one instance of class to be generated.
            instance[i] = new ZEDController(i);

        return instance[i];
    }

    static void destroyInstance(int i) {
        if (!instance[i]) // Only allow one instance of class to be generated.
            delete instance[i];
        instance[i] = nullptr;

    }

    static bool isNotCreated(int i) {
        return (instance[i] == nullptr);
    }

    inline bool isNull() {
		return !zed.isOpened();
    }
    void createCamera(bool verbose);

    void destroy();

    int initFromUSB(SL_InitParameters *params, const char* outputFile, const char* opt_settings_path, const char* opencv_calib_path);
    int initFromSVO(SL_InitParameters *params, const char* pathSVO, const char* outputFile, const char* opt_settings_path, const char* opencv_calib_path);
    int initFromStream(SL_InitParameters *params, const char* ip, int port, const char* outputFile, const char* opt_settings_path, const char* opencv_calib_path);

    sl::POSITIONAL_TRACKING_STATE getPoseArray(float* pose, int mat_type);

    inline unsigned int getWidth() {
        return width;
    }

    inline unsigned int getHeight() {
        return height;
    }

    sl::CameraInformation* getSLCameraInformation();
    SL_CalibrationParameters* getCalibrationParameters(bool raw);
    SL_SensorsConfiguration* getSensorsConfiguration();
	SL_CameraInformation* getCameraInformation(int width, int height);
    sl::MODEL getCameraModel();
    SL_InitParameters* getInitParameters();
    SL_RuntimeParameters* getRuntimeParameters();
    SL_PositionalTrackingParameters* getPositionalTrackingParameters();
    sl::ERROR_CODE grab(SL_RuntimeParameters *runtimeParameters);


    /************Recording******************/
    sl::ERROR_CODE enableRecording(const char* path, sl::SVO_COMPRESSION_MODE compressionMode, unsigned int bitrate, int targetFPS, bool transcode);
    void disableRecording();
	SL_RecordingStatus* getRecordingStatus();
	SL_RecordingParameters* getRecordingParameters();


    /************Tracking*******************/
    sl::ERROR_CODE enableTracking(const SL_Quaternion *initial_world_rotation, const SL_Vector3 *initial_world_position, bool enable_area_memory, bool enable_pose_smoothing, bool set_floor_as_origin, bool set_as_static, bool enable_imu_fusion, const char* area_file_path);
    sl::POSITIONAL_TRACKING_STATE getPosition(SL_Quaternion *quat, SL_Vector3 *vec, sl::REFERENCE_FRAME mat_type);
    sl::POSITIONAL_TRACKING_STATE getPosition(SL_Quaternion *quat, SL_Vector3 *vec, SL_Vector3 *offset, SL_Quaternion *offsetRotation, int type);
    sl::POSITIONAL_TRACKING_STATE getPosition(SL_PoseData *pose, int reference_frame);

    sl::ERROR_CODE resetTracking(SL_Quaternion rotation, SL_Vector3 translation);
    sl::ERROR_CODE resetTrackingWithOffset(SL_Quaternion rotation, SL_Vector3 translation, SL_Quaternion offsetRotation, SL_Vector3 offsetTranslation);
    void disableTracking(const char *path);


    sl::ERROR_CODE setIMUPriorOrientation(SL_Quaternion rotation);
    sl::ERROR_CODE getIMUOrientation(SL_Quaternion *quaternion, int time_reference);
    sl::ERROR_CODE getSensorData(SL_SensorData *sensorData, int time_reference);

    /***********Spatial mapping *********/
    sl::ERROR_CODE enableSpatialMapping(struct SL_SpatialMappingParameters mapping_param);
    void requestMeshAsync();
    sl::ERROR_CODE updateMesh(int* numVertices, int* numTriangles, int* numSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, const int maxSubmesh);
    sl::ERROR_CODE extractWholeSpatialMap();
    bool filterMesh(sl::MeshFilterParameters::MESH_FILTER filterParams, int* numVertices, int* numTriangles, int* numUpdatedSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, const int maxSubmesh);
    sl::ERROR_CODE retrieveMesh(float* vertices, int* triangles, const int maxSubmesh, float* uvs, unsigned char* texturePtr);
    sl::ERROR_CODE updateChunks(int* numVertices, int* numTriangles, int* numSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, const int maxSubmesh);
    sl::ERROR_CODE retrieveChunks(const int maxSubmesh, float* vertices, int* triangles);

    sl::ERROR_CODE updateFusedPointCloud(int* numPointsTot);
    sl::ERROR_CODE retrieveFusedPointCloud(float* vertices);

    void disableSpatialMapping();
	SL_SpatialMappingParameters* getSpatialMappingParameters();
    bool saveMesh(const char* filename, sl::MESH_FILE_FORMAT format);
    bool savePointCloud(const char* filename, sl::MESH_FILE_FORMAT format);
    bool loadMesh(const char* filename, int* numVertices, int* numTriangles, int* numSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, const int maxSubmesh, int* texturesSize = nullptr);
    bool applyTexture(int* numVertices, int* numTriangles, int* numUpdatedSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, int* texturesSize, const int maxSubmesh);
    void mergeChunks(int size, int* numVertices, int* numTriangles, int* numUpdatedSubmeshes, int* updatedIndices, int* numVerticesTot, int* numTrianglesTot, const int maxSubmesh);
    sl::float3 getGravityEstimation();

    void lock() {
        sdk_mutex.lock();
    }

    bool try_lock() {
        return sdk_mutex.try_lock();
    }

    void unlock() {
        sdk_mutex.unlock();
    }

    /*********************************
     ******* Plane Detection *********
     *************************************/
    SL_PlaneData* findFloorPlane(SL_Quaternion *resetQuaternion, SL_Vector3* resetTranslation, SL_Quaternion priorQuaternion, SL_Vector3 priorTranslation);
    SL_PlaneData* findPlaneAtHit(SL_Vector2 pixel, bool thres);
    sl::ERROR_CODE convertCurrentFloorPlaneToChunk(float* vertices, int* triangles, int* numVerticesTot, int* numTrianglesTot);
    sl::ERROR_CODE convertCurrentHitPlaneToChunk(float* vertices, int* triangles, int* numVerticesTot, int* numTrianglesTot);


    /*********************************
     ******* Streaming module (sdk) *********
     *************************************/
    sl::ERROR_CODE enableStreaming(sl::STREAMING_CODEC codec, unsigned int bitrate, unsigned short port, int gopSize, bool adaptativeBitrate, int chunk_size, int target_framerate);
    bool isStreamingEnabled();
    void disableStreaming();
	SL_StreamingParameters* getStreamingParameters();

    /*****************************
     ****** Save fct utils ********
     ******************************/
    sl::ERROR_CODE saveCurrentImage(sl::VIEW, const char* filename);
    sl::ERROR_CODE saveCurrentDepth(int side, const char* filename);
    sl::ERROR_CODE saveCurrentPointCloud(int side, const char* filename);

    /*********************************
     ******* Object Detection (sdk) *********
     *************************************/
#if WITH_OBJECT_DETECTION
    sl::ERROR_CODE enableObjectDetection(SL_ObjectDetectionParameters* objparams);
	SL_ObjectDetectionParameters* getObjectDetectionParameters();
    void pauseObjectDetection(bool status);
    void disableObjectDetection();
	sl::ERROR_CODE ingestCustomBoxObjectData(int nb_objects, SL_CustomBoxObjectData* objects_in);
    sl::ERROR_CODE retrieveObjectDetectionData(SL_ObjectDetectionRuntimeParameters* objruntimeparams, SL_Objects* data);
    sl::ERROR_CODE updateObjectsBatch(int* nb_batches);
    sl::ERROR_CODE getObjectsBatchData(int index, struct SL_ObjectsBatch *objs);
	sl::ERROR_CODE getObjectsBatchDataCSharp(int index, int* num_data, int* id, int* label, int* sublabel, int* tracking_state,
		SL_Vector3 positions[MAX_TRAJECTORY_SIZE], float position_covariances[MAX_TRAJECTORY_SIZE][6], SL_Vector3 velocities[MAX_TRAJECTORY_SIZE], unsigned long long timestamps[MAX_TRAJECTORY_SIZE],
		SL_Vector2 bounding_boxes_2d[MAX_TRAJECTORY_SIZE][4], SL_Vector3 bounding_boxes[MAX_TRAJECTORY_SIZE][8], float confidences[MAX_TRAJECTORY_SIZE], int action_states[MAX_TRAJECTORY_SIZE],
		SL_Vector2 keypoints_2d[MAX_TRAJECTORY_SIZE][18], SL_Vector3 keypoints[MAX_TRAJECTORY_SIZE][18], SL_Vector2 head_bounding_boxes_2d[MAX_TRAJECTORY_SIZE][4], SL_Vector3 head_bounding_boxes[MAX_TRAJECTORY_SIZE][8],
		SL_Vector3 head_positions[MAX_TRAJECTORY_SIZE], float keypoints_confidences[MAX_TRAJECTORY_SIZE][18]);
#endif

    int getInputType();

    /**************************************************/
    sl::Camera zed;

private:
    int open();

    bool cameraOpened = false;

    sl::CalibrationParameters camParameters;
    sl::CameraInformation camInformations;
    SL_PlaneData currentFloorPlane;
    SL_PlaneData currentPlaneAtHit;
    sl::Plane currentFloorPlaneSDK;
    sl::Plane currentPlaneAtHitSDK;
    bool activTracking;

    int input_type;
    unsigned int width;
    unsigned int height;
    sl::RESOLUTION resolution;

    bool stabilizer = false;
    bool verbosity = false;

    sl::Transform previousPath;
    sl::RuntimeParameters runtimeParams;
    CUstream stream;

    /*MESH*/
    bool isMeshUpdated = false;
    bool areTextureReady = false;
    bool isTextured = false;

    bool saveTexture;
    sl::Mesh mesh;
    sl::FusedPointCloud pointCloudFused;
    sl::ERROR_CODE OpeningErrorCode;
    std::thread threadOpening;
    sl::InitParameters initParams;
    CUcontext context;

    std::mutex sdk_mutex;
    static ZEDController* instance[MAX_CAMERA_PLUGIN];


    int grab_count_t = 0;
    sl::Timestamp initial_Timestamp = 0;

    std::vector<sl::ObjectsBatch> trajectories;
    bool isTrajectoriesUpdated = false;

    std::vector<sl::Plane> currentPlanes;
    FILE* fskl = nullptr;

    int camera_ID = 0;

    sl::DETECTION_MODEL current_detection_model;

};

#endif
