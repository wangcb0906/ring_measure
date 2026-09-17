#pragma once

#include "cali.hpp"

#if defined(_MSC_VER) || defined(__MINGW32__)
    #define EXPORT_API __declspec(dllexport)
#else
    #define EXPORT_API __attribute__((visibility("default")))
#endif

namespace rm = RingMeasure;
constexpr int RM_MAX_PATH_LEN = 256;

#ifdef __cplusplus
extern "C" {
#endif
   
    /**
     * @brief Creates a new instance of the RingCalibration class.
     * @return Pointer to the newly created RingCalibration instance.
     */
    EXPORT_API rm::RingCalibration* RM_RingCalibration_New();

    /**
     * @brief Deletes an instance of the RingCalibration class.
     * @param ptr Pointer to the RingCalibration instance to delete.
     */
    EXPORT_API void RM_RingCalibration_Delete(rm::RingCalibration* ptr);

    /// @brief Interop struct for sensor configuration, designed for C-style interop.
    struct SensorCfgInterop {
        int scan_num;
        int scan_res;
        int valid_len;
        float z_pitch;
        float z_oft;
        float x_pitch;
        char unit[32];     ///< Replaced std::string with fixed char array
        float mount_height;
    };
   
    /// @brief Interop struct for system configuration, designed for C-style interop.
    struct SystemCfgInterop {
        rm::RobotCfg robot;      ///< RobotCfg is already blittable (std::array, Point2f, floats)
        SensorCfgInterop sensor; ///< Uses the flat interop sensor config
    };
   
    /**
     * @brief Sets the system configuration for the ring calibration.
     * @param ptr Pointer to the ring calibration object.
     * @param interopSystem Pointer to the interop struct containing the system configuration.
     * @return bool True if the system configuration was set successfully, false otherwise.
     */
    EXPORT_API bool RM_RingCalibration_SetSystem(rm::RingCalibration* ptr, const SystemCfgInterop* interopSystem);

    /**
     * @brief Gets the system configuration from the ring calibration.
     * @param ptr Pointer to the ring calibration object.
     * @param outSystem Pointer to the interop struct to store the system configuration.
     * @return bool True if the system configuration was retrieved successfully, false otherwise.
     */
    EXPORT_API bool RM_RingCalibration_GetSystem(
        const rm::RingCalibration* ptr,
        SystemCfgInterop* outSystem);
   
    /// @brief Interop struct for ring configuration.
    struct RingCfgInterop {
        char spk_form[8];  ///< Replaced std::string with fixed char array
        bool isTured;
        char type[8];      ///< Replaced std::string with fixed char array
        float mke_top;
        float mke_btm;
        float mke_mid;
        float dI;
        float dA;
        float h;
    };

    /// @brief Interop struct for image configuration.
    struct ImgCfgInterop {
        char path[RM_MAX_PATH_LEN];
        char imgs[rm::NUM][RM_MAX_PATH_LEN];     ///< Flat array for filenames per location
        char luma_imgs[rm::NUM][RM_MAX_PATH_LEN]; ///< Flat array for luminance filenames per location
    };
   
    /// @brief Interop struct for measurement configuration.
    struct MeasureCfgInterop {
        ImgCfgInterop img;
        RingCfgInterop ring;                        ///< Assumed to be a blittable struct already
        std::array<rm::Point2f, rm::NUM> robot_pos; ///< Safe! Stays as-is
    };

    /**
     * @brief Sets the measurement configuration for the ring calibration.
     * @param ptr Pointer to the ring calibration object.
     * @param interopCfg Pointer to the interop struct containing the measurement configuration.
     * @return bool True if the measurement configuration was set successfully, false otherwise.
     */
    EXPORT_API bool RM_RingCalibration_SetMeasure(
        rm::RingCalibration* ptr,
        const MeasureCfgInterop* interopCfg);

    /**
     * @brief Gets the measurement configuration from the ring calibration object.
     * @param ptr Pointer to the ring calibration object.
     * @param outInteropCfg Pointer to the pre-allocated interop struct to be filled.
     * @return true if successful, false otherwise.
     */
    EXPORT_API bool RM_RingCalibration_GetMeasure(
        const rm::RingCalibration* ptr,
        MeasureCfgInterop* outInteropCfg);

    /**
     * @brief Sets the tuning parameters for the ring calibration.
     * @param ptr Pointer to the ring calibration object.
     * @param param Pointer to the tuning parameters struct.
     * @return bool True if the parameters were set successfully, false otherwise.
     */
    EXPORT_API bool RM_RingCalibration_SetParam(
        rm::RingCalibration* ptr,
        const rm::ParamTune* param);

    /**
     * @brief Gets the tuning parameters from the ring calibration object.
     * @param ptr Pointer to the ring calibration object.
     * @param outParam Pointer to the pre-allocated struct to store the tuning parameters.
     * @return bool True if successful, false otherwise.
     */
    EXPORT_API bool RM_RingCalibration_GetParam(
        const rm::RingCalibration* ptr,
        rm::ParamTune* outParam);

    /**
     * @brief Executes the full calibration routine for a specific scan location.
     * @param ptr Pointer to the ring calibration object.
     * @param scan Target scan location.
     * @param path Null-terminated string representing the directory path.
     * @return true if the operation was successful, false otherwise.
     */
    EXPORT_API bool RM_RingCalibration_Calibration(
        rm::RingCalibration* ptr,
        rm::ScanLocation scan,
        const char* path);

    /**
     * @brief Executes the verification routine for a specific scan location.
     * @param ptr Pointer to the ring calibration object.
     * @param scan Target scan location.
     * @param path Null-terminated string representing the directory path.
     * @return true if the operation was successful, false otherwise.
     */
    EXPORT_API bool RM_RingCalibration_Verification(
        rm::RingCalibration* ptr,
        rm::ScanLocation scan,
        const char* path);
   
    /**
     * @brief Retrieves the system correction for a specific ring calibration.
     * @param ptr Pointer to the ring calibration object.
     * @param outBuffer Pointer to the output buffer where the correction will be stored.
     * @param bufferSize Size of the output buffer.
     * @return bool True if the system correction was retrieved successfully, false otherwise.
     */
    EXPORT_API bool RM_RingCalibration_GetSystemCorrection(
        const rm::RingCalibration* ptr,
        rm::ScanCfg* outBuffer,
        int bufferSize);
   
    /**
     * @brief Sets the system correction for a specific ring calibration.
     * @param ptr Pointer to the ring calibration object.
     * @param correctionArray Pointer to the array of correction values.
     * @param arraySize Size of the correction array.
     * @return bool True if the system correction was set successfully, false otherwise.
     */
    EXPORT_API bool RM_RingCalibration_SetSystemCorrection(
        rm::RingCalibration* ptr,
        const rm::ScanCfg* correctionArray,
        int arraySize);

    /**
     * @brief Configures the system logger and binds PCL logging to stderr for a specific ring calibration.
     * @param ptr Pointer to the ring calibration object.
     * @param logger_name Pointer to the null-terminated string for the logger name.
     * @param log_path Pointer to the null-terminated string for the log file path.
     * @param overwrite True to overwrite an existing log file, false to append.
     * @return bool True if the logger was configured successfully, false otherwise.
     */
    EXPORT_API bool RM_RingCalibration_SetLogger(
        rm::RingCalibration* ptr,
        const char* logger_name,
        const char* log_path,
        bool overwrite);

    /**
     * @brief Retrieves the active logger name and log file path.
     * @param ptr Pointer to the ring calibration object.
     * @param name_buffer Buffer to receive the null-terminated logger name.
     * @param name_buffer_size Size of the name buffer in characters.
     * @param path_buffer Buffer to receive the null-terminated log file path.
     * @param path_buffer_size Size of the path buffer in characters.
     * @return bool True if retrieved successfully, false otherwise.
     *         If the buffers are too small, the function will return false.
     */
    EXPORT_API bool RM_RingCalibration_GetLogger(
        rm::RingCalibration* ptr,
        char* name_buffer, int name_buffer_size,
        char* path_buffer, int path_buffer_size);

#ifdef __cplusplus
}
#endif