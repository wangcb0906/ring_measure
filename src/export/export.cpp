#include "export.h"
#include <cstring>
#include <filesystem>

extern "C" {

    EXPORT_API rm::RingCalibration* RM_RingCalibration_New() {
        return new rm::RingCalibration();
    }

    EXPORT_API void RM_RingCalibration_Delete(rm::RingCalibration* ptr) {
        if (ptr) {
            delete ptr;
        }
    }

    EXPORT_API bool RM_RingCalibration_SetSystem(
        rm::RingCalibration* ptr, 
        const SystemCfgInterop* interopSystem) 
    {
        if (!ptr || !interopSystem) {
            return false;
        }

        rm::SystemCfg richSystem;
        richSystem.robot = interopSystem->robot;

        richSystem.sensor.scan_num = interopSystem->sensor.scan_num;
        richSystem.sensor.scan_res = interopSystem->sensor.scan_res;
        richSystem.sensor.valid_len = interopSystem->sensor.valid_len;
        richSystem.sensor.z_pitch = interopSystem->sensor.z_pitch;
        richSystem.sensor.z_oft = interopSystem->sensor.z_oft;
        richSystem.sensor.x_pitch = interopSystem->sensor.x_pitch;
        richSystem.sensor.mount_height = interopSystem->sensor.mount_height;

        richSystem.sensor.unit = interopSystem->sensor.unit;

        ptr->setSystem(richSystem);
        return true;
    }

    EXPORT_API bool RM_RingCalibration_GetSystem(
        const rm::RingCalibration* ptr,
        SystemCfgInterop* outSystem)
    {
        if (!ptr || !outSystem) {
            return false;
        }

        rm::SystemCfg richSystem = ptr->getSystem();

        outSystem->robot = richSystem.robot;

        outSystem->sensor.scan_num = richSystem.sensor.scan_num;
        outSystem->sensor.scan_res = richSystem.sensor.scan_res;
        outSystem->sensor.valid_len = richSystem.sensor.valid_len;
        outSystem->sensor.z_pitch = richSystem.sensor.z_pitch;
        outSystem->sensor.z_oft = richSystem.sensor.z_oft;
        outSystem->sensor.x_pitch = richSystem.sensor.x_pitch;
        outSystem->sensor.mount_height = richSystem.sensor.mount_height;

        std::strncpy(
            outSystem->sensor.unit,
            richSystem.sensor.unit.c_str(),
            sizeof(outSystem->sensor.unit)
        );
        outSystem->sensor.unit[sizeof(outSystem->sensor.unit) - 1] = '\0';

        return true;
    }

    EXPORT_API bool RM_RingCalibration_SetMeasure(
        rm::RingCalibration* ptr,
        const MeasureCfgInterop* interopCfg)
    {
        if (!ptr || !interopCfg) return false;

        rm::MeasureCfg richCfg;
        
        richCfg.img.path = interopCfg->img.path;

        for (int i = 0; i < static_cast<size_t>(rm::ScanLocation::NUM); ++i) {
            richCfg.img.imgs[i] = interopCfg->img.imgs[i];
            richCfg.img.luma_imgs[i] = interopCfg->img.luma_imgs[i];
        }

        richCfg.ring.isTured  = interopCfg->ring.isTured;
        richCfg.ring.spk_form = interopCfg->ring.spk_form;
        richCfg.ring.type     = interopCfg->ring.type;
        richCfg.ring.mke_top  = interopCfg->ring.mke_top;
        richCfg.ring.mke_btm  = interopCfg->ring.mke_btm;
        richCfg.ring.mke_mid  = interopCfg->ring.mke_mid;
        richCfg.ring.dI       = interopCfg->ring.dI;
        richCfg.ring.dA       = interopCfg->ring.dA;
        richCfg.ring.h        = interopCfg->ring.h;
        richCfg.robot_pos     = interopCfg->robot_pos;

        ptr->setMeasure(richCfg);
        return true;
    }

    EXPORT_API bool RM_RingCalibration_GetMeasure(
        const rm::RingCalibration* ptr,
        MeasureCfgInterop* outInteropCfg)
    {
        if (!ptr || !outInteropCfg) return false;

        rm::MeasureCfg richCfg = ptr->getMeasure();

        std::string pathStr = richCfg.img.path.string();
        std::strncpy(outInteropCfg->img.path, pathStr.c_str(), sizeof(outInteropCfg->img.path));
        outInteropCfg->img.path[sizeof(outInteropCfg->img.path) - 1] = '\0';

        for (int i = 0; i < static_cast<size_t>(rm::ScanLocation::NUM); ++i) {
            std::strncpy(outInteropCfg->img.imgs[i], richCfg.img.imgs[i].c_str(),
                         sizeof(outInteropCfg->img.imgs[i]));
            outInteropCfg->img.imgs[i][sizeof(outInteropCfg->img.imgs[i]) - 1] = '\0';

            std::strncpy(outInteropCfg->img.luma_imgs[i], richCfg.img.luma_imgs[i].c_str(),
                         sizeof(outInteropCfg->img.luma_imgs[i]));
            outInteropCfg->img.luma_imgs[i][sizeof(outInteropCfg->img.luma_imgs[i]) - 1] = '\0';
        }

        outInteropCfg->ring.isTured = richCfg.ring.isTured;
       
        std::strncpy(outInteropCfg->ring.spk_form, richCfg.ring.spk_form.c_str(),
                     sizeof(outInteropCfg->ring.spk_form));
        outInteropCfg->ring.spk_form[sizeof(outInteropCfg->ring.spk_form) - 1] = '\0';

        std::strncpy(outInteropCfg->ring.type, richCfg.ring.type.c_str(),
                     sizeof(outInteropCfg->ring.type));
        outInteropCfg->ring.type[sizeof(outInteropCfg->ring.type) - 1] = '\0';

        outInteropCfg->ring.mke_top = richCfg.ring.mke_top;
        outInteropCfg->ring.mke_btm = richCfg.ring.mke_btm;
        outInteropCfg->ring.mke_mid = richCfg.ring.mke_mid;
        outInteropCfg->ring.dI      = richCfg.ring.dI;
        outInteropCfg->ring.dA      = richCfg.ring.dA;
        outInteropCfg->ring.h       = richCfg.ring.h;

        outInteropCfg->robot_pos = richCfg.robot_pos;
        return true;
    }

    EXPORT_API bool RM_RingCalibration_SetParam(
        rm::RingCalibration* ptr,
        const rm::ParamTune* param)
    {
        if (!ptr || !param) {
            return false;
        }
        // Pass directly to the native core object
        ptr->setParam(*param);
        return true;
    }

    EXPORT_API bool RM_RingCalibration_GetParam(
        const rm::RingCalibration* ptr,
        rm::ParamTune* outParam)
    {
        if (!ptr || !outParam) {
            return false;
        }
        // Retrieve the struct from the core class and copy it to the caller's buffer
        *outParam = ptr->getParam();
        return true;
    }

    EXPORT_API bool RM_RingCalibration_Calibration(
        rm::RingCalibration* ptr,
        rm::ScanLocation scan,
        const char* path)
    {
        if (!ptr || !path) return false;

        std::filesystem::path fsPath(path);
        try {
            ptr->calibration(scan, fsPath);
        } catch (...) {
            return false;
        }
        return true;
    }

    EXPORT_API bool RM_RingCalibration_Verification(
        rm::RingCalibration* ptr,
        rm::ScanLocation scan,
        const char* path)
    {
        if (!ptr || !path) return false;

        std::filesystem::path fsPath(path);
        try {
            ptr->verification(scan, fsPath, false);
        } catch (...) {
            return false;
        }
        return true;
    }
   
    EXPORT_API bool RM_RingCalibration_GetSystemCorrection(
        const rm::RingCalibration* ptr,
        rm::ScanCfg* outBuffer,
        int bufferSize)
    {
        if (!ptr || !outBuffer || bufferSize != static_cast<size_t>(rm::ScanLocation::NUM)) {
            return false;
        }

        std::array<rm::ScanCfg, static_cast<size_t>(rm::ScanLocation::NUM)> correction 
            = ptr->getSystemCorrection();

        for (size_t i = 0; i < static_cast<size_t>(rm::ScanLocation::NUM); ++i) {
            outBuffer[i] = correction[i];
        }
        return true;
    }
   
    EXPORT_API bool RM_RingCalibration_SetSystemCorrection(
        rm::RingCalibration* ptr,
        const rm::ScanCfg* correctionArray,
        int arraySize)
    {
        if (!ptr || !correctionArray || arraySize != static_cast<size_t>(rm::ScanLocation::NUM)) {
            return false;
        }
        rm::EnumArray<rm::ScanLocation, rm::ScanCfg> correction;
       
        for (size_t i = 0; i < static_cast<size_t>(rm::ScanLocation::NUM); ++i) {
            correction[i] = correctionArray[i];
        }

        ptr->setSystemCorrection(correction);
        return true;
    }

    EXPORT_API bool RM_RingCalibration_SetLogger(
        rm::RingCalibration* ptr,
        const char* logger_name,
        const char* log_path,
        bool overwrite)
    {
        // Safety checks for null pointers
        if (!ptr || !logger_name || !log_path) {
            return false;
        }
        std::string name(logger_name);
        std::filesystem::path path(log_path);
        try
        {
            ptr->setLogger(name, path, overwrite);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            return false;
        }
        return true;
    }

    EXPORT_API bool RM_RingCalibration_GetLogger(
        rm::RingCalibration* ptr,
        char* name_buffer, int name_buffer_size,
        char* path_buffer, int path_buffer_size)
    {
        if (!ptr || !name_buffer || !path_buffer || name_buffer_size <= 0 || path_buffer_size <= 0) {
            return false;
        }
    
        try {
            std::string name, path;
            ptr->getLoggerInfo(name, path);
            
            if (name.length() >= (size_t)name_buffer_size || path.length() >= (size_t)path_buffer_size) {
                return false; // Buffer overflow protection
            }

            #if defined(_MSC_VER)
            strcpy_s(name_buffer, name_buffer_size, name.c_str());
            strcpy_s(path_buffer, path_buffer_size, path.c_str());
            #else
            std::strncpy(name_buffer, name.c_str(), name_buffer_size);
            name_buffer[name_buffer_size - 1] = '\0';
            std::strncpy(path_buffer, path.c_str(), path_buffer_size);
            path_buffer[path_buffer_size - 1] = '\0';
            #endif
    
            return true;
        }
        catch (...) {
            return false;
        }
    }
}