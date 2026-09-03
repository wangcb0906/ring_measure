#include "config.hpp"

#include <inicpp.h>

namespace RingMeasure {

    const auto I = static_cast<size_t>(RingMeasure::ScanLocation::In);
    const auto O = static_cast<size_t>(RingMeasure::ScanLocation::Out);
    const auto U = static_cast<size_t>(RingMeasure::ScanLocation::Under);
    
    void dumpConfig(const std::string& sys_cfg,
                                 const struct SystemCfg& sys)
    {
        ini::IniFile system_ini(sys_cfg);
      
        system_ini["Robot"]["angleI"] = sys.robot.scans[I].view_angle;
        system_ini["Robot"]["corrAngleI"] = sys.robot.scans[I].angle_corr;
        system_ini["Robot"]["corrXI"] = sys.robot.scans[I].pos_corr.x;
        system_ini["Robot"]["corrYI"] = sys.robot.scans[I].pos_corr.y;
      
        system_ini["Robot"]["angleA"] = sys.robot.scans[O].view_angle;
        system_ini["Robot"]["corrAngleA"] = sys.robot.scans[O].angle_corr;
        system_ini["Robot"]["corrXA"] = sys.robot.scans[O].pos_corr.x;
        system_ini["Robot"]["corrYA"] = sys.robot.scans[O].pos_corr.y;
      
        system_ini["Robot"]["angleU"] = sys.robot.scans[U].view_angle;
        system_ini["Robot"]["corrAngleU"] = sys.robot.scans[U].angle_corr;
        system_ini["Robot"]["corrXU"] = sys.robot.scans[U].pos_corr.x;
        system_ini["Robot"]["corrYU"] = sys.robot.scans[U].pos_corr.y;
      
        system_ini["Robot"]["armX"] = sys.robot.arm.x;
        system_ini["Robot"]["armY"] = sys.robot.arm.y;
        system_ini["Robot"]["dist"] = sys.robot.dist ;
        system_ini["Robot"]["table_h"] = sys.robot.table_h ;
        system_ini["Robot"]["wedge_h"] = sys.robot.wedge_h ;
      
        system_ini["Sensor"]["scan_num"] = sys.sensor.scan_num;
        system_ini["Sensor"]["scan_res"] = sys.sensor.scan_res;
        system_ini["Sensor"]["valid_len"] = sys.sensor.valid_len;
        system_ini["Sensor"]["z_pitch"] = sys.sensor.z_pitch;
        system_ini["Sensor"]["z_oft"] = sys.sensor.z_oft;
        system_ini["Sensor"]["x_pitch"] = sys.sensor.x_pitch;
        system_ini["Sensor"]["unit"] = sys.sensor.unit;
        system_ini["Sensor"]["mount"] = sys.sensor.mount_height;
        if(sys_cfg == "stdout"){
            system_ini.encode(std::cout);
            return;
        }
        std::ofstream outFile(sys_cfg);
        system_ini.encode(outFile);
    }
    
    ScanLocation stringToScanLocation(std::string_view str) {
        if (str == "In")    return ScanLocation::In;
        else if (str == "Out")   return ScanLocation::Out;
        else if (str == "Under") return ScanLocation::Under;
        throw std::invalid_argument("Invalid string for ScanLocation: " + std::string(str));
    }
    
    RingType stringToRingType(std::string_view str) {
        if (str == "GR") return RingType::GR;
        else if (str == "FR") return RingType::FR;
        else if (str == "RR") return RingType::RR;
        throw std::invalid_argument("Invalid string for RingType: " + std::string(str));
    }

    ProcessType stringToProcessType(std::string_view str) {
        if (str == "rot") return ProcessType::ROT;
        else if (str == "trans") return ProcessType::TRANS;
        throw std::invalid_argument("Invalid string for ProcessType: " + std::string(str));
    }

    PhaseType stringToPhaseType(std::string_view str) {
        if (str == "cali") return PhaseType::CALI;
        else if (str == "veri") return PhaseType::VERI;
        throw std::invalid_argument("Invalid string for PhaseType: " + std::string(str));
    }

    void loadMeasureConfig(const std::string& measure_cfg, struct MeasureCfg& measure)
    {
        ini::IniFile measure_ini(measure_cfg);
        measure.img.path = std::filesystem::path(measure_ini["Scan"]["path"].as<std::string>());
        measure.img.imgs[I]      = measure_ini["Scan"]["I"].as<std::string>();
        measure.img.imgs[O]      = measure_ini["Scan"]["A"].as<std::string>();
        measure.img.imgs[U]      = measure_ini["Scan"]["U"].as<std::string>();
        measure.img.luma_imgs[I] = measure_ini["Scan"]["LumaI"].as<std::string>();
        measure.img.luma_imgs[O] = measure_ini["Scan"]["LumaA"].as<std::string>();
        measure.img.luma_imgs[U] = measure_ini["Scan"]["LumaU"].as<std::string>();
      
        measure.ring.spk_form = measure_ini["Ring"]["spk_form"].as<std::string>();
        measure.ring.isTured = measure_ini["Ring"]["is_turned"].as<bool>();
        measure.ring.type = measure_ini["Ring"]["type"].as<std::string>();
        measure.ring.mke_top = measure_ini["Ring"]["mke_top"].as<float>();
        measure.ring.mke_btm = measure_ini["Ring"]["mke_btm"].as<float>();
        measure.ring.mke_mid = measure_ini["Ring"]["mke_mid"].as<float>();
        measure.ring.dI = measure_ini["Ring"]["di"].as<float>();
        measure.ring.dA = measure_ini["Ring"]["da"].as<float>();
        measure.ring.h = measure_ini["Ring"]["h"].as<float>();
      
        measure.robot_pos[I].x = measure_ini["Robot"]["xI"].as<float>();
        measure.robot_pos[I].y = measure_ini["Robot"]["yI"].as<float>();
        measure.robot_pos[O].x = measure_ini["Robot"]["xA"].as<float>();
        measure.robot_pos[O].y = measure_ini["Robot"]["yA"].as<float>();
        measure.robot_pos[U].x = measure_ini["Robot"]["xU"].as<float>();
        measure.robot_pos[U].y = measure_ini["Robot"]["yU"].as<float>();
    }

    void loadSystemConfig(const std::string& sys_cfg, struct SystemCfg& sys)
    {
    
        ini::IniFile system_ini(sys_cfg);
        sys.robot.scans[I].view_angle   = system_ini["Robot"]["angleI"].as<float>();
        sys.robot.scans[I].angle_corr   = system_ini["Robot"]["corrAngleI"].as<float>();
        sys.robot.scans[I].pos_corr.x = system_ini["Robot"]["corrXI"].as<float>();
        sys.robot.scans[I].pos_corr.y = system_ini["Robot"]["corrYI"].as<float>();
      
        sys.robot.scans[O].view_angle   = system_ini["Robot"]["angleA"].as<float>();
        sys.robot.scans[O].angle_corr   = system_ini["Robot"]["corrAngleA"].as<float>();
        sys.robot.scans[O].pos_corr.x = system_ini["Robot"]["corrXA"].as<float>();
        sys.robot.scans[O].pos_corr.y = system_ini["Robot"]["corrYA"].as<float>();
      
        sys.robot.scans[U].view_angle   = system_ini["Robot"]["angleU"].as<float>();
        sys.robot.scans[U].angle_corr   = system_ini["Robot"]["corrAngleU"].    as<float>();
        sys.robot.scans[U].pos_corr.x = system_ini["Robot"]["corrXU"].as<float>();
        sys.robot.scans[U].pos_corr.y = system_ini["Robot"]["corrYU"].as<float>();
      
        sys.robot.arm.x = system_ini["Robot"]["armX"].as<float>();
        sys.robot.arm.y = system_ini["Robot"]["armY"].as<float>();
        sys.robot.dist  = system_ini["Robot"]["dist"].as<float>();
        sys.robot.table_h  = system_ini["Robot"]["table_h"].as<float>();
        sys.robot.wedge_h  = system_ini["Robot"]["wedge_h"].as<float>();
      
        sys.sensor.scan_num = system_ini["Sensor"]["scan_num"].as<int>();
        sys.sensor.scan_res = system_ini["Sensor"]["scan_res"].as<int>();
        sys.sensor.valid_len = system_ini["Sensor"]["valid_len"].as<int>();
        sys.sensor.z_pitch = system_ini["Sensor"]["z_pitch"].as<float>();
        sys.sensor.z_oft = system_ini["Sensor"]["z_oft"].as<float>();
        sys.sensor.x_pitch = system_ini["Sensor"]["x_pitch"].as<float>();
        sys.sensor.unit = system_ini["Sensor"]["unit"].as<std::string>();
        sys.sensor.mount_height = system_ini["Sensor"]["mount"].as<float>();
    }
}