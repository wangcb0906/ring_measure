#pragma once

#include <string>
#include <filesystem>
#include <pcl/point_types.h>
namespace RingMeasure{

    enum class ScanLocation
    {
        In,
        Out,
        Under,
        NUM
    };
    enum class AxisType
    {
        MAJOR, // major axis of the ellipse
        MINOR, // minor axis of the ellipse
        Axis_NUM
    };
    const auto NUM = static_cast<size_t>(ScanLocation::NUM);
    const auto axis_num = static_cast<size_t>(AxisType::Axis_NUM);
    using str = std::string;
    constexpr std::string_view scanLocationToString(ScanLocation loc) {
        switch (loc) {
            case ScanLocation::In:    return "In";
            case ScanLocation::Out:   return "Out";
            case ScanLocation::Under: return "Under";
            default: break;
        }
        return "UNKNOWN";
    }
    ScanLocation stringToScanLocation(std::string_view str);

    enum class RingType
    {
        GR, // glatter ring
        FR, // flanch ring
        RR  // Radreifen
    };
    constexpr std::string_view ringTypeToString(RingType rt) {
        switch (rt) {
            case RingType::GR: return "GR";
            case RingType::FR: return "FR";
            case RingType::RR: return "RR";
        }
        return "UNKNOWN";
    }
    RingType stringToRingType(std::string_view str);

    enum class ProcessType
    {
        ROT, // rotation
        TRANS, // translation
    };
    constexpr std::string_view processTypeToString(ProcessType pt) {
        switch (pt) {
            case ProcessType::ROT: return "rot";
            case ProcessType::TRANS: return "trans";
        }
        return "UNKNOWN";
    }
    ProcessType stringToProcessType(std::string_view str);

    enum class PhaseType
    {
        CALI, // calibration
        VERI, // verification
    };
    constexpr std::string_view phaseTypeToString(PhaseType pt) {
        switch (pt) {
            case PhaseType::CALI: return "cali";
            case PhaseType::VERI: return "veri";
        }
        return "UNKNOWN";
    }
    PhaseType stringToPhaseType(std::string_view str);

    struct SensorCfg
    {
        int scan_num;
        int scan_res;
        int valid_len;
        float z_pitch;
        float z_oft;
        float x_pitch;
        std::string unit;
        float mount_height;
    };

    struct Point2f {
        float x;
        float y;

        Point2f() = default;
        Point2f(const Point2f &val) : x(val.x), y(val.y) {}
        Point2f(float x_, float y_) : x(x_), y(y_) {}

        friend Point2f operator+(Point2f& lhs, const Point2f& rhs)
        {
            return Point2f{lhs.x + rhs.x, lhs.y + rhs.y};
        }
        friend Point2f operator-(Point2f& lhs, const Point2f& rhs)
        {
            return Point2f{lhs.x - rhs.x, lhs.y - rhs.y};
        }
        Point2f& operator+=(const Point2f& rhs)
        {
            this->x += rhs.x;
            this->y += rhs.y;
            return *this;
        }
        Point2f& operator-=(const Point2f& rhs)
        {
            this->x -= rhs.x;
            this->y -= rhs.y;
            return *this;
        }
    };
    
    struct ScanCfg
    {
        // va = view angle
        float view_angle{0.0f};
        float angle_corr{0.0f};
        Point2f pos_corr{0.0f, 0.0f};
        ScanCfg() = default;
        ScanCfg(const ScanCfg &val) : view_angle(val.view_angle), angle_corr(val.angle_corr), pos_corr(val.pos_corr) {}
        ScanCfg(float va, float ac, Point2f pc) : view_angle(va), angle_corr(ac), pos_corr(pc) {}
    };

    struct RobotCfg
    {   
        std::array<ScanCfg, NUM> scans;
        Point2f arm;
        float dist;
        float table_h;
        float wedge_h;
    };
    
    struct SystemCfg
    {
        RobotCfg robot;
        SensorCfg sensor;
    };
    
    struct ImgCfg 
    {
        std::filesystem::path path;
        std::array<std::string, NUM> imgs;
        std::array<std::string, NUM> luma_imgs;
    };
    
    struct RingCfg
    {
        std::string spk_form;
        bool isTured;
        std::string type;
        float mke_top;
        float mke_btm;
        float mke_mid;
        float dI;
        float dA;
        float h;
    };

    struct MeasureCfg
    {
        ImgCfg  img;
        RingCfg ring;
        std::array<Point2f, NUM> robot_pos;
    };

    void loadMeasureConfig(const std::string& measure_cfg, struct MeasureCfg& measure);

    void loadSystemConfig(const std::string& sys_cfg, struct SystemCfg& sys);

    void dumpConfig(const std::string& sys_cfg,
                    const struct SystemCfg& sys);
}
