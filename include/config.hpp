#pragma once

#include <string>
#include <filesystem>
#include <pcl/point_types.h>
namespace RingMeasure{

    using str = std::string;

    /// @brief Scan location options relative to the measurement setup.
    enum class ScanLocation
    {
        /// Scan taken from the inner side of the ring
        In,
        /// Scan taken from the outer side of the ring
        Out,
        /// Scan taken from underneath the ring
        Under,
        /// Total number of scan locations
        NUM
    };

    /**
     * @brief Array of all scan locations.
     */
    constexpr std::array<ScanLocation, static_cast<size_t>(ScanLocation::NUM)> SCAN_LOCS = {
        ScanLocation::In,
        ScanLocation::Out,
        ScanLocation::Under
    };

    /// @brief Ellipse axis types for measurement calculations.
    enum class AxisType
    {
        /// Major axis of the ellipse
        MAJOR,
        /// Minor axis of the ellipse
        MINOR,
        Axis_NUM
    };

    const auto NUM = static_cast<size_t>(ScanLocation::NUM);
    const auto axis_num = static_cast<size_t>(AxisType::Axis_NUM);

    /**
     * @brief Converts a ScanLocation enum value to its string representation.
     * @param loc The ScanLocation enum value.
     * @return constexpr std::string_view String representation of the location.
     */
    constexpr std::string_view scanLocationToString(ScanLocation loc) {
        switch (loc) {
            case ScanLocation::In:    return "In";
            case ScanLocation::Out:   return "Out";
            case ScanLocation::Under: return "Under";
            default: break;
        }
        return "UNKNOWN";
    }
    /**
     * @brief Converts a string view to its corresponding ScanLocation enum value.
     * @param str String view representing the scan location.
     * @return ScanLocation The matching enum value.
     */
    ScanLocation stringToScanLocation(std::string_view str);

    /// @brief Types of manufacturing rings being measured.
    enum class RingType
    {
        /// Glatter ring
        GR,
        /// Flanch ring
        FR,
        /// Radreifen
        RR,
    };
    constexpr std::string_view ringTypeToString(RingType rt) {
        switch (rt) {
            case RingType::GR: return "GR";
            case RingType::FR: return "FR";
            case RingType::RR: return "RR";
        }
        return "UNKNOWN";
    }
    /**
     * @brief Converts a string view to its corresponding RingType enum value.
     * @param str String view representing the ring type.
     * @return RingType The matching enum value.
     */
    RingType stringToRingType(std::string_view str);

    /// @brief Types of processing operations applied.
    enum class ProcessType
    {   
        /// Rotation calibration process type.
        ROT, 
        /// Translation calibration process type.
        TRANS,
    };
    /**
     * @brief Converts a ProcessType enum value to its string representation.
     * @param pt The ProcessType enum value.
     * @return constexpr std::string_view String representation of the process type.
     */
    constexpr std::string_view processTypeToString(ProcessType pt) {
        switch (pt) {
            case ProcessType::ROT: return "rot";
            case ProcessType::TRANS: return "trans";
        }
        return "UNKNOWN";
    }
    /**
     * @brief Converts a string view to its corresponding ProcessType enum value.
     * @param str String view representing the process type.
     * @return ProcessType The matching enum value.
     */
    ProcessType stringToProcessType(std::string_view str);

    /// @brief Execution phase types for the workflow.
    enum class PhaseType
    {
        /// Calibration phase
        CALI,
        /// Verification phase
        VERI,
    };
    /**
     * @brief Converts a PhaseType enum value to its string representation.
     * @param pt The PhaseType enum value.
     * @return constexpr std::string_view String representation of the phase type.
     */
    constexpr std::string_view phaseTypeToString(PhaseType pt) {
        switch (pt) {
            case PhaseType::CALI: return "cali";
            case PhaseType::VERI: return "veri";
        }
        return "UNKNOWN";
    }
    /**
     * @brief Converts a string view to its corresponding PhaseType enum value.
     * @param str String view representing the phase type.
     * @return PhaseType The matching enum value.
     */
    PhaseType stringToPhaseType(std::string_view str);

    /**
     * @brief Configuration parameters for hardware sensors.
     */
    struct SensorCfg
    {
        int scan_num;       ///< Total number of scans, 360
        int scan_res;       ///< Scan resolution of a line, 3200
        int valid_len;      ///< Valid data length threshold, 720
        float z_pitch;      ///< Z-axis pitch distance
        float z_oft;        ///< Z-axis offset value
        float x_pitch;      ///< X-axis pitch distance
        std::string unit;   ///< Measurement unit string (e.g., mm)
        float mount_height; ///< Physical mounting height of the sensor relative to the camera.
                            ///< Not the height of the roll table.
    };

    /**
     * @brief A simple 2D point structure with basic vector operators.
     */
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

    /**
     * @brief Configuration parameters for individual scan nodes.
     */    struct ScanCfg
    {
        float view_angle{0.0f}; ///< View angle (va) in degrees
        float angle_corr{0.0f}; ///< Angle correction offset
        Point2f pos_corr{0.0f, 0.0f}; ///< Position correction offset (X, Y)
        ScanCfg() = default;
        ScanCfg(const ScanCfg &val) : view_angle(val.view_angle), angle_corr(val.angle_corr), pos_corr(val.pos_corr) {}
        ScanCfg(float va, float ac, Point2f pc) : view_angle(va), angle_corr(ac), pos_corr(pc) {}
    };

    /**
     * @brief System configuration parameters for the robot setup.
     */    
    struct RobotCfg
    {    
        /**< Array of scan configurations per location */
        std::array<ScanCfg, NUM> scans;
        Point2f arm;                    ///< Robot arm coordinate offset
        float dist;                     ///< Operating distance parameter
        float table_h;                  ///< Roll table height configuration
        float wedge_h;                  ///< Wedge height configuration
        ///< This is 10mm accoring to the measurement.
        ///< Confirm with the manufacturer for the exact value.
    };

    /**
     * @brief Global system configuration holding robot and sensor specifications.
     */
    struct SystemCfg
    {
        RobotCfg robot;   ///< Robot configuration parameters
        SensorCfg sensor; ///< Sensor configuration parameters
    };

    /**
     * @brief File paths and collection names for images.
     */
    struct ImgCfg 
    {
        std::filesystem::path path;    ///< Base path for image files
        std::array<std::string, NUM> imgs;     ///< Standard image filenames per location
        std::array<std::string, NUM> luma_imgs;///< Luminance image filenames per location
    };
    
    /**
     * @brief Specifications of the target ring being processed.
     */
    struct RingCfg
    {
        std::string spk_form; ///< Spoke form description
        bool isTured;         ///< Flag indicating if the ring is turned
        std::string type;     ///< Type description string
        float mke_top;        ///< Top measurement height
        float mke_btm;        ///< Bottom measurement height
        float mke_mid;        ///< Middle measurement height
        float dI;             ///< Inner diameter configuration
        float dA;             ///< Outer diameter configuration
        float h;              ///< Height configuration
    };

    /**
     * @brief Comprehensive configuration for measurement routines.
     */
    struct MeasureCfg
    {
        ImgCfg img;                     ///< Image configuration and paths
        RingCfg ring;                   ///< Target ring parameters
        std::array<Point2f, NUM> robot_pos; ///< Robot coordinate positions per location
    };

    /**
     * @brief Loads measurement configuration settings from an INI file.
     * 
     * @param measure_cfg Path to the measurement configuration file.
     * @param measure Reference to the MeasureCfg structure to populate.
     */
    void loadMeasureConfig(const std::filesystem::path& measure_cfg, struct MeasureCfg& measure);

    /**
     * @brief Loads system configuration settings from an INI file.
     * 
     * @param sys_cfg Path to the system configuration file.
     * @param sys Reference to the SystemCfg structure to populate.
     */
    void loadSystemConfig(const std::filesystem::path& sys_cfg, struct SystemCfg& sys);

    /**
     * @brief Dumps current system configuration settings to an INI file.
     * If the file is setting to "stdout", the parameters will be printed to the console
     * instead of being written to a file.
     * 
     * @param sys_cfg Path to the target output configuration file.
     * @param sys Constant reference to the SystemCfg structure to write.
     */
    void dumpConfig(const std::filesystem::path& sys_cfg,
                    const struct SystemCfg& sys);
}
