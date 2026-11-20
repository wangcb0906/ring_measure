#pragma once

#include <string>
#include <filesystem>
#include <array>
#include <iostream>
namespace RingMeasure{

    using str = std::string;

    // Primary template: defaults to false
    template <typename T, typename = void>
    struct has_num : std::false_type {};
    
    // Specialization: succeeds only if T::NUM is a valid expression
    template <typename T>
    struct has_num<T, std::void_t<decltype(T::NUM)>> : std::true_type {};
    
    template <typename T>
    inline constexpr bool has_num_v = has_num<T>::value;
    
    /**
     * @struct  EnumArray
     * @brief   std::array wrapper that supports indexing via `enum` keys.
     * @details Inherits from `std::array<T, N>` to preserve all features, 
     *              and standard container traits while overloading `operator[]`
     *              with zero runtime overhead.
     * @tparam  Enum Scoped enumeration type used as the index key. Must define 
     *          a trailing `NUM` enumerator if the default size parameter is used.
     * @tparam  T The element type stored in the array.
     * @tparam  N The number of elements in the array (static_cast<size_t>(NUM)).
     * @note    Enumerators must be contiguous and zero-based (0, 1, ..., NUM-1).
     */
    template <typename Enum, typename T, size_t N = static_cast<size_t>(Enum::NUM)>
    struct EnumArray : std::array<T, N> {
        static_assert(has_num_v<Enum> || N != 0,
                      "Enum must define a 'NUM' enumerator (or specify size N explicitly).");
        constexpr T& operator[](Enum e) noexcept {
            return std::array<T, N>::operator[](static_cast<size_t>(e));
        }
        constexpr T& operator[](size_t idx) noexcept {
            return std::array<T, N>::operator[](idx);
        }
        constexpr const T& operator[](Enum e) const noexcept {
            return std::array<T, N>::operator[](static_cast<size_t>(e));
        }
        constexpr const T& operator[](size_t idx) const noexcept {
            return std::array<T, N>::operator[](idx);
        }
    };

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
    constexpr EnumArray<ScanLocation, ScanLocation> SCAN_LOCS = {
        ScanLocation::In,
        ScanLocation::Out,
        ScanLocation::Under
    };
    
    /**
     * @brief Ellipse axis types for measurement calculations.
     */
    enum class AxisType
    {
        /// Major axis of the ellipse
        MAJOR,
        /// Minor axis of the ellipse
        MINOR,
        NUM
    };

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
        NUM,
    };

    /**
     * @brief Converts a RingType enum value to its string representation.
     * @param rt The RingType enum value.
     * @return constexpr std::string_view String representation of the ring type.
     */
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
        NUM
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
        NUM
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
        float pitch_scale;
        float pitch_offset;
        float cam_res;      ///< Camera resolution
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
        Point2f(const Point2f &val) = default;
        Point2f(float x_, float y_) : x(x_), y(y_) {}

        friend Point2f operator+(const Point2f& lhs, const Point2f& rhs)
        {
            return Point2f{lhs.x + rhs.x, lhs.y + rhs.y};
        }
        friend Point2f operator-(const Point2f& lhs, const Point2f& rhs)
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
     */    
    struct ScanCfg
    {
        float view_angle; ///< View angle (va) in degrees
        float angle_corr; ///< Angle correction offset
        Point2f pos_corr; ///< Position correction offset (X, Y)
        ScanCfg() = default;
        ScanCfg(const ScanCfg &val) = default;
        ScanCfg(float va, float ac, Point2f pc) : view_angle(va), angle_corr(ac), pos_corr(pc) {}
    };

    /**
     * @brief System configuration parameters for the robot setup.
     */    
    struct RobotCfg
    {    
        /**< Array of scan configurations per location */
        EnumArray<ScanLocation, ScanCfg> scans;
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
        EnumArray<ScanLocation, std::string> imgs;
        ///< Standard image filenames per location
        EnumArray<ScanLocation, std::string> luma_imgs;
        ///< Luminance image filenames per location
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
        float mid_height;     ///< Middle height of the scan
    };

    /**
     * @brief Comprehensive configuration for measurement routines.
     */
    struct MeasureCfg
    {
        ImgCfg img;                     ///< Image configuration and paths
        RingCfg ring;                   ///< Target ring parameters
        EnumArray<ScanLocation, Point2f> robot_pos; ///< Robot coordinate positions per location
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
    /**
     * @brief Dumps correction parameters to an INI file.
     * If the file is set to "stdout", the parameters will be printed to the console
     * instead of being written to a file.
     * 
     * @param correction_cfg Path to the target output configuration file.
     * @param scans Array of scan configurations.
     * @param mode File open mode.
     */
    void dumpCorrection(const std::filesystem::path& correction_cfg,
                        const EnumArray<ScanLocation, ScanCfg>& scans,
                        std::ios_base::openmode mode = std::ios_base::out);
}
