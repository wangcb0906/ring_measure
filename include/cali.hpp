#pragma once

#include "config.hpp"
#include "param.hpp"
#include "log.hpp"
#include <Eigen/Core>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace RingMeasure {

    namespace rm=::RingMeasure;
    namespace fs=std::filesystem;

    using pcl::PointXYZ;
    using pcl::PointNormal;

    /// @brief Tuple type for base fit metrics (height, normal).
    using BaseFitMetricTuple = std::tuple<float, Eigen::Vector3f>;
    /// @brief Tuple type for cylinder fit metrics (radius, axis, center)
    using CYLFitMetricTuple = std::tuple<float, Eigen::Vector3f, Eigen::Vector3f>;

    /// @brief Evaluation metrics calculated for individual scan poses.
    enum class ScanMetrics{
        H_PLANE_FIT,         ///< Base plane fit evaluation metric
        ANGLE_MAJOR_LINE_FIT,  ///< Major line angle evaluation metric
        ANGLE_MINOR_LINE_FIT,  ///< Minor line angle evaluation metric
        RADIUS_MAJOR_LINE_FIT, ///< Major line radius evaluation metric
        RADIUS_MINOR_LINE_FIT, ///< Minor line radius evaluation metric
        RADIUS_CYL_FIT,        ///< Cylinder fit radius evaluation metric
        NUM                    ///< Total number of scan metrics
    };
    
    /// @brief Global reconstruction metrics evaluated across multi-position scans.
    enum class ReconMetrics{
        CENTER_CYL_FIT,        ///< Center cylinder fit distance metric
        AXIS_CYL_FIT,          ///< Cylinder axis alignment metric
        BASE_PARALLELISM,      ///< Base planes parallelism metric
        NUM                    ///< Total number of reconstruction metrics
    };
    using ReconMetricsArray = EnumArray<ReconMetrics, float>;
    using ScanMetricsArray = EnumArray<ScanMetrics, float>;
    template <typename PointT>
    using PcPtr = typename pcl::PointCloud<PointT>::Ptr;

    template <typename Enum>
    constexpr std::size_t idx(Enum e) noexcept {
        static_assert(std::is_enum_v<Enum>, "idx() requires an enum type");
        return static_cast<std::size_t>(e);
    }

    /**
     * @brief Core calibration and verification engine for ring measurement systems.
     * @details Manages point cloud processing pipelines, geometric fitting (planes,
     *  cylinders, lines), and evaluation metrics across various scan positions.
     */
    class RingCalibration{

        public:
            /**
             * @brief Construct a new RingCalibration object with default settings.
             * Note that the measure and system must be set before performing any
             * calibration or verification operations.
             */
            RingCalibration() = default;

            std::shared_ptr<spdlog::logger> getLogger() const { return logger; }

            void setLogger(const std::shared_ptr<spdlog::logger> &logger_) { 
                logger = logger_;
                logger->flush_on(spdlog::level::err);
            }

            void setLogger(const std::string &logger_name, const fs::path &log_path, bool overwrite) {
                // Bind PCL logger to stderr
                auto handler = createPclLogHandler(spdlog::stderr_color_mt("stderr_logger"));
                pcl::console::Logger::getInstance().setCallback(handler);
                logger = spdlog::basic_logger_mt(logger_name, log_path.string(), overwrite);
                logger->set_level(spdlog::level::trace);
                logger->set_pattern("[%Y-%m-%d %H:%M:%S] [%l] [%s:%#] %v");
                logger->flush_on(spdlog::level::err);
            }

            void getLoggerInfo(std::string& name, std::string& path) const {
                if (!logger) return;
                name.clear();
                path.clear();
                name = logger->name();
                
                if (!logger->sinks().empty()) {
                    auto file_sink = 
                        std::dynamic_pointer_cast<spdlog::sinks::basic_file_sink_mt>(logger->sinks()[0]);
                    if (file_sink) {
                        path = file_sink->filename();
                    }
                }
            }

            ScanLocation scanLoc() const { return loc; }
            
            const auto & getLoc() const { return loc; }

            rm::MeasureCfg getMeasure() const { return measure; }

            void setMeasure(const rm::MeasureCfg &measure_)
            { 
                measure = std::move(measure_); 
                system.sensor.cam_res = measure.ring.dA * M_PI / 360.0f;
                if(system_ready)
                    measure.ring.mid_height = measure.ring.h/2.0 + system.robot.table_h;
                measure_ready = true;
            }

            rm::SystemCfg getSystem() const { return system; }
            void setSystem(const rm::SystemCfg &system_)
            { 
                system = system_; 
                system_ready = true;
                system.robot.table_h -= system.robot.wedge_h;
                if(measure_ready)
                    measure.ring.mid_height = measure.ring.h/2.0 + system.robot.table_h;
                system.sensor.pitch_scale = system.sensor.z_pitch/1.0e5;
                system.sensor.pitch_offset = 
                    system.sensor.z_oft/1.0e5 - system.sensor.mount_height;
            }

            void setSystemCorrection(const EnumArray<ScanLocation, ScanCfg> &correction)
            { 
                system.robot.scans = correction;
            }

            const auto & getSystemCorrection() const { return system.robot.scans; }

            const auto & rawRecs() const { return raw_recs; }

            const auto & getLaterals() const { return laterals; }

            const auto & getBases() const { return bases; }

            const auto & recNormals() const { return rec_normals; }

            const auto & getRadius() const { return radius; }

            const auto & getAngle() const { return angle; }

            auto getScanNum() const { return system.sensor.scan_num; }

            /**
             * @brief Computes global multi-scan reconstruction evaluation metrics.
             * @return ReconMetricsArray Array containing calculated cross-scan metrics.
             */
            friend ReconMetricsArray reconEval(const RingCalibration& cali);
            
            /**
             * @brief Computes metrics for the currently active scan location.
             * @return ScanMetricsArray Array containing calculated single-scan metrics.
             */
            friend ScanMetricsArray scanEval(const RingCalibration& cali);

            PhaseType phaseType() const { return phase_type; }
            void setPhaseType(const PhaseType &phaseType) { phase_type = phaseType;}
            
            /**
             * @brief Executes the full calibration routine for a specific scan location.
             * @param scan Target scan location.
             * @param path Directory path for logging and exporting intermediate artifacts.
             */
            void calibration(const ScanLocation& scan, const fs::path& path);

            /**
             * @brief Executes the verification routine for a specific scan location.
             * @param scan Target scan location.
             * @param path Directory path for output artifacts.
             * @param profile Flag indicating whether profiling output should be generated.
             */
            void verification(const ScanLocation& scan, const fs::path& path,
                              bool profile=true);

            const auto & cylFit() const { return cyl_fit; }

            const auto & baseFit() const { return base_fit; }

            ParamTune getParam() const { return param; }
            void setParam(const ParamTune &param_) { param = param_; }

        protected:
            /**
             * @brief Checks if both measure and system configurations are ready for processing.
             */
            bool isReady() const {
                return measure_ready && system_ready;
            }

            /**
             * @brief Sets the scan location and updates the corresponding index.
             * @param scanLoc The new scan location.
             */
            void setScanLoc(const ScanLocation &scanLoc) { loc = scanLoc; }

            ProcessType processType() const { return process_type; }
            void setProcessType(const ProcessType &processType) { process_type = processType; }

            /**
             * @brief Reconstructs point cloud data from raw files.
             * @param path Path to the raw data files.
             */
            void reconstruct(const fs::path& path);

            /**
             * @brief Applies denoising filters and segmentation.
             * @param path_den Path for denoising outputs.
             * @param path_seg Path for segmentation outputs.
             * @param free_raw Flag to release raw memory after processing.
             */
            void denoise(fs::path path_den, fs::path path_seg, bool free_raw=true);

            /**
             * @brief Performs rotational calibration routine.
             * @param path Target directory path.
             */
            void caliRot(fs::path path);

            /**
             * @brief Performs translational calibration routine.
             * @param path Target directory path.
             */
            void caliTrans(fs::path path);

            /**
             * @brief Extracts Euclidean clusters from a point cloud.
             * @param input Pointer to input point cloud.
             * @param min_cluster_size Minimum point count requirement for clusters.
             * @param path Export directory path.
             * @param prefix Filename prefix for outputs.
             * @return pcl::PointCloud<pcl::PointXYZ>::Ptr Extracted cluster cloud.
             */
            pcl::PointCloud<pcl::PointXYZ>::Ptr
            ecExtract(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                      int min_cluster_size,
                      fs::path path,
                      const std::string prefix
            );

            /**
             * @brief Performs normal-based segmentation to isolate lateral and base surfaces.
             * @param input Pointer to source point cloud.
             * @param path Export directory path.
             * @param prefix Filename prefix for exports.
             * @param lateral Reference to output lateral point cloud.
             * @param base Reference to output base point cloud.
             * @return pcl::PointCloud<pcl::PointNormal>::Ptr Processed normal point cloud.
             */
            pcl::PointCloud<pcl::PointNormal>::Ptr
            normalSegmentation(
                pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                fs::path path,
                const std::string prefix,
                pcl::PointCloud<pcl::PointNormal>::Ptr& lateral,
                pcl::PointCloud<pcl::PointNormal>::Ptr& base
            );
    
        protected:
            /// Active logger instance, default console
            std::shared_ptr<spdlog::logger> logger=spdlog::default_logger(); 
            ScanLocation loc;              ///< Current scan location
            ParamTune param;               ///< Calibration parameters
            MeasureCfg measure;            ///< Measure configuration
            SystemCfg system;              ///< System configuration

            // result from cylinder fitting
            EnumArray<ScanLocation, CYLFitMetricTuple> cyl_fit;

            // radius from line fitting
            EnumArray<ScanLocation, EnumArray<AxisType, float>> radius;
            // cosine of angle from line fitting
            EnumArray<ScanLocation, EnumArray<AxisType, Eigen::Vector2f>> angle;

            // result from plane fitting
            EnumArray<ScanLocation, BaseFitMetricTuple> base_fit;

            EnumArray<ScanLocation, PcPtr<PointXYZ>> raw_recs{nullptr};
            EnumArray<ScanLocation, PcPtr<PointNormal>> laterals{nullptr};
            EnumArray<ScanLocation, PcPtr<PointNormal>> bases{nullptr};
            EnumArray<ScanLocation, PcPtr<PointNormal>> rec_normals{nullptr};
        
        private:
            RingType ring_type=RingType::GR; ///< Calibration ring type (fixed to smooth ring)
            PhaseType phase_type;            ///< Active execution phase type
            ProcessType process_type;        ///< Active process type
            bool measure_ready = false;      ///< Flag indicating measure configuration readiness
            bool system_ready = false;       ///< Flag indicating system configuration readiness
    };
}