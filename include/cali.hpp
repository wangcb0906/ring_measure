#pragma once

#include "config.hpp"
#include "param.hpp"
#include <Eigen/Core>
#include <spdlog/spdlog.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace RingMeasure {

    namespace rm = RingMeasure;
    namespace fs=std::filesystem;

    /**
     * @brief Core calibration and verification engine for ring measurement systems.
     * @details Manages point cloud processing pipelines, geometric fitting (planes, cylinders, lines),
     *          and evaluation metrics across various scan positions.
     */
    class RingCalibration{
        public:

            /**
             * @brief Construct a new RingCalibration object with default settings.
             * Note that the measure and system must be set before performing any calibration or verification operations.
             */
            RingCalibration() = default;

            /**
             * @brief Construct a new RingCalibration object using measurement and system configurations.
             * 
             * @param measure Measurement configuration parameters.
             * @param system System and robot configuration parameters.
             */
            RingCalibration(const rm::MeasureCfg& measure, const rm::SystemCfg& system)
             : measure(measure), system(system)
            {
                logger = spdlog::default_logger();;
                camera_resolution = measure.ring.dA * M_PI / 360.0f;
                mid_height = measure.ring.h/2.0 + system.robot.table_h - system.robot.wedge_h;
                smallest_value = system.sensor.z_oft/1.0e5  - system.sensor.mount_height;
                measure_ready = true;
                system_ready = true;
            };

            /**
             * @brief Destroy the RingCalibration object and release allocated point cloud resources.
             */
            ~RingCalibration()
            {
                for (size_t i = 0; i < NUM; ++i)
                {
                    raw_recs[i].reset();
                    laterals[i].reset();
                    bases[i].reset();
                    rec_normals[i].reset();
                }
            };

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

            /// @brief Tuple type for base fit metrics (height, normal).
            using BaseFitMetricTuple = std::tuple<float, Eigen::Vector3f>;
            /// @brief Tuple type for cylinder fit metrics (radius, axis, center)
            using CYLFitMetricTuple = std::tuple<float, Eigen::Vector3f, Eigen::Vector3f>;
            using ReconMetricsArray = std::array<float, static_cast<size_t>(ReconMetrics::NUM)>;
            using ScanMetricsArray = std::array<float, static_cast<size_t>(ScanMetrics::NUM)>;

            std::shared_ptr<spdlog::logger> getLogger() const { return logger; }
            void setLogger(const std::shared_ptr<spdlog::logger> &logger_) { logger = logger_; }

            ScanLocation scanLoc() const { return scan_loc; }
            
            size_t getLoc() const { return loc; }

            rm::MeasureCfg getMeasure() const { return measure; }
            void setMeasure(const rm::MeasureCfg &measure_)
            { 
                measure = measure_; 
                camera_resolution = measure.ring.dA * M_PI / 360.0f;
                if(system_ready)
                    mid_height = measure.ring.h/2.0 + system.robot.table_h - system.robot.wedge_h;
                measure_ready = true;
            }

            rm::SystemCfg getSystem() const { return system; }
            void setSystem(const rm::SystemCfg &system_)
            { 
                system = system_; 
                smallest_value = system.sensor.z_oft/1.0e5  - system.sensor.mount_height;
                system_ready = true;
                if(measure_ready)
                    mid_height = measure.ring.h/2.0 + system.robot.table_h - system.robot.wedge_h;
            }

            void setSystemCorrection(const std::array<RingMeasure::ScanCfg, static_cast<size_t>(rm::ScanLocation::NUM)> &correction)
            { 
                system.robot.scans = correction;
            }

            std::array<pcl::PointCloud<pcl::PointXYZ>::Ptr, NUM> rawRecs() const { return raw_recs; }

            std::array<pcl::PointCloud<pcl::PointNormal>::Ptr, NUM> getLaterals() const { return laterals; }

            std::array<pcl::PointCloud<pcl::PointNormal>::Ptr, NUM> getBases() const { return bases; }

            std::array<pcl::PointCloud<pcl::PointNormal>::Ptr, NUM> recNormals() const { return rec_normals; }

            std::array<std::array<float, axis_num>, NUM> radiusEval() const { return radius_eval; }

            std::array<std::array<std::array<float, 2>, axis_num>, NUM> angleEval() const { return angle_eval; }

            auto getScanNum() const { return system.sensor.scan_num; }

            /**
             * @brief Computes global multi-scan reconstruction evaluation metrics.
             * @return ReconMetricsArray Array containing calculated cross-scan metrics.
             */
            ReconMetricsArray reconEval() const {
                std::array<Eigen::Vector2f, NUM> center_points;
                std::transform(cyl_fit.begin(), cyl_fit.end(), center_points.begin(), [](const CYLFitMetricTuple& t) {
                    return Eigen::Vector2f{std::get<1>(t).x(), std::get<1>(t).z()};
                });
                std::array<Eigen::Vector3f, NUM> axises;
                std::transform(cyl_fit.begin(), cyl_fit.end(), axises.begin(), [](const CYLFitMetricTuple& t) {
                    return std::get<1>(t);
                });
                std::array<Eigen::Vector3f, NUM> base_normals;
                std::transform(base_fit.begin(), base_fit.end(), base_normals.begin(), [](const BaseFitMetricTuple& t) {
                    return std::get<1>(t);
                });
                // distance(max), cos(min), cos(min) 
                ReconMetricsArray ret{0.0, 2.0, 2.0};
                for (size_t i = 0; i < center_points.size(); ++i)
                    for (size_t j = i + 1; j < center_points.size(); ++j){
                        ret[static_cast<size_t>(ReconMetrics::CENTER_CYL_FIT)] =
                           std::max(ret[static_cast<size_t>(ReconMetrics::CENTER_CYL_FIT)], (center_points[i] - center_points[j]).norm());
                        ret[static_cast<size_t>(ReconMetrics::AXIS_CYL_FIT)] = 
                            std::min(ret[static_cast<size_t>(ReconMetrics::AXIS_CYL_FIT)], std::abs(axises[i].dot(axises[j])));
                        ret[static_cast<size_t>(ReconMetrics::BASE_PARALLELISM)] = 
                            std::min(ret[static_cast<size_t>(ReconMetrics::BASE_PARALLELISM)], std::abs(base_normals[i].dot(base_normals[j])));
                    }
                ret[static_cast<size_t>(ReconMetrics::AXIS_CYL_FIT)] = std::acos(ret[static_cast<size_t>(ReconMetrics::AXIS_CYL_FIT)]) * 180.0 / M_PI;
                ret[static_cast<size_t>(ReconMetrics::BASE_PARALLELISM)] = std::acos(ret[static_cast<size_t>(ReconMetrics::BASE_PARALLELISM)]) * 180.0 / M_PI;
                return ret; 
            }
            
            /**
             * @brief Computes metrics for the currently active scan location.
             * @return ScanMetricsArray Array containing calculated single-scan metrics.
             */
            ScanMetricsArray scanEval() const {
                ScanMetricsArray ret{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                ret[static_cast<size_t>(ScanMetrics::ANGLE_MAJOR_LINE_FIT)] = std::acos(angle_eval[loc][static_cast<size_t>(rm::AxisType::MAJOR)][0]) * 180.0 / M_PI;
                ret[static_cast<size_t>(ScanMetrics::ANGLE_MINOR_LINE_FIT)] = std::acos(angle_eval[loc][static_cast<size_t>(rm::AxisType::MINOR)][0]) * 180.0 / M_PI;
                ret[static_cast<size_t>(ScanMetrics::RADIUS_MAJOR_LINE_FIT)] = radius_eval[loc][static_cast<size_t>(rm::AxisType::MAJOR)];
                ret[static_cast<size_t>(ScanMetrics::RADIUS_MINOR_LINE_FIT)] = radius_eval[loc][static_cast<size_t>(rm::AxisType::MINOR)];
                ret[static_cast<size_t>(ScanMetrics::RADIUS_CYL_FIT)] = std::get<0>(cyl_fit[loc]);
                ret[static_cast<size_t>(ScanMetrics::H_PLANE_FIT)] = std::get<0>(base_fit[loc]);
                return ret;
            }

            rm::PhaseType phaseType() const { return phase_type; }
            void setPhaseType(const rm::PhaseType &phaseType) { phase_type = phaseType;}
            
            /**
             * @brief Executes the full calibration routine for a specific scan location.
             * @param scan Target scan location.
             * @param path Directory path for logging and exporting intermediate artifacts.
             */
            void calibration(const rm::ScanLocation& scan, const fs::path& path);

            /**
             * @brief Executes the verification routine for a specific scan location.
             * @param scan Target scan location.
             * @param path Directory path for output artifacts.
             * @param profile Flag indicating whether profiling output should be generated.
             */
            void verification(const rm::ScanLocation& scan, const fs::path& path,
                              bool profile=true);

            std::array<CYLFitMetricTuple, NUM> cylFit() const { return cyl_fit; }

            std::array<BaseFitMetricTuple, NUM> baseFit() const { return base_fit; }

            rm::ParamTune getParam() const { return param; }
            void setParam(const rm::ParamTune &param_) { param = param_; }

        protected:
            /**
             * @brief Checks if both measure and system configurations are ready for processing.
             */
            bool isReady() const {
                return measure_ready && system_ready;
            }

            void setScanLoc(const ScanLocation &scanLoc)
            {
                 scan_loc = scanLoc;
                 loc = static_cast<size_t>(scanLoc);
            }

            rm::ProcessType processType() const { return process_type; }
            void setProcessType(const rm::ProcessType &processType) { process_type = processType; }

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
                const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                fs::path path,
                const std::string prefix,
                pcl::PointCloud<pcl::PointNormal>::Ptr& lateral,
                pcl::PointCloud<pcl::PointNormal>::Ptr& base
            );
    
        protected:
            /// Active logger instance, default console
            std::shared_ptr<spdlog::logger> logger=spdlog::default_logger(); 
            ScanLocation scan_loc;                  ///< Current scan location
            size_t loc;                             ///< Current scan index
            
            rm::ParamTune param;                    ///< Calibration parameters
    
            rm::MeasureCfg measure;                 ///< Measure configuration
            rm::SystemCfg system;                   ///< System configuration
            int camera_resolution;                  ///< Camera resolution
            float mid_height;                       ///< Middle height of the scan
            float smallest_value;                   ///< Smallest value in the point cloud

            // result from cylinder fitting
            std::array<CYLFitMetricTuple, NUM> cyl_fit;

            // result from line fitting
            std::array<std::array<float, axis_num>, NUM> radius_eval;
            std::array<std::array<std::array<float, 2>, axis_num>, NUM> angle_eval;

            // result from plane fitting
            std::array<BaseFitMetricTuple, NUM> base_fit;

            std::array<pcl::PointCloud<pcl::PointXYZ>::Ptr, NUM> raw_recs{nullptr};
            std::array<pcl::PointCloud<pcl::PointNormal>::Ptr, NUM> laterals{nullptr};
            std::array<pcl::PointCloud<pcl::PointNormal>::Ptr, NUM> bases{nullptr};
            std::array<pcl::PointCloud<pcl::PointNormal>::Ptr, NUM> rec_normals{nullptr};
        
        private:
            rm::RingType ring_type=rm::RingType::GR; ///< Calibration ring type (fixed to smooth ring)
            rm::PhaseType phase_type;                ///< Active execution phase type
            rm::ProcessType process_type;            ///< Active process type
            bool measure_ready = false;              ///< Flag indicating measure configuration readiness
            bool system_ready = false;               ///< Flag indicating system configuration readiness
    };
}