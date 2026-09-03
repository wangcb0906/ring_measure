#pragma once

#include "config.hpp"
#include <Eigen/Core>
#include <spdlog/spdlog.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace RingMeasure {

    namespace rm = RingMeasure;
    namespace fs=std::filesystem;

    class RingCalibration{


        public:
            
            RingCalibration() = default;
        
            RingCalibration(const rm::MeasureCfg& measure, const rm::SystemCfg& system)
             : measure(measure), system(system)
            {
                logger = spdlog::default_logger();;
                camera_resolution = measure.ring.dA * M_PI / 360.0f;
                resample = camera_resolution * 0.75f;
                mid_height = measure.ring.h/2.0 + system.robot.table_h - system.robot.wedge_h;
                smallest_value = system.sensor.z_oft/1.0e5  - system.sensor.mount_height;
            };
            
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

            enum class ScanMetrics{
                // metrics for the bases
                H_PLANE_FIT,           // yes base_fit[0][0]
                // metrics for the lateral itself
                ANGLE_MAJOR_LINE_FIT,  // yes angle_eval[0][0]
                ANGLE_MINOR_LINE_FIT,  // yes angle_eval[0][1]
                RADIUS_MAJOR_LINE_FIT, // yes radius_eval[0][1]
                RADIUS_MINOR_LINE_FIT, // yes radius_eval[0][1]
                RADIUS_CYL_FIT,        // yes cyl_fit_eval[0][0]
                NUM
            };
            
            enum class ReconMetrics{
                // metrics for the lateral
                CENTER_CYL_FIT,        // yes cyl_fit_eval[0][1]
                AXIS_CYL_FIT,          // yes cyl_fit_eval[0][2]
                // metrics for the bases
                BASE_PARALLELISM,      // yes base_fit[0][1]
                NUM
            };

            // height, normal
            using BaseFitMetricTuple = std::tuple<float, Eigen::Vector3f>;
            // radius, axis, center
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
                resample = camera_resolution * 0.75f;
                mid_height = measure.ring.h/2.0 + system.robot.table_h - system.robot.wedge_h;
            }

            rm::SystemCfg getSystem() const { return system; }
            void setSystem(const rm::SystemCfg &system_)
            { 
                system = system_; 
                smallest_value = system.sensor.z_oft/1.0e5  - system.sensor.mount_height;
            }

            std::array<float, NUM> angleEst() const { return angle_est; }

            std::array<Point2f, NUM> posEst() const { return pos_est; }

            std::array<pcl::PointCloud<pcl::PointXYZ>::Ptr, NUM> rawRecs() const { return raw_recs; }

            std::array<pcl::PointCloud<pcl::PointNormal>::Ptr, NUM> getLaterals() const { return laterals; }

            std::array<pcl::PointCloud<pcl::PointNormal>::Ptr, NUM> getBases() const { return bases; }

            std::array<pcl::PointCloud<pcl::PointNormal>::Ptr, NUM> recNormals() const { return rec_normals; }

            std::array<std::array<float, axis_num>, NUM> radiusEval() const { return radius_eval; }

            std::array<std::array<float, axis_num>, NUM> angleEval() const { return angle_eval; }

            // enum class ReconMetrics{
            //     // metrics for the lateral
            //     CENTER_CYL_FIT,        // yes cyl_fit_eval[0][1]
            //     AXIS_CYL_FIT,          // yes cyl_fit_eval[0][2]
            //     // metrics for the bases
            //     BASE_PARALLELISM,      // yes base_fit[0][1]
            //     NUM
            // };

            ReconMetricsArray reconEval() const {
                std::array<Eigen::Vector2f, NUM> center_points;
                std::transform(cyl_fit.begin(), cyl_fit.end(), center_points.begin(), [](const CYLFitMetricTuple& t) {
                    return Eigen::Vector2f{std::get<1>(t).x(), std::get<1>(t).z()};
                });
                std::array<Eigen::Vector3f, NUM> axises;
                std::transform(cyl_fit.begin(), cyl_fit.end(), axises.begin(), [](const CYLFitMetricTuple& t) {
                    return std::get<2>(t);
                });
                std::array<Eigen::Vector3f, NUM> base_normals;
                std::transform(base_fit.begin(), base_fit.end(), base_normals.begin(), [](const BaseFitMetricTuple& t) {
                    return std::get<1>(t);
                });
                // distance(max), cos(min), cos(min) 
                ReconMetricsArray ret{0.0,-1.0, -1.0};
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

            ScanMetricsArray scanEval() const {
                ScanMetricsArray ret{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                ret[static_cast<size_t>(ScanMetrics::ANGLE_MAJOR_LINE_FIT)] = angle_eval[loc][static_cast<size_t>(rm::AxisType::MAJOR)];
                ret[static_cast<size_t>(ScanMetrics::ANGLE_MINOR_LINE_FIT)] = angle_eval[loc][static_cast<size_t>(rm::AxisType::MINOR)];
                ret[static_cast<size_t>(ScanMetrics::RADIUS_MAJOR_LINE_FIT)] = radius_eval[loc][static_cast<size_t>(rm::AxisType::MAJOR)];
                ret[static_cast<size_t>(ScanMetrics::RADIUS_MINOR_LINE_FIT)] = radius_eval[loc][static_cast<size_t>(rm::AxisType::MINOR)];
                ret[static_cast<size_t>(ScanMetrics::RADIUS_CYL_FIT)] = std::get<0>(cyl_fit[loc]);
                ret[static_cast<size_t>(ScanMetrics::H_PLANE_FIT)] = std::get<0>(base_fit[loc]);
                return ret;
            }

            rm::PhaseType phaseType() const { return phase_type; }
            void setPhaseType(const rm::PhaseType &phaseType) { phase_type = phaseType;}

            void calibration(const rm::ScanLocation& scan, const fs::path& path);

            void verification(const rm::ScanLocation& scan, const fs::path& path,
                              bool profile=true);

            std::array<CYLFitMetricTuple, NUM> cylFit() const { return cyl_fit; }

            std::array<BaseFitMetricTuple, NUM> baseFit() const { return base_fit; }

        protected:
        
            void setScanLoc(const ScanLocation &scanLoc)
            {
                 scan_loc = scanLoc;
                 loc = static_cast<size_t>(scanLoc);
            }

            rm::ProcessType processType() const { return process_type; }
            void setProcessType(const rm::ProcessType &processType) { process_type = processType; }

            void reconstruct(std::string path);
            void denoise(fs::path path_den, fs::path path_seg, bool free_raw=true);
            void caliRot(fs::path path);
            void caliTrans(fs::path path);
    
        protected:

            std::shared_ptr<spdlog::logger> logger;
            ScanLocation scan_loc;
            size_t loc;
    
            rm::MeasureCfg measure;
            rm::SystemCfg system;
            int camera_resolution;
            float resample;
            float mid_height;
            float smallest_value;

            // result from cylinder fitting
            std::array<float, NUM> angle_est;
            std::array<Point2f, NUM> pos_est;
            std::array<CYLFitMetricTuple, NUM> cyl_fit;

            // result from line fitting
            std::array<std::array<float, axis_num>, NUM> radius_eval;
            std::array<std::array<float, axis_num>, NUM> angle_eval;
            std::array<BaseFitMetricTuple, NUM> base_fit;

            std::array<pcl::PointCloud<pcl::PointXYZ>::Ptr, NUM> raw_recs{nullptr};
            std::array<pcl::PointCloud<pcl::PointNormal>::Ptr, NUM> laterals{nullptr};
            std::array<pcl::PointCloud<pcl::PointNormal>::Ptr, NUM> bases{nullptr};
            std::array<pcl::PointCloud<pcl::PointNormal>::Ptr, NUM> rec_normals{nullptr};
        
        private:
            // Only allow glatt ring as the calibration ring type.
            rm::RingType ring_type=rm::RingType::GR;
            rm::PhaseType phase_type;
            rm::ProcessType process_type;



    };
}