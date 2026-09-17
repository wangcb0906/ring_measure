#pragma once
#include <string>
#include <ios>
#include <filesystem>

namespace RingMeasure {

    #define M_PI 3.14159265358979323846

    /// @brief  Operating mode for threshold determination.
    enum class OperationMode{
        /// The threshold is manually tuned by the user in the configuration file.
        MANUAL,
        /// The threshold is automatically determined based on underlying statistics.
        AUTO,
    };
   
    /// @brief Moodes for threshold tuning, it varies for different scenarios.
    /// See in each specific case for the recommended values.
    enum class Modes {
        MODE1,
        MODE2,
        MODE3,
        MODE_NUM
    };
    
    /**
     * @brief Configuration parameters for Euclidean segmentation.
     * 
     * Contains all tuning handles for threshold offsets, cluster sizes, 
     * and candidate counts used in the Euclidean segmentation phase.
     */
    struct Euclidean{
        /// Offset added to the tolerance for Euclidean segmentation
        float ec_tol_offset = 1.5f;
        /// Minimum cluster size for Euclidean segmentation, usually for debugging
        int min_cluster_size = 100;
        /// Cluster size for the underneath scan, based on observation
        int under_cluster_size = 2;
        /// Number of segments to select for the underneath scan, based on observation
        int under_selected_seg_size = 4;
        /// Number of candidates to select for the in and out scan, based on observation
        int io_num_candidates = 5;
    };

    /**
     * @brief Configuration parameters for Region Growing.
     * 
     * Contains all tuning handles for threshold offsets, cluster sizes, 
     * and candidate counts used in the Region Growing phase.
     */
    struct RegionGrowing{
        /// The search radius for normal estimation, radius = radius_ratio * camera_resolution.
        float radius_ratio = 1.2f; 
        /// The residual threshold is used to filter out points that are too far from the fitted model.
        /// residual = residual_ratio * camera_resolution.
        float residual_ratio = 0.75f; 
        /// The minimum cluster size for region growing, used to filter out small clusters.
        int min_cluster_size = 1l; 
        /// The number of neighbors to consider for each point during region growing,
        int num_neighbors = 8l;
        /// The curvature threshold type for region growing, can be AUTO or MANUAL.
        OperationMode curvature_type = OperationMode::AUTO;
        /// The normal threshold type for region growing, can be AUTO or MANUAL.
        OperationMode normal_type = OperationMode::AUTO;
        /// The curvature threshold for region growing,
        /// does not take effect if curvature_type is AUTO
        float curvature_thld = 0.1f;
        /// The smooth threshold for region growing,
        /// does not take effect if normal_type is AUTO
        float smooth_thld = 10.0f * M_PI / 180.0f; // Convert degrees to radians
        /// The number of clusters is set based on the observation of the point cloud.
        int cluster_size = 3;
        /// The normal threshold mode for region growing, can be MODE1, MODE2.
        /// Mode1 is driven by kmeans clustering.
        /// Mode2 is driven by the percentile setting, which depends on an extra parameter.
        /// Mode3 takes the smaller value from MODE1 and MODE2, which is more robust to noise
        /// at the risk of discarding some valid points.
        Modes curv_thld_mode = Modes::MODE3;
        /// This parameter depends on the number of layers you want to remove. 
        /// See point cloud visualization for more details. 
        /// It only takes effect when curvature_thld_mode is MODE2 or MODE3.
        float curv_thld_ratio = 3.0;
        /// The number of clusters fo k-means to determine the normal threshold.
        /// Tune this parameter only based on observation of the point cloud,
        /// and do not change it without a good reason.
        int normal_thld_cluster_size = 4;
    };

    /**
     * @brief Configuration parameters for Primitive Fitting.
     * 
     * Contains all tuning handles for Ellipse fitting, Cylinder fitting,
     * and Plane fitting, 
     */
    struct PrimitiveFit{
        /// The number of points to fit an ellipse,
        /// at least 1% of the total points or 120 points.
        int ellipse_fit_num_thld = 120;
        /// The maximum number of iterations for ellipse fitting.
        int ellipse_fit_max_iter = 2000;
        /// The distance threshold for ellipse fitting.
        float ellipse_fit_dist_thld = 0.1f;
        /// If true, use MAD to estimate the radius from the slices,
        /// otherwise, use cylinder fitting.
        bool slices_mad_enabled = true;
        /// The threshold for MAD to detect outliers.
        /// See mad.hpp to tune the threshold for outlier detection.
        float mad_thld = 2.0f;
        /// The distance threshold for cylinder fitting.
        /// Set the threshold to roughly 1.5x to 2x the sensor's noise level.
        /// Too small a threshold may result in too few inliers, while too large a threshold may include outliers.
        /// Please refer to the sensor's datasheet for the noise level, or use the empirical default value.
        /// The default value of 2.0mm.
        float cylinder_fit_dist_thld = 2.0f; 
        /// The normal and distance weight for cylinder fitting.
        float cylinder_fit_normal_weight = 0.1f; 
        /// The maximum number of iterations for cylinder fitting.
        int cylinder_fit_max_iter = 10000; 
        /// The probability for cylinder fitting, a higher value forces a more conservative fit.
        float cylinder_fit_prob = 0.999f;
        /// The distance threshold for plane fitting.
        float plane_fit_dist_thld = 0.1f;
        /// The normal and distance weight for plane fitting.
        float plane_fit_normal_weight = 0.1f; 
        /// The maximum number of iterations for plane fitting.
        int plane_fit_max_iter = 1000; 
    };

    /**
     * @brief Configuration parameters for miscellaneous settings.
     * 
     */
    struct MiscParam{
        /// If true, the point cloud will be released when not needed,
        /// otherwise, the point cloud will be kept in memory for later use.
        bool memory_save = true;
        /// The leaf size for downsampling the point cloud before Euclidean segmentation.
        /// To reduce the number of points and to accelerate Euclidean segmentation.
        float leaf_size_before_EC = 1.5f;
        /// The resampling ratio for the in and out scan in y direction, based on observation.
        float io_resampling_ratio_for_ne = 1.0f;
        /// The resampling ratio for the underneath scan in y direction, based on observation.
        float under_resampling_ratio_for_ne = 0.5f;
        /// The ratio of resampling to camera resolution.
        /// Ratio = resample / camera_resolution, camera_resolution = da * PI / 360.0f;
        float resample_ratio = 0.75f;
    };

    /**
     * @brief All configuration parameters.
     * 
     */
    struct ParamTune{
         /// Configuration parameters for Euclidean segmentation.
         struct Euclidean euclidean;
         /// Configuration parameters for Region Growing.
         struct RegionGrowing reg_growing;
         /// Configuration parameters for Primitive Fitting.
         struct PrimitiveFit primitive;
         /// Configuration parameters for miscellaneous settings.
         struct MiscParam misc;
    };

    /**
     * @brief Loads tuning parameters from an INI configuration file.
     * 
     * @param param_cfg Path to the INI configuration file to read from.
     * @param param Reference to the ParamTune struct where the loaded values will be stored.
     * @throws std::invalid_argument If an invalid enum string is encountered.
     */
    void loadParam(const std::filesystem::path& param_cfg, struct ParamTune& param);

    /**
     * @brief Dumps tuning parameters to an INI configuration file.
     * If the file is setting to "stdout", the parameters will be printed to the console
     * instead of being written to a file.
     * 
     * @param param_cfg Path to the INI configuration file to write to.
     * @param param Reference to the ParamTune struct containing the values to be dumped.
     * @param mode The open mode for the output file.
     */
    void dumpParam(const std::filesystem::path& param_cfg, const struct ParamTune& param,
                   std::ios_base::openmode mode=std::ios_base::out);
}
