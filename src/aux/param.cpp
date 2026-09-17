#include "param.hpp"
#include <iomanip> 
#include <inicpp.h>
#include <iostream>
#include <stdexcept>

namespace RingMeasure {

    std::string operationModeToString(OperationMode mode) {
        switch (mode) {
            case OperationMode::AUTO:   return "auto";
            case OperationMode::MANUAL: return "manual";
        }
        return "auto";
    }

    OperationMode stringToOperationMode(std::string_view str) {
        if (str == "auto")   return OperationMode::AUTO;
        if (str == "manual") return OperationMode::MANUAL;
        throw std::invalid_argument("Invalid string for OperationMode: " + std::string(str));
    }

    std::string modesToString(Modes mode) {
        switch (mode) {
            case Modes::MODE1:     return "mode1";
            case Modes::MODE2:     return "mode2";
            case Modes::MODE3:     return "mode3";
            case Modes::MODE_NUM:  break;
        }
        return "mode1";
    }

    Modes stringToModes(std::string_view str) {
        if (str == "mode1")  return Modes::MODE1;
        if (str == "mode2")  return Modes::MODE2;
        if (str == "mode3")  return Modes::MODE3;
        throw std::invalid_argument("Invalid string for Modes: " + std::string(str));
    }

    void loadParam(const std::filesystem::path& param_cfg, struct ParamTune& param)
    {
        ini::IniFile param_ini(param_cfg.string());

        // Euclidean section
        auto& euc = param_ini["Euclidean"];
        param.euclidean.ec_tol_offset           = euc["ec_tol_offset"].as<float>();
        param.euclidean.min_cluster_size        = euc["min_cluster_size"].as<int>();
        param.euclidean.under_cluster_size      = euc["under_cluster_size"].as<int>();
        param.euclidean.under_selected_seg_size = euc["under_selected_seg_size"].as<int>();
        param.euclidean.io_num_candidates       = euc["io_num_candidates"].as<int>();

        // RegionGrowing section
        auto& rg = param_ini["RegionGrowing"];
        param.reg_growing.radius_ratio             = rg["radius_ratio"].as<float>();
        param.reg_growing.residual_ratio           = rg["residual_ratio"].as<float>();
        param.reg_growing.min_cluster_size         = rg["min_cluster_size"].as<int>();
        param.reg_growing.num_neighbors            = rg["num_neighbors"].as<int>();
        param.reg_growing.curvature_type           = stringToOperationMode(rg["curvature_type"].as<std::string>());
        param.reg_growing.normal_type              = stringToOperationMode(rg["normal_type"].as<std::string>());
        param.reg_growing.curvature_thld           = rg["curvature_thld"].as<float>();
        // Convert smooth_thld from degrees (INI) to radians (Struct)
        param.reg_growing.smooth_thld              = rg["smooth_thld"].as<float>() * (static_cast<float>(M_PI) / 180.0f);
        param.reg_growing.cluster_size             = rg["cluster_size"].as<int>();
        param.reg_growing.curv_thld_mode           = stringToModes(rg["curv_thld_mode"].as<std::string>());
        param.reg_growing.curv_thld_ratio          = rg["curv_thld_ratio"].as<float>();
        param.reg_growing.normal_thld_cluster_size = rg["normal_thld_cluster_size"].as<int>();

        // PrimitiveFit section
        auto& pf = param_ini["PrimitiveFit"];
        param.primitive.ellipse_fit_num_thld       = pf["ellipse_fit_num_thld"].as<int>();
        param.primitive.ellipse_fit_max_iter       = pf["ellipse_fit_max_iter"].as<int>();
        param.primitive.ellipse_fit_dist_thld      = pf["ellipse_fit_dist_thld"].as<float>();
        param.primitive.slices_mad_enabled         = pf["slices_mad_enabled"].as<bool>();
        param.primitive.mad_thld                   = pf["mad_thld"].as<float>();
        param.primitive.cylinder_fit_dist_thld     = pf["cylinder_fit_dist_thld"].as<float>();
        param.primitive.cylinder_fit_normal_weight = pf["cylinder_fit_normal_weight"].as<float>();
        param.primitive.cylinder_fit_max_iter      = pf["cylinder_fit_max_iter"].as<int>();
        param.primitive.cylinder_fit_prob          = pf["cylinder_fit_prob"].as<float>();
        param.primitive.plane_fit_dist_thld        = pf["plane_fit_dist_thld"].as<float>();
        param.primitive.plane_fit_normal_weight    = pf["plane_fit_normal_weight"].as<float>();
        param.primitive.plane_fit_max_iter         = pf["plane_fit_max_iter"].as<int>();

        // MiscParam section
        auto& misc = param_ini["MiscParam"];
        param.misc.memory_save                   = misc["memory_save"].as<bool>();
        param.misc.leaf_size_before_EC           = misc["leaf_size_before_EC"].as<float>();
        param.misc.io_resampling_ratio_for_ne    = misc["io_resampling_ratio_for_ne"].as<float>();
        param.misc.under_resampling_ratio_for_ne = misc["under_resampling_ratio_for_ne"].as<float>();
        param.misc.resample_ratio                = misc["resample_ratio"].as<float>();
    }

    void dumpParam(const std::filesystem::path& param_cfg, const struct ParamTune& param, std::ios_base::openmode mode)
    {
        ini::IniFile param_ini;

        // Euclidean section
        auto& euc = param_ini["Euclidean"];
        euc["ec_tol_offset"]           = param.euclidean.ec_tol_offset;
        euc["min_cluster_size"]        = param.euclidean.min_cluster_size;
        euc["under_cluster_size"]      = param.euclidean.under_cluster_size;
        euc["under_selected_seg_size"] = param.euclidean.under_selected_seg_size;
        euc["io_num_candidates"]       = param.euclidean.io_num_candidates;

        // RegionGrowing section
        auto& rg = param_ini["RegionGrowing"];
        rg["radius_ratio"]             = param.reg_growing.radius_ratio;
        rg["residual_ratio"]           = param.reg_growing.residual_ratio;
        rg["min_cluster_size"]         = param.reg_growing.min_cluster_size;
        rg["num_neighbors"]            = param.reg_growing.num_neighbors;
        rg["curvature_type"]           = operationModeToString(param.reg_growing.curvature_type);
        rg["normal_type"]              = operationModeToString(param.reg_growing.normal_type);
        rg["curvature_thld"]           = param.reg_growing.curvature_thld;
        // Convert smooth_thld from radians (Struct) back to degrees (INI)
        rg["smooth_thld"]              = param.reg_growing.smooth_thld * (180.0f / static_cast<float>(M_PI));
        rg["cluster_size"]             = param.reg_growing.cluster_size;
        rg["curv_thld_mode"]           = modesToString(param.reg_growing.curv_thld_mode);
        rg["curv_thld_ratio"]          = param.reg_growing.curv_thld_ratio;

        // PrimitiveFit section
        auto& pf = param_ini["PrimitiveFit"];
        pf["ellipse_fit_num_thld"]       = param.primitive.ellipse_fit_num_thld;
        pf["ellipse_fit_max_iter"]       = param.primitive.ellipse_fit_max_iter;
        pf["ellipse_fit_dist_thld"]      = param.primitive.ellipse_fit_dist_thld;
        pf["slices_mad_enabled"]         = param.primitive.slices_mad_enabled;
        pf["mad_thld"]                   = param.primitive.mad_thld;
        pf["cylinder_fit_dist_thld"]     = param.primitive.cylinder_fit_dist_thld;
        pf["cylinder_fit_normal_weight"] = param.primitive.cylinder_fit_normal_weight;
        pf["cylinder_fit_max_iter"]      = param.primitive.cylinder_fit_max_iter;
        pf["cylinder_fit_prob"]          = param.primitive.cylinder_fit_prob;
        pf["plane_fit_dist_thld"]        = param.primitive.plane_fit_dist_thld;
        pf["plane_fit_normal_weight"]    = param.primitive.plane_fit_normal_weight;
        pf["plane_fit_max_iter"]         = param.primitive.plane_fit_max_iter;

        // MiscParam section
        auto& misc = param_ini["MiscParam"];
        misc["memory_save"]                   = param.misc.memory_save;
        misc["leaf_size_before_EC"]           = param.misc.leaf_size_before_EC;
        misc["io_resampling_ratio_for_ne"]    = param.misc.io_resampling_ratio_for_ne;
        misc["under_resampling_ratio_for_ne"] = param.misc.under_resampling_ratio_for_ne;
        misc["resample_ratio"]                = param.misc.resample_ratio;

        // Output logic
        if (param_cfg.string() == "stdout") {
            param_ini.encode(std::cout);
            return;
        }

        std::ofstream outFile(param_cfg.string(), mode);
        param_ini.encode(outFile);
    }
}