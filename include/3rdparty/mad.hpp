#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include <numeric>

/**
 * @brief An outlier detector using the Median Absolute Deviation (MAD) method.
 * @tparam T Numeric data type of the input values (e.g., float, double).
 */
template <typename T>
class MADDetector {
public:
    /// Calculated decision scores for each data point
    std::vector<double> decision_scores_; 
    /// Classification labels: true = Inlier (keep), false = Outlier (discard)
    std::vector<bool> labels_;

    /**
     * @brief Construct a new MADDetector object.
     * @details Modified Z-score Threshold Guidelines (Normal distribution coverage):
     *          - 1.0: Captures ~68.27% of data (Standard 1-sigma equivalent; aggressive filtering)
     *          - 1.2: Captures ~76.99% of data
     *          - 1.4: Captures ~83.85% of data
     *          - 1.5: Captures ~86.64% of data
     *          - 1.6: Captures ~89.04% of data
     *          - 1.8: Captures ~92.81% of data
     *          - 2.0: Captures ~95.45% of data (Standard 2-sigma equivalent)
     *          - 3.0: Captures ~99.73% of data (Standard 3-sigma equivalent)
     *          - 3.5: Captures ~99.95% of data (Default / Recommended sweet spot)
     *          - 4.0: Captures ~99.99% of data (Ultra-conservative)
     * 
     * @param threshold Modified Z-score threshold for outlier detection.
     */
    explicit MADDetector(double threshold = 3.5) 
        : threshold_(threshold), median_(0), median_diff_(0) {}

    /**
     * @brief Sets the input data container explicitly.
     * @param X Vector of raw input values to process.
     */
    void setData(const std::vector<T>& X) {
        data_ = X;
    }

    /**
     * @brief Fits the MAD detector using the internally stored data.
     * @details Computes the median, absolute deviations, scales the MAD to match 
     *          standard deviation (using the 0.6745 inverse factor), calculates 
     *          decision scores, and labels points as inliers or outliers.
     */
    void fit() {
        if (data_.empty()) return;

        median_ = computeMedian(data_);

        std::vector<T> diffs;
        diffs.reserve(data_.size());
        for (const auto& val : data_) {
            diffs.push_back(std::abs(val - median_));
        }

        median_diff_ = computeMedian(diffs);

        decision_scores_.clear();
        decision_scores_.reserve(data_.size());
        labels_.clear();
        labels_.reserve(data_.size());

        for (size_t i = 0; i < data_.size(); ++i) {
            double score = 0.0;
            if (median_diff_ != 0) {
                // 0.6745 is the scaling factor (inverse of 1.4826).
                // A normal distribution has a standard deviation of 1.4826 * MAD,
                // so we scale the MAD to match the standard deviation.
                score = 0.6745 * static_cast<double>(diffs[i]) / static_cast<double>(median_diff_);
            }
            decision_scores_.push_back(score);

            // true for inliers, false for outliers
            if (score > threshold_) {
                labels_.push_back(false); // Outlier -> discard
            } else {
                labels_.push_back(true);  // Inlier -> keep
            }
        }
    }

    /**
     * @brief Computes the mean of the identified inlier points.
     * @tparam RetT The return data type for the mean calculation (defaults to double).
     * @return RetT The mean value of all filtered inlier points, or default if empty.
     */
    template <typename RetT = double>
    RetT getInlierMean() const {
        if (data_.empty() || data_.size() != labels_.size()) return RetT{};

        double sum = 0.0;
        size_t count = 0;
        for (size_t i = 0; i < data_.size(); ++i) {
            if (labels_[i]) { // true = Inlier
                sum += static_cast<double>(data_[i]);
                ++count;
            }
        }


        if (count == 0) return RetT{};
        return static_cast<T>(sum / static_cast<double>(count));
    }

private:
    double threshold_;   ///< Current modified Z-score threshold limit
    std::vector<T> data_;///< Stored internal raw data vector
    T median_;           ///< Computed median of the raw data
    T median_diff_;      ///< Computed median of the absolute deviations (MAD)

    /**
     * @brief Helper function to compute the median of a data vector.
     * @param data Vector of values to compute the median for.
     * @return T The calculated median value.
     */
    T computeMedian(std::vector<T> data) {
        if (data.empty()) return T{};
        size_t n = data.size();
        std::sort(data.begin(), data.end());
        if (n % 2 == 0) {
            return (data[n / 2 - 1] + data[n / 2]) / static_cast<T>(2);
        } else {
            return data[n / 2];
        }
    }
};