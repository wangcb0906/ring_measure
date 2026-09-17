using System;
using RingCalibration;

namespace RingCalibrationTest
{
    public class RingCalibration : IDisposable
    {
        private IntPtr _nativePtr;
        private bool _disposed = false;

        public RingCalibration()
        {
            _nativePtr = RingCaliMethods.RM_RingCalibration_New();
            if (_nativePtr == IntPtr.Zero)
            {
                throw new Exception("Failed to create native RingCalibration instance.");
            }
        }

        // ==========================================
        // System Configuration
        // ==========================================

        public bool SetSystem(ref SystemCfgInterop systemCfg)
        {
            return RingCaliMethods.RM_RingCalibration_SetSystem(_nativePtr, ref systemCfg);
        }

        public bool GetSystem(out SystemCfgInterop outSystem)
        {
            return RingCaliMethods.RM_RingCalibration_GetSystem(_nativePtr, out outSystem);
        }

        // ==========================================
        // Measurement Configuration
        // ==========================================

        public bool SetMeasure(ref MeasureCfgInterop measureCfg)
        {
            return RingCaliMethods.RM_RingCalibration_SetMeasure(_nativePtr, ref measureCfg);
        }

        public bool GetMeasure(out MeasureCfgInterop outMeasureCfg)
        {
            return RingCaliMethods.RM_RingCalibration_GetMeasure(_nativePtr, out outMeasureCfg);
        }

        // ==========================================
        // Calibration & Verification Routines
        // ==========================================

        public bool Calibration(ScanLoc scan, string path)
        {
            return RingCaliMethods.RM_RingCalibration_Calibration(_nativePtr, scan, path);
        }

        public bool Verification(ScanLoc scan, string path)
        {
            return RingCaliMethods.RM_RingCalibration_Verification(_nativePtr, scan, path);
        }

        // ==========================================
        // System Correction Arrays
        // ==========================================

        public bool GetSystemCorrection(ScanCfgInterop[] outBuffer, int bufferSize)
        {
            return RingCaliMethods.RM_RingCalibration_GetSystemCorrection(_nativePtr, outBuffer, bufferSize);
        }

        public bool SetSystemCorrection(ScanCfgInterop[] correctionArray, int arraySize)
        {
            return RingCaliMethods.RM_RingCalibration_SetSystemCorrection(_nativePtr, correctionArray, arraySize);
        }

        // ==========================================
        // Tuning Parameters (ParamTune)
        // ==========================================

        public bool SetParam(ref ParamTuneInterop param)
        {
            return RingCaliMethods.RM_RingCalibration_SetParam(_nativePtr, ref param);
        }

        public bool GetParam(out ParamTuneInterop outParam)
        {
            return RingCaliMethods.RM_RingCalibration_GetParam(_nativePtr, out outParam);
        }

        // ==========================================
        // Logging Configuration
        // ==========================================

        public bool SetLogger(string loggerName, string logPath, bool overwrite = false)
        {
            return RingCaliMethods.RM_RingCalibration_SetLogger(_nativePtr, loggerName, logPath, overwrite);
        }

        public bool GetLogger(out string loggerName, out string logPath)
        {
            var nameBuilder = new System.Text.StringBuilder(Constants.MaxPathLen);
            var pathBuilder = new System.Text.StringBuilder(Constants.MaxPathLen);
        
            bool success = RingCaliMethods.RM_RingCalibration_GetLogger(
                _nativePtr, 
                nameBuilder, nameBuilder.Capacity, 
                pathBuilder, pathBuilder.Capacity
            );
        
            if (success)
            {
                loggerName = nameBuilder.ToString();
                logPath = pathBuilder.ToString();
                return true;
            }
        
            loggerName = string.Empty;
            logPath = string.Empty;
            return false;
        }

        // ==========================================
        // Memory Cleanup (IDisposable Pattern)
        // ==========================================

        protected virtual void Dispose(bool disposing)
        {
            if (!_disposed)
            {
                if (_nativePtr != IntPtr.Zero)
                {
                    RingCaliMethods.RM_RingCalibration_Delete(_nativePtr);
                    _nativePtr = IntPtr.Zero;
                }
                _disposed = true;
            }
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        ~RingCalibration()
        {
            Dispose(false);
        }
    }
}