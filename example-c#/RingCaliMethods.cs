using System;
using System.Runtime.InteropServices;

namespace RingCalibration
{
    internal static class RingCaliMethods
    {
        // Set the correct DLL name for your native library here.
        private const string DllName = "libs/libcore_shared.dll";

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr RM_RingCalibration_New();

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void RM_RingCalibration_Delete(IntPtr ptr);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool RM_RingCalibration_SetSystem(IntPtr ptr, ref SystemCfgInterop interopSystem);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool RM_RingCalibration_GetSystem(IntPtr ptr, out SystemCfgInterop outSystem);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool RM_RingCalibration_SetMeasure(IntPtr ptr, ref MeasureCfgInterop interopCfg);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool RM_RingCalibration_GetMeasure(IntPtr ptr, out MeasureCfgInterop outInteropCfg);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool RM_RingCalibration_SetParam(IntPtr ptr, ref ParamTuneInterop param);
        
        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool RM_RingCalibration_GetParam(IntPtr ptr, out ParamTuneInterop outParam);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool RM_RingCalibration_Calibration(IntPtr ptr, ScanLoc scan, [MarshalAs(UnmanagedType.LPStr)] string path);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool RM_RingCalibration_Verification(IntPtr ptr, ScanLoc scan, [MarshalAs(UnmanagedType.LPStr)] string path);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool RM_RingCalibration_GetSystemCorrection(IntPtr ptr, [Out] ScanCfgInterop[] outBuffer, int bufferSize);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        public static extern bool RM_RingCalibration_SetSystemCorrection(IntPtr ptr, [In] ScanCfgInterop[] correctionArray, int arraySize);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        [return: MarshalAs(UnmanagedType.U1)]
        public static extern bool RM_RingCalibration_SetLogger(
        IntPtr ptr,
        [MarshalAs(UnmanagedType.LPStr)] string loggerName,
        [MarshalAs(UnmanagedType.LPStr)] string logPath,
        [MarshalAs(UnmanagedType.U1)] bool overwrite);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        [return: MarshalAs(UnmanagedType.U1)]
        public static extern bool RM_RingCalibration_GetLogger(
        IntPtr ptr,
        [Out, MarshalAs(UnmanagedType.LPStr)] System.Text.StringBuilder nameBuffer, int nameBufferSize,
        [Out, MarshalAs(UnmanagedType.LPStr)] System.Text.StringBuilder pathBuffer, int pathBufferSize);
    }
}