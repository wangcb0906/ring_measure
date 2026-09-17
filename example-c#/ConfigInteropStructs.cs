using System.Text;
using System.Runtime.InteropServices;

namespace RingCalibration
{

    public static class Constants
    {
        // DO NOT MODIFY: This must match the C++ definition of RM_MAX_PATH_LEN
        // If you have to change this, ensure that the C++ and C# definitions are consistent.
        // Note: If path is longer than 256 characters, it will be truncated when marshalled to C++.
        public const int MaxPathLen = 256;
    }

    public enum ScanLoc
    {
        IN = 0,
        OUT = 1,
        UNDER = 2,
        NUM = 3
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct SensorCfgInterop
    {
        public int ScanNum;
        public int ScanRes;
        public int ValidLen;
        public float ZPitch;
        public float ZOft;
        public float XPitch;
        // DO NOT MODIFY: This must match the C++ definition
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 32)]
        public string Unit;
        public float MountHeight;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct Point2f
    {
        public float X;
        public float Y;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ScanCfgInterop
    {
        public float ViewAngle;
        public float AngleCorr;
        public Point2f PosCorr;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct RobotCfgInterop
    {
        // Maps to std::array<ScanCfg, NUM>
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = (int)ScanLoc.NUM)]
        public ScanCfgInterop[] Scans;

        public Point2f Arm;
        public float Dist;
        public float TableH;
        public float WedgeH;
    }
    
    [StructLayout(LayoutKind.Sequential)]
    public struct SystemCfgInterop
    {
        // Note: RobotCfg and ScanCfg must also be defined as matching managed structs on the C# side
        public RobotCfgInterop Robot; 
        public SensorCfgInterop Sensor;
    }
    
    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct RingCfgInterop
    {
        // DO NOT MODIFY: This must match the C++ definition
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 8)]
        public string SpkForm;
        [MarshalAs(UnmanagedType.I1)]
        public bool IsTurned;
        // DO NOT MODIFY: This must match the C++ definition
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 8)]
        public string Type;
        public float MkeTop;
        public float MkeBtm;
        public float MkeMid;
        public float DI;
        public float DA;
        public float H;
    }
    
    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct ImgCfgInterop
    {
        // DO NOT MODIFY: This must match the C++ definition
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = Constants.MaxPathLen)]
        public string Path;
        
        // 2D char array [rm::NUM][RM_MAX_PATH_LEN] -> [3][256] = 768 bytes total
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = Constants.MaxPathLen * (int)ScanLoc.NUM)] 
        public byte[] ImgsData; 
        
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = Constants.MaxPathLen * (int)ScanLoc.NUM)]
        public byte[] LumaImgsData;

        // --- GET HELPER ---
        public string GetFilename(ScanLoc loc, bool isLuma = false)
        {
            byte[] sourceArray = isLuma ? LumaImgsData : ImgsData;
            if (sourceArray == null) return string.Empty;
    
            int maxLen = Constants.MaxPathLen;
            int offset = (int)loc * maxLen;
            int length = 0;
            
            while (length < maxLen && offset + length < sourceArray.Length && sourceArray[offset + length] != 0)
            {
                length++;
            }
    
            return Encoding.ASCII.GetString(sourceArray, offset, length);
        }
    
        // --- SET HELPER ---
        public void SetFilename(ScanLoc loc, string filename, bool isLuma = false)
        {
            int maxLen = Constants.MaxPathLen;
            int totalSize = maxLen * (int)ScanLoc.NUM;
    
            // Ensure arrays are initialized before writing
            if (isLuma)
            {
                if (LumaImgsData == null || LumaImgsData.Length != totalSize)
                    LumaImgsData = new byte[totalSize];
            }
            else
            {
                if (ImgsData == null || ImgsData.Length != totalSize)
                    ImgsData = new byte[totalSize];
            }
    
            byte[] targetArray = isLuma ? LumaImgsData : ImgsData;
            int offset = (int)loc * maxLen;
    
            if (string.IsNullOrEmpty(filename))
            {
                Array.Clear(targetArray, offset, maxLen);
                return;
            }
    
            byte[] stringBytes = Encoding.ASCII.GetBytes(filename);
            int copyLength = Math.Min(stringBytes.Length, maxLen - 1); // Leave room for null terminator
    
            // Copy string bytes into position
            Array.Copy(stringBytes, 0, targetArray, offset, copyLength);
    
            // Explicitly zero out the rest of the 256-byte chunk
            for (int i = copyLength; i < maxLen; i++)
            {
                targetArray[offset + i] = 0;
            }
        }
    }
    
    [StructLayout(LayoutKind.Sequential)]
    public struct MeasureCfgInterop
    {
        public ImgCfgInterop Img;
        public RingCfgInterop Ring;

        // std::array<rm::Point2f, rm::NUM> -> Point2f[3]
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = (int)ScanLoc.NUM)]
        public Point2f[] RobotPos;
    }
}