using System;
using System.Runtime.InteropServices;

namespace RingCalibration
{
    public enum OperationMode
    {
        MANUAL = 0,
        AUTO = 1
    }
    
    public enum Modes
    {
        MODE1 = 0,
        MODE2 = 1,
        MODE3 = 2,
        MODE_NUM = 3
    }
    
    [StructLayout(LayoutKind.Sequential)]
    public struct EuclideanInterop
    {
        public float EcTolOffset;
        public int MinClusterSize;
        public int UnderClusterSize;
        public int UnderSelectedSegSize;
        public int IoNumCandidates;
    }
    
    [StructLayout(LayoutKind.Sequential)]
    public struct RegionGrowingInterop
    {
        public float RadiusRatio;
        public float ResidualRatio;
        public int MinClusterSize;
        public int NumNeighbors;
        public OperationMode CurvatureType;
        public OperationMode NormalType;
        public float CurvatureThld;
        public float SmoothThld;
        public int ClusterSize;
        public Modes CurvThldMode;
        public float CurvThldRatio;
        public int NormalThldClusterSize;
    }
    
    [StructLayout(LayoutKind.Sequential)]
    public struct PrimitiveFitInterop
    {
        public int EllipseFitNumThld;
        public int EllipseFitMaxIter;
        public float EllipseFitDistThld;
        
        [MarshalAs(UnmanagedType.I1)]
        public bool SlicesMadEnabled;
        
        public float MadThld;
        public float CylinderFitDistThld;
        public float CylinderFitNormalWeight;
        public int CylinderFitMaxIter;
        public float CylinderFitProb;
        public float PlaneFitDistThld;
        public float PlaneFitNormalWeight;
        public int PlaneFitMaxIter;
    }
    
    [StructLayout(LayoutKind.Sequential)]
    public struct MiscParamInterop
    {
        [MarshalAs(UnmanagedType.I1)]
        public bool MemorySave;
        
        public float LeafSizeBeforeEC;
        public float IoResamplingRatioForNe;
        public float UnderResamplingRatioForNe;
        public float ResampleRatio;
    }
    
    [StructLayout(LayoutKind.Sequential)]
    public struct ParamTuneInterop
    {
        public EuclideanInterop Euclidean;
        public RegionGrowingInterop RegGrowing;
        public PrimitiveFitInterop Primitive;
        public MiscParamInterop Misc;
    }
}