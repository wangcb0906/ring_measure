using System.Text;
using RingCalibration;

namespace RingCalibrationTest
{
    class Program
    {
        static void Main(string[] args)
        {
            using var calibrator = new RingCalibration();
            Console.WriteLine("Creating and setting System Configuration...");

            // 1. Declare and initialize SystemCfgInterop
            SystemCfgInterop sysCfg = new SystemCfgInterop();

            // If RobotCfg has fields, populate them here:
            // sysCfg.Robot.SomeField = ...;

            // Populate Sensor configuration
            sysCfg.Sensor.ScanNum = 360;
            sysCfg.Sensor.ScanRes = 3200;
            sysCfg.Sensor.ValidLen = 720;
            sysCfg.Sensor.ZPitch = 1600.0f;
            sysCfg.Sensor.ZOft = -5.24288e+07f;
            sysCfg.Sensor.XPitch = 22500.0f;
            // sysCfg.Sensor.Unit = "mm";
            sysCfg.Sensor.MountHeight = 980.0f;

            sysCfg.Robot.Scans = new ScanCfgInterop[(int)ScanLoc.NUM];
            sysCfg.Robot.Scans[(int)ScanLoc.IN] = new ScanCfgInterop
            {
                ViewAngle = -50.0f,
                AngleCorr = 0.0f,
                PosCorr = new Point2f { X = -762.3377f, Y = 129.76f }
            };
            sysCfg.Robot.Scans[(int)ScanLoc.OUT] = new ScanCfgInterop
            {
                ViewAngle = -50.0f,
                AngleCorr = 0.0f,
                PosCorr = new Point2f { X = -761.385f, Y = 125.19f }
            };
            sysCfg.Robot.Scans[(int)ScanLoc.UNDER] = new ScanCfgInterop
            {
                ViewAngle = 20.5f,
                AngleCorr = 0.0f,
                PosCorr = new Point2f { X = 196.63f, Y = -704.341f }
            };

            // These variables are commented out because they DO NOT make a difference
            // in the calibration process, but you can set them if needed.
            // sysCfg.Robot.Arm = new Point2f { X = 534.0f, Y = 734.5f };
            // sysCfg.Robot.Dist = 189.0f;
            sysCfg.Robot.TableH = 892.0f;
            sysCfg.Robot.WedgeH = 10.0f;
            // Adjust TableH by subtracting WedgeH
            // This is done in the C++ code, DO NOT REPEAT IT HERE.
            // sysCfg.Robot.TableH -= sysCfg.Robot.WedgeH;

            // Call SetSystem  
            bool sysSuccess = calibrator.SetSystem(ref sysCfg);
            Console.WriteLine($"SetSystem Result: {sysSuccess}");

            // To check if the system configuration was set correctly, you can retrieve it back:
            bool sysGetSuccess = calibrator.GetSystem(out SystemCfgInterop retrievedSysCfg);
            Console.WriteLine($"GetSystem Result: {sysGetSuccess}");

            Console.WriteLine("Creating and setting Measure Configuration...");

            // 2. Declare and initialize MeasureCfgInterop
            MeasureCfgInterop measCfg = new MeasureCfgInterop();
            measCfg.Img = new ImgCfgInterop();
            measCfg.Ring = new RingCfgInterop();
            measCfg.RobotPos = new Point2f[(int)ScanLoc.NUM];

            // Populate Image configuration
            measCfg.Img.Path = @"input";

            // CRITICAL: Arrays inside structs MUST be allocated before marshalling!
            // Size: RM_MAX_PATH_LEN (256) * rm::NUM (3) = 768 bytes
            // The function SetFilename will handle memory allocation.
            // So you DO NOT need to manually allocate the arrays here.
            // measCfg.Img.ImgsData = new byte[Constants.MaxPathLen * (int)ScanLoc.NUM];
            // Take 20260622/4_ND_20260622_4_08 as an example:
            measCfg.Img.SetFilename(ScanLoc.IN,    @"ND########_00004_00_R01_20260622_101647_B0.tif");
            measCfg.Img.SetFilename(ScanLoc.OUT,   @"ND########_00004_00_R01_20260622_101703_B1.tif");
            measCfg.Img.SetFilename(ScanLoc.UNDER, @"ND########_00004_00_R01_20260622_101718_B2.tif");
            // DO NOT need to fill this array because the C++ code does not use it.
            // measCfg.Img.LumaImgsData = new byte[Constants.MaxPathLen * (int)ScanLoc.NUM];

            // Populate Ring configuration
            // DO NOT need to set SpkForm, IsTurned, Type.
            // measCfg.Ring.SpkForm = "";
            // measCfg.Ring.IsTurned = false;
            // measCfg.Ring.Type = "GR";
            // DO NOT need to set MkeTop, MkeBtm, MkeMid.
            // measCfg.Ring.MkeTop = 81.0f;
            // measCfg.Ring.MkeBtm = 45.0f;
            // measCfg.Ring.MkeMid = 9.0f;
            measCfg.Ring.DI = 810.0f;
            measCfg.Ring.DA = 990.0f;
            measCfg.Ring.H = 90.0f;

            // Populate RobotPos array (size must match rm::NUM = 3)
            measCfg.RobotPos = new Point2f[(int)ScanLoc.NUM];
            measCfg.RobotPos[(int)ScanLoc.IN]    = new Point2f { X = 855.0f,  Y = 1500.0f };
            measCfg.RobotPos[(int)ScanLoc.OUT]   = new Point2f { X = -55.0f,  Y = 1500.0f };
            measCfg.RobotPos[(int)ScanLoc.UNDER] = new Point2f { X = 1130.0f, Y = 1320.0f };

            // Call SetMeasure
            bool measSuccess = calibrator.SetMeasure(ref measCfg);
            Console.WriteLine($"SetMeasure Result: {measSuccess}");

            bool measGetSuccess = calibrator.GetMeasure(out MeasureCfgInterop retrievedMeasCfg);
            Console.WriteLine($"GetMeasure Result: {measGetSuccess}");
            for (int i = 0; i < (int)ScanLoc.NUM; i++)
            {
                Console.WriteLine($"RobotPos[{i}]: X = {retrievedMeasCfg.RobotPos[i].X}, Y = {retrievedMeasCfg.RobotPos[i].Y}");
                Console.WriteLine($"Scan Location: {(ScanLoc)i}, {retrievedMeasCfg.Img.GetFilename((ScanLoc)i)}");
            }

            // 3. Declare and initialize ParamTuneInterop (optional)
            // Note: If the param is not set, the C++ code will use default values.
            // You can set it if you want to customize the tuning parameters.
            // See the documents for details on how to tune the parameters.
            // ParamTuneInterop paramTune = new();
            // Set your own parameters here if needed, e.g.:
            // paramTune.SomeParameter = someValue;
            
            // bool paramSuccess = calibrator.SetParam(ref paramTune);
            // Console.WriteLine($"SetParam Result: {paramSuccess}");

            // bool paramGetSuccess = calibrator.GetParam(out ParamTuneInterop retrievedParamTune);
            // Console.WriteLine($"GetParam Result: {paramGetSuccess}");

            // 4. Set the logger system (optional)
            // If not, everything will be logged to the default console logger.
            // The PCL logger is always bound to stderr/stdout with thread id.
            // In multithreading setting, the internal log messages will not be mixed.
            // But everything from PCL will be mixed in the console with thread id.
            // MUST: The loggerName MUST BE UNIQUE for each instance of RingCalibration.
            // Otherwise, the function call will return false.
            // The filename can be any name you want, if it is the same as an existing file, 
            // you have to take care of overwriting which is set to false by default in C#.
            // The new log would be appended to the existing log file if it exists.
            // See "RingCalibration.cs" for the SetLogger function definition.
            // If no logfile is expected, DO NOT set the logger.
 
            string loggerName = @"RingCalibrationLogger";
            // SPDLOG would handle the creation of the log directory if it does not exist.
            string logPath = @"logs/ring_calibration.log";
            bool loggerSuccess = calibrator.SetLogger(loggerName, logPath, true);
            Console.WriteLine($"SetLogger Result: {loggerSuccess}");
            
            bool loggerGetSuccess = calibrator.GetLogger(out string retrievedLoggerName, out string retrievedLogPath);
            Console.WriteLine($"GetLogger Result: {loggerGetSuccess}");
            Console.WriteLine($"Retrieved Logger Name: {retrievedLoggerName}, Retrieved Log Path: {retrievedLogPath}");
    
            // 4. Now run your calibration routine
            if (sysSuccess && measSuccess)
            {
                // Set the filenames for each scan location
                for (int i = 0; i < (int)ScanLoc.NUM; i++)
                {
                    ScanLoc loc = (ScanLoc)i;
                    // Create the filenames for the calibration
                    string path = "test_output";

                    if (!calibrator.Calibration(loc, path))
                    {
                        throw new ArgumentException("Calibration failed for location: " + loc.ToString());
                    }
                    // Verification step is optional. It measures the ring again with the calibration.
                    // The results are written in the log file, but it does not affect the calibration process.
                    // To check them, you have to turn on SPDLOG in the C++ code at least at INFO level.
                    if (!calibrator.Verification(loc, path))
                    {
                        throw new ArgumentException("Verification failed for location: " + loc.ToString());
                    }
                }
            }

            // 5. Optionally, you can retrieve the system correction arrays after calibration
            ScanCfgInterop[] correctionArray = new ScanCfgInterop[(int)ScanLoc.NUM];
            if (calibrator.GetSystemCorrection(correctionArray, correctionArray.Length))
            {
                Console.WriteLine("System correction arrays retrieved successfully.");
                for (int i = 0; i < correctionArray.Length; i++)    
                {
                    Console.WriteLine(
                        $"Scan Location: {(ScanLoc)i}, " +
                        $"ViewAngle: {correctionArray[i].ViewAngle}, " +
                        $"AngleCorr: {correctionArray[i].AngleCorr}, " +
                        $"PosCorr: ({correctionArray[i].PosCorr.X}, {correctionArray[i].PosCorr.Y})"
                    );
                }
            }
            else
            {
                Console.WriteLine("Failed to retrieve system correction arrays.");
            }

        }
    }
}