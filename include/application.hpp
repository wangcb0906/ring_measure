#include <argparse/argparse.hpp>
#include <spdlog/spdlog.h>

inline void calibration_parser(
    argparse::ArgumentParser& program,
    int argc,
    char* argv[])
{

    program.add_argument("cali")
           .help("Folder to calibrate a folder or a single measure configuration file.");

    program.add_argument("-m", "--mode")
           .default_value(3)
           .nargs(1)
           .scan<'i', int>()
           .choices(0, 1, 2, 3)
           .help("Mode of operation.\n"
                 "Lower bit for calibration, higher bit for verification.\n"
                 "0: sanity check only, 1: calibration only,\n"
                 "2: verification only, 3: both calibration and verification.");

    program.add_argument("-b", "--binary")
           .help("If the binary correction, use it to apply the correction.\n"
                 "The defaut value is false.\n")
           .default_value(false)
           .nargs('?')
           .implicit_value(true);

    program.add_argument("-p", "--parameter")
           .help("Path to the parameter file. "
                 "If not specified, the default parameters will be used,\n"
                 "the out_path will be set to 'result'.\n"
                 "If specified, the parameters will be loaded from the file,\n"
                 "and the out_path will be set to folder of the same name.\n"
                 );

    program.add_argument("-o", "--output")
           .help("Path to the output directory.\n"
                 "If not specified, the output will be set to 'result' in the root folder.\n"
                 "or in accordance with the value of 'parameter'.\n"
                 "If specified, the output will be overwritten.\n"
                 );

    program.add_argument("-r", "--ring-cfg")
       .help("Specific ring to calibrate, otherwise,\n"
             "all rings in the folder will be processed.\n"
             "If -r specified, the parameters will be dumped to the log file.\n"
             "Otherwise, the parameters will be dumped to  file in the root folder.\n");

    program.add_argument("-s", "--suffix")
           .default_value(std::string(""))
           .help("If -r specified, add the suffix to the log_file name.\n"
                 "Otherwise, no effect.\n"
                 "Usually, used to distinguish different runs on the same ring.")
           .nargs(1);

    try {
        program.parse_args(argc, argv);
    } catch (const std::exception& err) {
        SPDLOG_LOGGER_ERROR(spdlog::default_logger(), "Error parsing arguments: {}", err.what());
        throw std::runtime_error(err.what());
    }
}


void evaluation_parser(
    argparse::ArgumentParser& program,
    int argc,
    char* argv[])
{
    program.add_argument("input")
           .help("Path to the calibrations.\n");

    program.add_argument("output")
           .help("Path to the output directory.\n");

    program.add_argument("-e", "--evaluation")
           .help(
                 "Path to the rings to evaluate.\n"
                 "The defaut value is set to $input.\n"
                 )
           .nargs(1);

    program.add_argument("-b", "--binary")
           .help("If it is specified, use binary format to apply the correction.\n"
                 "The defaut value is false.\n"
                 )
           .default_value(false)
           .implicit_value(true);

    program.add_argument("-p", "--parameter")
           .help("Path to the parameter file.\n"
                 "If not specified, use the parameters of the same name of the calibration.\n"
                 "If specified, the parameters will be loaded from the file.\n"
                 "The parameters will be dumped to log.\n"
                 );

    program.add_argument("-t", "--test-ring")
           .help(
                 "If specified, only the test ring will be processed.\n"
                 "Otherwise, all rings in the folder will be processed.\n"
                 "The test ring should be specified as a file name, e.g., 'ring_1.ini'.\n"
                 "The test ring should be located in the 'config' folder.\n"
                );

    program.add_argument("-c", "--calibration")
            .help("Specific calibration to evaluate, otherwise,\n"
                  "all calibrations in the folder will be processed.\n");

    program.add_argument("-m", "--minus-wedge")
           .help("If it is specified, the height of the table will be subtracted by the height of the wedge.\n"
                 "Set it true for calibration and false for measurement.\n"
                 "The defaut value is false.\n"
                 )
           .default_value(false)
           .implicit_value(true);

    program.add_argument("-s", "--suffix")
           .help("Add the suffix to the log_file and output file names.\n"
                 "Otherwise, no effect.\n"
                 "Default value is empty string.")
           .nargs(1);

    try {
        program.parse_args(argc, argv);
    } catch (const std::exception& err)
    {
        SPDLOG_LOGGER_ERROR(spdlog::default_logger(), "Error parsing arguments: {}", err.what());
        auto aux = std::ostringstream{};
        aux << err.what();
        SPDLOG_LOGGER_ERROR(spdlog::default_logger(), "{}", aux.str());
        throw std::runtime_error(aux.str());
    }
}

std::vector<std::filesystem::path> get_ini_files(
    const std::filesystem::path& dir_path,
    std::optional<std::string>&  single_file) 
{
    std::vector<std::filesystem::path> ini_files;
    if(single_file) {
        std::filesystem::path single_path = single_file.value();
        ini_files.emplace_back(dir_path / single_path.filename());
    } else {
        if (std::filesystem::exists(dir_path) && std::filesystem::is_directory(dir_path)) {
            for (const auto& entry : std::filesystem::directory_iterator(dir_path)) {
                if (entry.path().extension() == ".ini") {
                    ini_files.push_back(entry.path());
                }
            }
            std::sort(ini_files.begin(), ini_files.end());
        } else {
            SPDLOG_LOGGER_ERROR(spdlog::default_logger(), "Directory {} does not exist.", dir_path);
            throw std::runtime_error("Directory does not exist.");
        }
    }
    return ini_files;
}