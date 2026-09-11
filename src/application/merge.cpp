#include <Magick++.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <filesystem>
#include <omp.h>

int main(int argc, char** argv) {
    // 1. Initialize Magick++
    Magick::InitializeMagick(*argv);

    namespace fs = std::filesystem;
    fs::path in1 = argv[1];
    fs::path in2 = argv[2];
    fs::path out = argv[3];
    int offset = std::stoi(argv[4]);

    size_t rads = 360;

    if(in1.string().find("In") == std::string::npos) {
        std::swap(in1, in2);
    }

    fs::create_directories(out);

    #pragma omp parallel for schedule(dynamic)
    for(size_t i = 0; i < rads; ++i) {
        std::ostringstream oss1;
        oss1 << std::setw(3) << std::setfill('0') << (i + offset)%360;
        std::ostringstream oss2;
        oss2 << std::setw(3) << std::setfill('0') << i;

        fs::path in_image = in1/("line_" + oss1.str() + "_In_recon.png");
        fs::path out_image = in2/("line_" + oss2.str() + "_Out_recon.png");

        if(fs::exists(in_image) && fs::exists(out_image)) {
            Magick::Image baseImage(out_image.string());
            Magick::Image blendImage(in_image.string());
    
            blendImage.alpha(true);
            blendImage.evaluate(Magick::AlphaChannel, Magick::MultiplyEvaluateOperator, 0.5);
    
            // Composite the images
            baseImage.composite(blendImage, 0, 0, Magick::DissolveCompositeOp);
    
            // Save the result with a unique filename
            std::string output_filename = out.string() + "/combined_in_out_" + oss1.str() + ".png";
            baseImage.write(output_filename);
        }
    }

    return 0;
}