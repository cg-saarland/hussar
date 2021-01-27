#include <stdio.h>

#include "gtest/gtest.h"

#include <hussar/hussar.h>
#include <hussar/io/exr.h>

namespace hussar {

// Loads a file created with Python's OpenEXR
// N.B. fails if tests are run from the wrong directory!
#define FILENAME_LOAD_FLOAT "file_load_float.exr"
TEST(ExrSupportTest, load_float) {
    Image<Float> img(2, 2);
    ExrLoadFile exr(FILENAME_LOAD_FLOAT);
    exr.load(img, ExrFile::ChannelName("a"));
    EXPECT_EQ(img.at(0, 0), 0.1f);
    EXPECT_EQ(img.at(1, 0), 0.2f);
    EXPECT_EQ(img.at(0, 1), 0.4f);
    EXPECT_EQ(img.at(1, 1), 0.8f);
}

#define FILENAME_SAVE_FLOAT "file_save_float.exr"
TEST(ExrSupportTest, save_float) {
    std::remove(FILENAME_SAVE_FLOAT);
    Image<Float> img(2, 2);
    img.at(0, 0) = 0.1f;
    img.at(1, 0) = 0.2f;
    img.at(0, 1) = 0.4f;
    img.at(1, 1) = 0.8f;
    ExrSaveFile exr(FILENAME_SAVE_FLOAT);
    exr.add(img, ExrFile::ChannelName("a"));
    exr.save();
    // Verify with assumed correct load()
    Image<Float> imgload(2, 2);
    ExrLoadFile exrload(FILENAME_SAVE_FLOAT);
    exrload.load(imgload, ExrFile::ChannelName("a"));
    EXPECT_EQ(imgload.at(0, 0), 0.1f);
    EXPECT_EQ(imgload.at(1, 0), 0.2f);
    EXPECT_EQ(imgload.at(0, 1), 0.4f);
    EXPECT_EQ(imgload.at(1, 1), 0.8f);
}

#define FILENAME_SAVE_DESTRUCTOR "file_save_destructor.exr"
TEST(ExrSupportTest, save_destructor) {
    std::remove(FILENAME_SAVE_DESTRUCTOR);
    {
        Image<Float> img(2, 2);
        img.at(0, 0) = 0.1f;
        img.at(1, 0) = 0.2f;
        img.at(0, 1) = 0.4f;
        img.at(1, 1) = 0.8f;
        ExrSaveFile exr(FILENAME_SAVE_DESTRUCTOR);
        exr.add(img, ExrFile::ChannelName("a"));
    }
    // Verify with assumed correct load()
    Image<Float> imgload(2, 2);
    ExrLoadFile exrload(FILENAME_SAVE_DESTRUCTOR);
    exrload.load(imgload, ExrFile::ChannelName("a"));
    EXPECT_EQ(imgload.at(0, 0), 0.1f);
    EXPECT_EQ(imgload.at(1, 0), 0.2f);
    EXPECT_EQ(imgload.at(0, 1), 0.4f);
    EXPECT_EQ(imgload.at(1, 1), 0.8f);
}

}
