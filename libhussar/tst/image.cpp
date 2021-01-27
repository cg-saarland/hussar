#include "gtest/gtest.h"

#include <hussar/hussar.h>
#include <hussar/core/image.h>

namespace hussar {

class ImageTest : public ::testing::Test {
public:
    ImageTest() : img(2, 2) {}

protected:
    void SetUp() override {
        img.at(0, 0) = 0.1f;
        img.at(0, 1) = 0.2f;
        img.at(1, 0) = 0.4f;
        img.at(1, 1) = 0.8f;
    }

    void TearDown() override {

    }

    Image<Float> img;
};

TEST_F(ImageTest, at) {
    EXPECT_EQ(img.at(0, 0), 0.1f);
    EXPECT_EQ(img.at(0, 1), 0.2f);
    EXPECT_EQ(img.at(1, 0), 0.4f);
    EXPECT_EQ(img.at(1, 1), 0.8f);
}

TEST_F(ImageTest, splat) {
    img.splat(Vector2f(0.1f, 0.1f), 0.5f);
    EXPECT_EQ(img.at(0, 0), 0.6f);
    EXPECT_EQ(img.at(0, 1), 0.2f);
    EXPECT_EQ(img.at(1, 0), 0.4f);
    EXPECT_EQ(img.at(1, 1), 0.8f);
}

TEST_F(ImageTest, each) {
    img.each([](Float& e) {
        e = 2 * e;
    });
    EXPECT_EQ(img.at(0, 0), 0.2f);
    EXPECT_EQ(img.at(0, 1), 0.4f);
    EXPECT_EQ(img.at(1, 0), 0.8f);
    EXPECT_EQ(img.at(1, 1), 1.6f);
}

TEST_F(ImageTest, width) {
    EXPECT_EQ(img.width(), 2);
}

TEST_F(ImageTest, height) {
    EXPECT_EQ(img.height(), 2);
}

TEST_F(ImageTest, clear) {
    img.clear(0.f);
    EXPECT_EQ(img.at(0, 0), 0.f);
    EXPECT_EQ(img.at(0, 1), 0.f);
    EXPECT_EQ(img.at(1, 0), 0.f);
    EXPECT_EQ(img.at(1, 1), 0.f);
}

}
