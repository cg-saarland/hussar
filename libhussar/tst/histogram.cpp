#include "gtest/gtest.h"

#include <hussar/hussar.h>
#include <hussar/core/histogram.h>

namespace hussar {

class HistogramTest : public ::testing::Test {
public:
    HistogramTest() : hist(2, 2) {}

protected:
    void SetUp() override {
        hist.at(0, 0) = 0.1f;
        hist.at(0, 1) = 0.2f;
        hist.at(1, 0) = 0.4f;
        hist.at(1, 1) = 0.8f;

        hist.rebuild();
    }

    void TearDown() override {

    }

    Histogram<Float> hist;
};

TEST_F(HistogramTest, sample) {
    EXPECT_EQ(hist.sample(Vector2f(0.1f, 0.1f)), 0.1f);
    EXPECT_EQ(hist.sample(Vector2f(0.9f, 0.9f)), 0.8f);
    EXPECT_EQ(hist.sample(Vector2f(0.1f, 0.9f)), 0.2f);
    EXPECT_EQ(hist.sample(Vector2f(0.9f, 0.1f)), 0.4f);

    // Add a dominating factor and ensure it is sampled at all non-exact boundaries
    hist.at(0, 1) = 10.f;
    hist.rebuild();
    EXPECT_EQ(hist.sample(Vector2f(0.f, 0.f)), 0.1f);
    EXPECT_EQ(hist.sample(Vector2f(0.1f, 0.1f)), 10.f);
    EXPECT_EQ(hist.sample(Vector2f(1.f, 1.f)), 0.8f);
    EXPECT_EQ(hist.sample(Vector2f(0.9f, 0.9f)), 10.f);
    // N.B. (0, 1) was replaced with 10.f
    EXPECT_EQ(hist.sample(Vector2f(0.f, 1.f)), 10.f);
    EXPECT_EQ(hist.sample(Vector2f(0.1f, 0.9f)), 10.f);
    EXPECT_EQ(hist.sample(Vector2f(1.f, 0.f)), 0.4f);
    EXPECT_EQ(hist.sample(Vector2f(0.9f, 0.1f)), 10.f);

    // Add another dominating factor and ensure it is sampled at all non-exact boundaries
    hist.at(1, 0) = 1000.f;
    hist.rebuild();
    EXPECT_EQ(hist.sample(Vector2f(0.f, 0.f)), 0.1f);
    EXPECT_EQ(hist.sample(Vector2f(0.1f, 0.1f)), 1000.f);
    EXPECT_EQ(hist.sample(Vector2f(1.f, 1.f)), 0.8f);
    EXPECT_EQ(hist.sample(Vector2f(0.9f, 0.9f)), 1000.f);
    EXPECT_EQ(hist.sample(Vector2f(0.f, 1.f)), 10.f);
    EXPECT_EQ(hist.sample(Vector2f(0.1f, 0.9f)), 1000.f);
    // N.B. (1, 0) was replaced with 1000.f
    EXPECT_EQ(hist.sample(Vector2f(1.f, 0.f)), 1000.f);
    EXPECT_EQ(hist.sample(Vector2f(0.9f, 0.1f)), 1000.f);
}

TEST_F(HistogramTest, evaluate) {
    EXPECT_EQ(hist.evaluate(Vector2f(0.1f, 0.1f)), 0.1f);
    EXPECT_EQ(hist.evaluate(Vector2f(0.9f, 0.9f)), 0.8f);
    EXPECT_EQ(hist.evaluate(Vector2f(0.1f, 0.9f)), 0.2f);
    EXPECT_EQ(hist.evaluate(Vector2f(0.9f, 0.1f)), 0.4f);
}

}
