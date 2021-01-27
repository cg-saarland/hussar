#include <nanogui/opengl.h>
#include <nanogui/glutil.h>
#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/button.h>
#include <nanogui/entypo.h>
#include <nanogui/textbox.h>
#include <nanogui/slider.h>

#include <iostream>
#include <iomanip>
#include <sstream>

#include "hussar.h"
#include "guiding/btree.h"

#include "visualizer/imageview.h"

using namespace nanogui;
using namespace hussar;

using BT2F = BTreeGuiding<2, GuidingLeaf<Float>>;

class GLTexture {
public:
    int len;

    GLTexture() = default;
    GLTexture(const std::string &textureName)
        : mTextureName(textureName), mTextureId(0) {
    }

    GLTexture(GLTexture&& other)
        : mTextureName(move(other.mTextureName)),
        mTextureId(other.mTextureId) {
        other.mTextureId = 0;
    }

    GLTexture& operator=(GLTexture &&other) {
        mTextureName = move(other.mTextureName);
        std::swap(mTextureId, other.mTextureId);
        return *this;
    }

    ~GLTexture() {
        if (mTextureId)
            glDeleteTextures(1, &mTextureId);
    }

    GLuint texture(const BT2F &btree) {
        load(btree);
        return mTextureId;
    }

    const std::string &textureName() const {
        return mTextureName;
    }

protected:
    void loadData(const BT2F &btree, std::vector<Float> &data) {
        len = 1 << btree.depth();
        data.resize(len * len);

        for (int x = 0; x < len; ++x) {
            for (int y = 0; y < len; ++y) {
                data[x * len + y] = btree.pdf(Vector2f(
                    (x + 0.5f) / len,
                    (y + 0.5f) / len
                ));
            }
        }
    }

    void load(const BT2F &btree) {
        if (mTextureId) {
            glDeleteTextures(1, &mTextureId);
            mTextureId = 0;
        }

        std::vector<Float> data;
        loadData(btree, data);

        glGenTextures(1, &mTextureId);
        glBindTexture(GL_TEXTURE_2D, mTextureId);

        glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, len, len, 0, GL_RED, GL_FLOAT, &data[0]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    }

    std::string mTextureName;
    GLuint mTextureId;
};

class App : public Screen {
public:
    BT2F btree;
    GLTexture tex;
    ImageViewFC *mImageView;

    TextBox *mExposureTextbox;
    Slider *mExposureSlider;

    App() : Screen(Vector2i(1024, 768), "Visualizer") {
        glfwMaximizeWindow(mGLFWWindow);

        Window *window = new Window(this, "Controls");
        window->setPosition(Vector2i(15, 15));
        window->setLayout(new GroupLayout());

        //

        new Label(window, "Exposure", "sans-bold");
        Widget *panel = new Widget(window);
        panel->setLayout(new BoxLayout(Orientation::Horizontal,
            Alignment::Middle, 0, 20));

        mExposureSlider = new Slider(panel);
        mExposureSlider->setRange({-15.0f, 0.0f});
        mExposureSlider->setFixedWidth(120);

        mExposureSlider->setCallback([this](float value) {
            setExposure(value);
        });

        mExposureTextbox = new TextBox(panel);
        mExposureTextbox->setFixedSize(Vector2i(60, 25));
        mExposureTextbox->setFixedSize(Vector2i(60, 25));
        mExposureTextbox->setFontSize(20);
        mExposureTextbox->setAlignment(TextBox::Alignment::Right);

        //

        std::ifstream test("/Users/alex/Desktop/serialized", std::ios::binary);
        btree.deserialize(test);
        btree.dump();

        auto mImageContainer = new Window(this, "Samplingpoint dir");
        mImageContainer->setPosition(Vector2i(mSize.x() - 256, 0));
        mImageContainer->setLayout(new BoxLayout(Orientation::Vertical));
        mImageContainer->setFixedWidth(256);

        mImageView = new ImageViewFC(mImageContainer, tex.texture(btree));
        mImageView->setFixedSize(Vector2i(256, 256));
        mImageView->setScale(256 / tex.len);

        //

        setExposure(-9);
        performLayout();
    }

    float exposure() const {
        return mExposureSlider->value();
    }

    void setExposure(float exposure) {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(2) << exposure;
        mExposureTextbox->setValue(stream.str());

        mExposureSlider->setValue(exposure);
        mExposureTextbox->setValue(stream.str());
        mImageView->setExposure(exposure);
    }
};

int main(int argc, char *argv[]) {
    init();

    App app;
    app.setBackground(Color(0.0f, 0.0f, 0.0f, 1.0f));
    app.drawAll();
    app.setVisible(true);

    mainloop();
}
