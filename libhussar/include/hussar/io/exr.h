#ifndef HUSSAR_IO_EXR_H
#define HUSSAR_IO_EXR_H

#include <functional>
#include <exception>
#include <string>
#include <map>

#include <tinyexr.h>

#include <hussar/core/image.h>

// Cleanup utility
#define LINE_ID_COMBINE2(x, y) x ## y
#define LINE_ID_COMBINE(x, y) LINE_ID_COMBINE2(x, y) // necessary
#define ON_SCOPE_EXIT(x) \
std::shared_ptr<void*> LINE_ID_COMBINE(cleanup, __LINE__) (nullptr, [&] (void*) { \
    x; \
});

namespace hussar {

class Channelizer {
public:
    struct ChannelName {
        std::string name;

        ChannelName() : name("") {}
        ChannelName(const std::string& name) : name(name) {}

        ChannelName operator()(const std::string& suffix) {
            return ChannelName(name.empty() ? suffix : name + "." + suffix);
        }
    };

    template<typename T>
    void transfer(ChannelName channel, T &el);

    // Defines transfer for deriving class's direction
    virtual void directedTransfer(ChannelName channel, float& el) = 0;
};

class ExrFile : public Channelizer {
public:
    struct exr_file_exception : public std::exception {
    public:
        explicit exr_file_exception(const std::string& message) : message(message) {};
        const char* what() const throw() {
            return message.c_str();
        }
    private:
        std::string message;
    };

    struct exr_invalid_op_exception : public std::exception {
    public:
        explicit exr_invalid_op_exception(const std::string& message) : message(message) {};
        const char* what() const throw() {
            return message.c_str();
        }
    private:
        std::string message;
    };

protected:
    ExrFile(const std::string& filename) : m_filename(filename) {}

    std::string m_filename  {};
    int m_width             {};
    int m_height            {};
    size_t m_stride         {};
    EXRImage m_image        {};
    EXRHeader m_header      {};
};

class ExrLoadFile : public ExrFile {
public:
    ExrLoadFile(const std::string& filename) : ExrFile(filename) {
        int err;
        const char* exror = nullptr;
        // Note: handles call with nullptr
        ON_SCOPE_EXIT(FreeEXRErrorMessage(exror));

        // Verify that the file exists and is valid
        err = IsEXR(m_filename.c_str());
        if (err != TINYEXR_SUCCESS) {
            throw exr_file_exception(m_filename + std::string(" is not an EXR file ")
                + std::string("(err = ") + std::to_string(err) + std::string(")"));
        }

        EXRVersion version;
        err = ParseEXRVersionFromFile(&version, m_filename.c_str());
        if (err != TINYEXR_SUCCESS) {
            throw exr_file_exception(std::string("Failed to read EXR version from ")
                + m_filename + std::string(" (err = ") + std::to_string(err)
                + std::string(")"));
        }

        InitEXRHeader(&m_header);
        err = ParseEXRHeaderFromFile(&m_header, &version, m_filename.c_str(), &exror);
        if (err != TINYEXR_SUCCESS) {
            throw exr_file_exception(std::string("Failed to read EXR header from ")
                + m_filename + std::string(" err = ") + std::to_string(err)
                + std::string(") ") + std::string(exror));
        }

        // Read the EXR file
        InitEXRImage(&m_image);
        err = LoadEXRImageFromFile(&m_image, &m_header, m_filename.c_str(), &exror);
        if (err != TINYEXR_SUCCESS) {
            throw exr_file_exception(std::string("Failed to load EXR image from ")
                + m_filename + std::string(" (err = ") + std::to_string(err)
                + std::string(") ") + std::string(exror));
        }
    }

    virtual ~ExrLoadFile() {
        FreeEXRHeader(&m_header);
        FreeEXRImage(&m_image);
    }

    template<typename T>
    void load(Image<T>& target, ChannelName channel = ChannelName()) {
        const char* exror = nullptr;
        // Note: handles call with nullptr
        ON_SCOPE_EXIT(FreeEXRErrorMessage(exror));

        // Verify the target has correct dimensions
        int xSize = m_header.data_window.max_x - m_header.data_window.min_x + 1;
        int ySize = m_header.data_window.max_y - m_header.data_window.min_y + 1;
        if (target.width() != xSize) {
            throw std::invalid_argument(std::string("Invalid target width: ")
                + std::to_string(target.width()) + std::string(" vs ")
                + std::to_string(xSize));
        }
        if (target.height() != ySize) {
            throw std::invalid_argument(std::string("Invalid target height ")
                + std::to_string(target.height()) + std::string(" vs ")
                + std::to_string(ySize));
        }

        // Verify the requested base channel name exists
        bool chanFound = false;
        for (int chan = 0; chan < m_header.num_channels; chan++) {
            EXRChannelInfo chanInfo = m_header.channels[chan];
            std::string fullChanName(chanInfo.name);
            // Compare only base part of channel name (up to first '.')
            size_t dotPos = fullChanName.find_first_of('.');
            if (fullChanName.compare(0, dotPos, channel.name) == 0) {
                chanFound = true;
                break;
            }
        }
        if (chanFound == false) {
            throw std::invalid_argument(std::string("No base channel name ")
                + channel.name + std::string(" in ") + m_filename);
        }

        // Set the object state for the transfer
        // Note: width and height have been verified to match file
        m_width = target.width();
        m_height = target.height();
        m_stride = sizeof(T);

        ExrFile::transfer(channel, target.at(0, 0));

        target.rebuild();
    }

protected:
    virtual void directedTransfer(ChannelName channel, float& el) {
        size_t count = static_cast<size_t>(m_width * m_height);

        // Find the index of the requested channel name
        int chanIndex = -1;
        for (int chan = 0; chan < m_header.num_channels; chan++) {
            EXRChannelInfo chanInfo = m_header.channels[chan];
            std::string fullChanName(chanInfo.name);
            if (fullChanName.compare(channel.name) == 0) {
                chanIndex = chan;
                break;
            }
        }
        if (chanIndex == -1) {
            throw std::invalid_argument(std::string("Channel ") + channel.name
                + std::string(" not found in ") + m_filename);
        }
        float* fileData = reinterpret_cast<float*>(m_image.images[chanIndex]);

        // Copy the data
        /// @todo support alternative pixel types
        /// verify: half is likely already supported with internal TinyEXR conversion
        uint8_t* imageData = reinterpret_cast<uint8_t*>(&el);
        for (size_t i = 0; i < count; i++)
            *(reinterpret_cast<float *>(imageData + i * m_stride)) = static_cast<float>(*(fileData + i));
    }

private:

};

class ExrSaveFile : public ExrFile {
public:
    ExrSaveFile(const std::string& filename) : ExrFile(filename) {}

    virtual ~ExrSaveFile() {
        // Flush any added channels to the file
        if (m_dirty) {
            try {
                save();
            } catch(...) {
                // Ignore
            }
        }

        // Cleanup allocations not handed over for FreeEXRImage cleanup
        for (auto it = m_writechannels.begin(); it != m_writechannels.end(); ++it) {
            if (it->second != nullptr) {
                free(it->second);
            }
        }
    }

    template<typename T>
    void add(Image<T>& target, ChannelName channel = ChannelName()) {
        // Catch invalid target size
        if (m_writechannels.size() > 0) {
            if (m_width * m_height != target.width() * target.height()) {
                throw exr_invalid_op_exception("Cannot mix channel sizes in EXR file");
            }
        } else {
            m_width = target.width();
            m_height = target.height();
        }

        // Set the object state for the transfer
        m_width = target.width();
        m_height = target.height();
        m_stride = sizeof(T);

        ExrFile::transfer(channel, target.at(0, 0));

        // Need to save
        m_dirty = true;
    }

    void save() {
        int err;
        const char* exror = nullptr;
        // Note: handles call nullptr
        ON_SCOPE_EXIT(FreeEXRErrorMessage(exror));

        InitEXRHeader(&m_header);
        ON_SCOPE_EXIT(FreeEXRHeader(&m_header));

        InitEXRImage(&m_image);
        ON_SCOPE_EXIT(FreeEXRImage(&m_image));

        // Allocate the header channels
        m_header.num_channels = static_cast<int>(m_writechannels.size());
        // Note: freed by FreeEXRHeader
        m_header.channels = (EXRChannelInfo*)malloc(sizeof(EXRChannelInfo) * m_header.num_channels);
        if (m_header.channels == nullptr) {
            throw std::bad_alloc();
        }

        // Set the channel names and image data
        m_image.num_channels = m_header.num_channels;
        m_image.width = m_width;
        m_image.height = m_height;
        m_image.images = (uint8_t**)malloc(sizeof(uint8_t*) * m_header.num_channels);
        if (m_image.images == nullptr) {
            throw std::bad_alloc();
        }
        int chanIndex = 0;
        for (auto it = m_writechannels.begin(); it != m_writechannels.end(); ++it) {
            // Set the channel name
            strncpy(m_header.channels[chanIndex].name, it->first.c_str(), 255);
            size_t len = strlen(it->first.c_str());
            m_header.channels[chanIndex].name[len > 255 ? 255 : len] = '\0';
            // Set the image pointer
            m_image.images[chanIndex] = reinterpret_cast<uint8_t*>(it->second);
            // Note: ownership transfer successful: freed by FreeEXRImage now
            it->second = nullptr;
            chanIndex++;
        }

        // Allocate the pixel types
        // Note: freed by FreeEXRHeader
        m_header.pixel_types = (int*)malloc(sizeof(int) * m_header.num_channels);
        if (m_header.pixel_types == nullptr) {
            throw std::bad_alloc();
        }
        // Note: freed by FreeEXRHeader
        m_header.requested_pixel_types = (int*)malloc(sizeof(int) * m_header.num_channels);
        if (m_header.requested_pixel_types == nullptr) {
            throw std::bad_alloc();
        }

        // Set the pixel types
        /// @todo support alternative pixel types
        for (int i = 0; i < m_header.num_channels; i++) {
            m_header.pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;
            m_header.requested_pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;
        }

        // Write the image to the file
        err = SaveEXRImageToFile(&m_image, &m_header, m_filename.c_str(), &exror);
        if (err != TINYEXR_SUCCESS) {
            throw exr_file_exception(std::string("Failed to write EXR file ")
                + m_filename + std::string("( err = ") + std::to_string(err)
                + std::string(") ") + std::string(exror));
        }

        // Everything up to date
        m_dirty = false;
    }

protected:
    virtual void directedTransfer(ChannelName channel, float& el) {
        size_t count = static_cast<size_t>(m_width * m_height);

        // Insert the write information into the channel write map
        // Note: freed by destructor or FreeEXRImage if ownership transfer completes
        float* fileData = (float*)malloc(count * sizeof(float));
        if (fileData == nullptr) {
            throw std::bad_alloc();
        }
        m_writechannels[channel.name] = fileData;

        // Copy the data
        /// @todo support alternative pixel types
        /// verify: half is likely already supported with internal TinyEXR conversion
        uint8_t* imageData = reinterpret_cast<uint8_t*>(&el);
        for (size_t i = 0; i < count; i++)
            *(fileData + i) = static_cast<float>(*(reinterpret_cast<float *>(imageData + i * m_stride)));
    }

private:
    std::map<std::string, float*> m_writechannels;
    // Indicates unsaved channels are present
    bool m_dirty {};
};

// Declarations for specializations (implementations in cpp file)
template<> void Channelizer::transfer(ChannelName channel, float& el);
template<> void Channelizer::transfer(ChannelName channel, radar::complex<float>& el);

}

#endif
