#ifndef HUSSAR_CORE_TEV_H
#define HUSSAR_CORE_TEV_H

#include <hussar/io/exr.h>
#include <hussar/core/image.h>

#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <signal.h>

#include <string>
#include <vector>
#include <map>

namespace hussar {

class TevStream : public Channelizer {
public:
    TevStream(const std::string &name) : m_name(name) {}

    template<typename T>
    void add(Image<T> &target, ChannelName channel = ChannelName()) {
        m_width = target.width();
        m_height = target.height();
        m_stride = sizeof(T);

        Channelizer::transfer(channel, target.at(0, 0));
    }

    void stream();

protected:
    virtual void directedTransfer(ChannelName channel, float &el);

private:
    class Stream {
    public:
        static int sockfd;

        struct flush {};

        template<typename T>
        struct binary {
            const T *data;
            size_t len;

            binary(const T *d, size_t l) : data(d), len(l) {}
        };

        Stream();

        template<typename T>
        Stream &operator<<(const T &el) {
            if (m_buffer.size() < m_index + sizeof(T)) {
                m_buffer.resize(m_index + sizeof(T));
            }

            *(T *)&m_buffer[m_index] = el;
            m_index += sizeof(T);

            return *this;
        }

        template<typename T>
        Stream &operator<<(const std::vector<T> &el) {
            for (const auto &e : el) {
                *this << e;
            }

            return *this;
        }

        template<typename T>
        Stream &operator<<(binary<T> b) {
            const uint32_t len = b.len * sizeof(T);
            m_buffer.resize(m_index + len);
            memcpy(m_buffer.data() + m_index, b.data, len);
            m_index += len;

            return *this;
        }

        Stream &operator<<(const std::string &el);
        Stream &operator<<(flush);
        bool fresh();

    private:
        void performFlush();
        void reset();

        std::vector<uint8_t> m_buffer;
        uint32_t m_index = 0;
        bool m_fresh = false;
    };

    Stream m_stream;
    std::string m_name;
    std::map<std::string, std::vector<float>> m_data;
    int m_width, m_height, m_stride;
};

}

#endif
