#include <hussar/io/tev.h>

using namespace hussar;
int TevStream::Stream::sockfd = 0;

TevStream::Stream::Stream() {
    reset();

    if (sockfd) {
        // reuse existing socket
        return;
    }

    struct hostent *server = gethostbyname("127.0.0.1");

    struct sockaddr_in serv_addr;

    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    memcpy(server->h_addr_list, &serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(14158);

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1) {
        std::cout << "could not create socket for tev" << std::endl;
        sockfd = 0;
    } else if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr))) {
        //std::cout << "connnection to tev failed" << std::endl;
        //std::cout << strerror(errno) << std::endl;
        
        close(sockfd);
        sockfd = 0;
    } else {
        m_fresh = true;
    }

    signal(SIGPIPE, SIG_IGN); // allow write to fail
}

TevStream::Stream &TevStream::Stream::operator<<(const std::string &el) {
    const char *data = el.c_str();
    const uint32_t len = strlen(data) + 1;
    *this << binary(el.c_str(), len);
    
    return *this;
}

TevStream::Stream &TevStream::Stream::operator<<(flush) {
    performFlush();
    return *this;
}

bool TevStream::Stream::fresh() {
    bool wasFresh = m_fresh;
    m_fresh = false;
    return wasFresh;
}

void TevStream::Stream::performFlush() {
    *(uint32_t *)&m_buffer[0] = m_index; // write size
    //std::cout << "len" << m_index << std::endl;
    //std::cout << std::string((char *)m_buffer.data(), m_index) << std::endl;

    if (sockfd && write(sockfd, m_buffer.data(), m_index) < m_index) {
        std::cout << "connection to tev lost" << std::endl;

        // we were connected, but now we are't anymore
        close(sockfd);
        sockfd = 0;
    }
    
    reset();
}

void TevStream::Stream::reset() {
    m_buffer.clear();
    m_index = 4;
}

void TevStream::stream() {
    std::vector<std::string> channelNames;
    for (const auto &kv : m_data) {
        channelNames.push_back(kv.first);
    }

    if (m_stream.fresh()) {
        m_stream
            // close existing image
            << char(2)     // type
            << m_name      // filename
            << Stream::flush()

            // create image
            << char(4)     // type
            << bool(false) // grab focus
            << m_name      // filename
            << int32_t(m_width) << int32_t(m_height)
            << int32_t(channelNames.size())
            << channelNames
            << Stream::flush()
        ;
    }

    for (const auto &kv : m_data) {
        int y_step = 128; /// @todo search for an elegant solution!

        for (int y = 0; y < m_height; y += y_step) {
            int height = std::min(y+y_step, m_height) - y;

            m_stream
                // update channel
                << char(3)
                << bool(false)
                << m_name
                << kv.first
                << int32_t(0) << int32_t(y)
                << int32_t(m_width) << int32_t(height)
                << Stream::binary(&kv.second.data()[m_width * y], m_width * height)
                << Stream::flush()
            ;
        }
    }
}

void TevStream::directedTransfer(ChannelName channel, float &el) {
    size_t count = static_cast<size_t>(m_width * m_height);
    std::vector<float> &data = m_data[channel.name];
    data.resize(count);

    uint8_t *imageData = reinterpret_cast<uint8_t*>(&el);
    for (size_t i = 0; i < count; i++)
        data[i] = static_cast<float>(*(reinterpret_cast<float *>(imageData + i * m_stride)));
}
