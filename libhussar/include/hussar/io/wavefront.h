#ifndef HUSSAR_IO_WAVEFRONT_H
#define HUSSAR_IO_WAVEFRONT_H

#include <fstream>

#include <hussar/hussar.h>
#include <hussar/core/mesh.h>
#include <hussar/core/logging.h>

namespace hussar {

class WavefrontFile {
public:
    WavefrontFile(const std::string &path)
    : m_file(path) {}

    void read(TriangleMesh &mesh) {
        if (!m_file) {
            Log(EError, "invalid file passed to WavefrontFile");
            return;
        }

        int offset = mesh.vertexBuffer.size();
        while (!m_file.eof()) {
            readLine(mesh, offset);
        }
    }

private:
    void readLine(TriangleMesh &mesh, int offset) {
        std::string line;
        if (m_file.peek() == '#') {
            // read comment
            std::getline(m_file, line);
        } else {
            std::string cmd;
            m_file >> cmd;

            if (cmd == "v") {
                Vector3f vertex;
                for (int i = 0; i < 3; ++i) {
                    m_file >> vertex[i];
                }

                mesh.vertexBuffer.push_back(vertex);
            } else if (cmd == "f") {
                TriangleMesh::IndexTriplet indices;
                for (int i = 0; i < 3; ++i) {
                    m_file >> indices.raw[i];
                    indices.raw[i] += offset - 1;
                }

                mesh.indexBuffer.push_back(indices);
            } else if (cmd == "s") {
                // we do not care about smoothing.
            } else if (cmd == "") {
                // empty line
            } else {
                std::cout << "unsupported wavefront command: " << cmd << std::endl;
            }

            std::getline(m_file, line); // read until end of line
        }
    }

    std::ifstream m_file;
};

}

#endif
