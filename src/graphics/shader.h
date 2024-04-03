#pragma once

#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "GL/glew.h"
#include "Eigen/Dense"
#include <util/unsupportedeigenthing/OpenGLSupport>


class Shader {
public:
    Shader(const std::string &vertexPath, const std::string &fragmentPath);
    Shader(const std::string &vertexPath, const std::string &geometryPath, const std::string &fragmentPath);

    Shader(Shader &that) = delete;
    Shader& operator=(Shader &that) = delete;
    virtual ~Shader();
    Shader(Shader &&that);
    Shader& operator=(Shader &&that);

    GLuint getUniformLocation(std::string name);
    GLuint getEnumeratedUniformLocation(std::string name, int index);

    template<typename type, int n, int m>
    void setUniform(const std::string &name, const Eigen::Matrix<type, n, m> &mat) {
        glUniform(m_uniforms[name], mat);
    }

    void setUniform(const std::string &name, float f);
    void setUniform(const std::string &name, int i);
    void setUniform(const std::string &name, bool b);

    void setUniformArrayByIndex(const std::string &name, float f, size_t index);
    void setUniformArrayByIndex(const std::string &name, const Eigen::Vector2f &vec2, size_t index);
    void setUniformArrayByIndex(const std::string &name, const Eigen::Vector3f &vec3, size_t index);
    void setUniformArrayByIndex(const std::string &name, const Eigen::Vector4f &vec4, size_t index);
    void setUniformArrayByIndex(const std::string &name, int i, size_t index);
    void setUniformArrayByIndex(const std::string &name, const Eigen::Vector2i &ivec2, size_t index);
    void setUniformArrayByIndex(const std::string &name, const Eigen::Vector3i &ivec3, size_t index);
    void setUniformArrayByIndex(const std::string &name, const Eigen::Vector4i &ivec4, size_t index);
    void setUniformArrayByIndex(const std::string &name, bool b, size_t index);
    void setUniformArrayByIndex(const std::string &name, const Eigen::Matrix2f &mat2, size_t index);
    void setUniformArrayByIndex(const std::string &name, const Eigen::Matrix3f &mat3, size_t index);
    void setUniformArrayByIndex(const std::string &name, const Eigen::Matrix4f &mat4, size_t index);


    void bind() const;
    void unbind() const;
    GLuint id() const { return m_programID; }

    bool printDebug();
    void resetDebug();

private:

    std::string getFileContents(std::string path);

    GLuint createFragmentShaderFromSource(const std::string &source);
    GLuint createGeometryShaderFromSource(const std::string &source);
    void compileShader(GLuint handle, const std::string &source);
    GLuint createVertexShaderFromSource(const std::string &source);
    GLuint createShaderFromSource(const std::string &source, GLenum shaderType);

    void createProgramID();
    void attachShaders(const std::vector<GLuint> &shaders);
    void buildShaderProgramFromShaders(const std::vector<GLuint> &shaders);
    void linkShaderProgram();
    void detachShaders(const std::vector<GLuint> &shaders);
    void deleteShaders(const std::vector<GLuint> &shaders);

    void discoverShaderData();
    void discoverAttributes();
    void discoverUniforms();

    bool isUniformArray(const GLchar *name , GLsizei nameLength);
    bool isTexture(GLenum type);
    void addUniform(const std::string &name);
    void addUniformArray(const std::string &name, size_t size);
    void addTexture(const std::string &name);
    GLuint m_programID;

    std::map<std::string, GLuint> m_attributes;
    std::map<std::string, GLuint> m_uniforms;
    std::map<std::tuple<std::string, size_t>, GLuint> m_uniformArrays;
    std::map<std::string, GLuint> m_textureLocations; // name to uniform location
    std::map<GLuint, GLuint> m_textureSlots; // uniform location to texture slot

    // Debugging
    std::map<std::string, bool> m_trackedUniforms;
    std::map<std::string, bool> m_trackedTextures;
    std::map<std::string, bool> m_trackedUniformArrays;

    friend class Graphics;
};
