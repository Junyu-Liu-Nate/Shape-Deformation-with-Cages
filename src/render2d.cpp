#include "render2d.h"
#include <QFile>
#include <QTextStream>

Render2D::Render2D(QWidget *parent)
    : QOpenGLWidget(parent)
{

}

void Render2D::initializeGL()
{
    //// PRE-SETUP ////

    // Initialize the GLEW library to provide access to OpenGL functions
    glewExperimental = GL_TRUE;

    // Print success/error message
    GLenum err = glewInit();
    if (GLEW_OK != err) fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
    fprintf(stdout, "Using GLEW %s\n", glewGetString(GLEW_VERSION));

    // Set draw size
    glViewport(0, 0, size().width(), size().height());

    // Load the shader
    m_shader = createShaderProgram(":/resources/shaders/texture.vert", ":/resources/shaders/texture.frag");

    //// TEXTURE ////

    // Load the image for texture
    QImage img("texture/test.jpg");
    m_image = img.convertToFormat(QImage::Format_RGBA8888);

    // Generate texture, choose it, and bind
    glGenTextures(1, &m_texture);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_texture);

    // Load the image into texture
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_image.width(), m_image.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, m_image.bits());

    // Set interpolation mode of texture
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Unbind texture
    glBindTexture(GL_TEXTURE_2D, 0);

    // Set sampler uniform
    glUseProgram(m_shader);
    GLint textureSamplerLocation = glGetUniformLocation(m_shader, "textureSampler");
    if (textureSamplerLocation != -1) {
        glUniform1i(textureSamplerLocation, 0);
    }
    glUseProgram(0);

    //// VBO and VAO ////

    std::vector<GLfloat> data = {
        // Vertices         // UV Coordinates (flipped Y)
        -1.f,  1.f, 0.f,    0.f, 0.f,  // Top Left
        -1.f, -1.f, 0.f,    0.f, 1.f,  // Bottom Left
         1.f,  1.f, 0.f,    1.f, 0.f,  // Top Right
         1.f, -1.f, 0.f,    1.f, 1.f   // Bottom Right
    };

    // Generate, bind, and then pass data to VBO
    glGenBuffers(1, &m_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(GLfloat), data.data(), GL_STATIC_DRAW);

    // Generate, bind, and then define VAO
    glGenVertexArrays(1, &m_vao);
    glBindVertexArray(m_vao);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), reinterpret_cast<void*>(3 * sizeof(GLfloat)));

    // Unbind VBO and VAO
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void Render2D::paintGL()
{
    // Clear screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Bind shader
    glUseProgram(m_shader);

    // Activate and bind texture
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_texture);

    // Bind VAO
    glBindVertexArray(m_vao);

    // Draw
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    // Unbind VAO, texture, and shader
    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glUseProgram(0);
}

GLuint Render2D::createShaderProgram(const char *vertex_file_path, const char *fragment_file_path)
{
    // Create and compile the shaders.
    GLuint vertexShaderID = createShader(GL_VERTEX_SHADER, vertex_file_path);
    GLuint fragmentShaderID = createShader(GL_FRAGMENT_SHADER, fragment_file_path);

    // Link the shader program.
    GLuint programID = glCreateProgram();
    glAttachShader(programID, vertexShaderID);
    glAttachShader(programID, fragmentShaderID);
    glLinkProgram(programID);

    // Print the info log if error
    GLint status;
    glGetProgramiv(programID, GL_LINK_STATUS, &status);
    if (status == GL_FALSE) {
        GLint length;
        glGetProgramiv(programID, GL_INFO_LOG_LENGTH, &length);
        std::string log(length, '\0');
        glGetProgramInfoLog(programID, length, nullptr, &log[0]);
        glDeleteProgram(programID);
        throw std::runtime_error(log);
    }

    // Shaders no longer necessary, stored in program
    glDeleteShader(vertexShaderID);
    glDeleteShader(fragmentShaderID);
    return programID;
}

GLuint Render2D::createShader(GLenum shaderType, const char *filepath)
{
    GLuint shaderID = glCreateShader(shaderType);

    // Read shader file.
    std::string code;
    QString filepathStr = QString(filepath);
    QFile file(filepathStr);
    if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QTextStream stream(&file);
        code = stream.readAll().toStdString();
    } else {
        throw std::runtime_error(std::string("Failed to open shader: ") + filepath);
    }

    // Compile shader code.
    const char *codePtr = code.c_str();
    glShaderSource(shaderID, 1, &codePtr, nullptr); // Assumes code is null terminated
    glCompileShader(shaderID);

    // Print info log if shader fails to compile.
    GLint status;
    glGetShaderiv(shaderID, GL_COMPILE_STATUS, &status);
    if (status == GL_FALSE) {
        GLint length;
        glGetShaderiv(shaderID, GL_INFO_LOG_LENGTH, &length);
        std::string log(length, '\0');
        glGetShaderInfoLog(shaderID, length, nullptr, &log[0]);
        glDeleteShader(shaderID);
        throw std::runtime_error(log);
    }
    return shaderID;
}

void Render2D::resizeGL(int w, int h) {}

void Render2D::finish()
{
    glDeleteProgram(m_shader);
    glDeleteVertexArrays(1, &m_vao);
    glDeleteBuffers(1, &m_vbo);
    glDeleteTextures(1, &m_texture);

    doneCurrent();
}

void transformVerticesByRatio()
{

}

void transformUV()
{

}
