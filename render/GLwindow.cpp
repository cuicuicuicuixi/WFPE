#include "GLwindow.h"

GLwindow::GLwindow(QWidget *parent) :
    QOpenGLWidget(parent),
    camera(this)
{
    timer.setInterval(18);
    connect(&timer,&QTimer::timeout, this, static_cast<void (GLwindow::*)()>(&GLwindow::update));
    timer.start();
    QSurfaceFormat a;
    a.setSamples(4);
    setFormat(a);
}

GLwindow::~GLwindow()
{
    makeCurrent();
    doneCurrent();
}

QSize GLwindow::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize GLwindow::sizeHint() const
{
    return QSize(200, 200);
}

void GLwindow::setClearColor(const QColor &color)
{
    clearColor = color;
    update();
}

void GLwindow::initializeGL()
{
    initializeOpenGLFunctions();

    camera.SetPosition(QVector3D(0.0f, 0.0f, 300.0f));
    camera.SetMovementSpeed(500.0f);
    camera.SetMovementSpeed(1000.0f);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);

    framebuffer = new QOpenGLFramebufferObject(this->width(), this->height());
    intermediateFBO = new QOpenGLFramebufferObject(this->width(), this->height());
    FrameBufferConfiguration();
    QuadInit();
    AxisInit();

    physicProgram.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shader/blinePhong.vert");
    physicProgram.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shader/blinePhong.frag");
    physicProgram.link();
    physical.init();

    lastFrame = myGetTime();

}


void GLwindow::paintGL()
{
    float currentFrame = myGetTime();
    deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;

    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    // 渲染
    // ------
    framebuffer->bind(); // 多采样缓冲区

    glEnable(GL_DEPTH_TEST);
    glClearColor(clearColor.redF(), clearColor.greenF(), clearColor.blueF(), clearColor.alphaF());
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT); // don't forget to clear the stencil buffer!

    physical.Step(deltaTime);

    QMatrix4x4 model;
    QMatrix4x4 projection;
    projection.perspective(camera.GetZoom(), (float)this->width() / (float)this->height(), 0.001f, 100000.0f);
    QMatrix4x4 view = camera.GetViewMatrix();
    AxisRendering(model, projection, view);

    physicProgram.bind();
    physicProgram.setUniformValue("model", model);
    physicProgram.setUniformValue("view", view);
    physicProgram.setUniformValue("projection", projection);
    physicProgram.setUniformValue("camPos", camera.GetPosition());
    physical.Draw(context()->versionFunctions<QOpenGLFunctions_3_3_Core>(), &physicProgram);

    // 渲染结束
    //从多采样缓冲区传递数据到默认缓冲区
    // ---------
    framebuffer->release();
    intermediateFBO->bind();
    QOpenGLFramebufferObject::blitFramebuffer(intermediateFBO,framebuffer,GL_COLOR_BUFFER_BIT| GL_DEPTH_BUFFER_BIT| GL_STENCIL_BUFFER_BIT,GL_NEAREST);
    QVector<GLuint> textureid;//屏幕纹理
    textureid = intermediateFBO->textures();
    intermediateFBO->release();

    //抓取
    intermediateFBO->bindDefault();
    //

    // 连接到默认缓冲区并用得到的纹理渲染屏幕
    QOpenGLFramebufferObject::bindDefault();// clear all relevant buffers
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // 重新清理屏幕为白色
    glClear(GL_COLOR_BUFFER_BIT);
    QuadRendering(textureid.at(0));
}

void GLwindow::resizeGL(int width, int height)
{
    int side = qMin(width, height);
    glViewport((width - side) / 2, (height - side) / 2, side, side);
    FrameBufferConfiguration();
}

// MSAA抗锯齿*******************************************************************
//
void GLwindow::QuadInit()
// 屏幕四边形设置
{
    if(quadVAO.objectId() == 0)
    {
        screenShaderProgram.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shader/framebuffer.vert");
        screenShaderProgram.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shader/framebuffer.frag");
        screenShaderProgram.link();
        float quadVertices[] =
        {
            // vertex attributes for a quad that fills the entire screen in Normalized Device Coordinates.
            // positions       // texCoords
            //主屏幕
            -1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
            -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
             1.0f, -1.0f, 0.0f, 1.0f, 0.0f,

            -1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
             1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
             1.0f,  1.0f, 0.0f, 1.0f, 1.0f,
             //分屏1
            -1.0f,  0.0f, 0.0f, 0.0f, 1.0f,
            -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
             1.0f, -1.0f, 0.0f, 1.0f, 0.0f,

            -1.0f,  0.0f, 0.0f, 0.0f, 1.0f,
             1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
             1.0f,  0.0f, 0.0f, 1.0f, 1.0f,
              //分屏2
            -1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
            -1.0f,  0.0f, 0.0f, 0.0f, 0.0f,
             0.0f,  0.0f, 0.0f, 1.0f, 0.0f,

            -1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
             0.0f,  0.0f, 0.0f, 1.0f, 0.0f,
             0.0f,  1.0f, 0.0f, 1.0f, 1.0f,
            //分屏3
             0.0f,  1.0f, 0.0f, 0.0f, 1.0f,
             0.0f,  0.0f, 0.0f, 0.0f, 0.0f,
             1.0f,  0.0f, 0.0f, 1.0f, 0.0f,

             0.0f,  1.0f, 0.0f, 0.0f, 1.0f,
             1.0f,  0.0f, 0.0f, 1.0f, 0.0f,
             1.0f,  1.0f, 0.0f, 1.0f, 1.0f,

        };
        QOpenGLVertexArrayObject::Binder binder{&quadVAO};
        QOpenGLBuffer vbo(QOpenGLBuffer::VertexBuffer);

        vbo.create();
        vbo.bind();
        vbo.allocate(&quadVertices, sizeof(quadVertices));

        screenShaderProgram.setUniformValue("screenTexture", 0);
        screenShaderProgram.setUniformValue("bloomBlur", 1);
        int Loaction = screenShaderProgram.attributeLocation("aPosi");
        screenShaderProgram.setAttributeBuffer(Loaction, GL_FLOAT, 0, 3, 5 * sizeof(float));
        screenShaderProgram.enableAttributeArray(Loaction);
        Loaction = screenShaderProgram.attributeLocation("aTexCoords");
        screenShaderProgram.setAttributeBuffer(Loaction, GL_FLOAT, 3 * sizeof(float), 2, 5 * sizeof(float));
        screenShaderProgram.enableAttributeArray(Loaction);
        binder.release();
        screenShaderProgram.release();
    }
}

void GLwindow::QuadRendering(GLint textureid)
{
    screenShaderProgram.bind();
    glDisable(GL_DEPTH_TEST);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, textureid);
    screenShaderProgram.setUniformValue("hdr", false);
    screenShaderProgram.setUniformValue("exposure", 1.0f);
    screenShaderProgram.setUniformValue("gray", false);
    screenShaderProgram.setUniformValue("gamma", 1.0f);
    screenShaderProgram.setUniformValue("bloom", true);
    QOpenGLVertexArrayObject::Binder binder{&quadVAO};
    this->glDrawArrays(GL_TRIANGLES, 0, 6);
}

void GLwindow::FrameBufferConfiguration()
// 多采样缓冲区设置
{
    if(framebuffer->isValid()&&intermediateFBO->isValid())
    {
        framebuffer->~QOpenGLFramebufferObject();
        intermediateFBO->~QOpenGLFramebufferObject();
    }
    QOpenGLFramebufferObjectFormat muliSampleFormat;
    muliSampleFormat.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);
    muliSampleFormat.setMipmap(true);
    muliSampleFormat.setSamples(4);
    muliSampleFormat.setTextureTarget(GL_TEXTURE_2D_MULTISAMPLE);
    muliSampleFormat.setInternalTextureFormat(GL_RGBA32F_ARB);
    framebuffer = new QOpenGLFramebufferObject(this->width(), this->height(), muliSampleFormat);
    QOpenGLFramebufferObjectFormat downSampledFormat;
    downSampledFormat.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);
    downSampledFormat.setMipmap(true);
    downSampledFormat.setTextureTarget(GL_TEXTURE_2D);
    downSampledFormat.setInternalTextureFormat(GL_RGBA32F_ARB);
    intermediateFBO = new QOpenGLFramebufferObject(this->width(), this->height(), downSampledFormat);
}

// 坐标系**************************************************************************
//
void GLwindow::AxisInit()
// 坐标系初始化
{
    if(axisVAO.objectId() == 0)
    {
        axisProgram.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shader/axis.vert");
        axisProgram.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shader/axis.frag");
        axisProgram.link();
        float axisver[] = {
                // positions      // colors
               -100.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f,
                100.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f,
                0.0f,-100.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f,
                0.0f, 100.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f,
                0.0f, 0.0f,-100.0f, 0.0f, 0.0f, 1.0f, 1.0f,
                0.0f, 0.0f, 100.0f, 0.0f, 0.0f, 1.0f, 1.0f
            };

        QOpenGLVertexArrayObject::Binder binder{&axisVAO};
        QOpenGLBuffer vbo(QOpenGLBuffer::VertexBuffer);


        vbo.create();
        vbo.bind();
        vbo.allocate(&axisver, sizeof(axisver));

        int Location = axisProgram.attributeLocation("aPos");
        axisProgram.enableAttributeArray(Location);
        axisProgram.setAttributeBuffer(Location, GL_FLOAT, 0, 3, 7 * sizeof(float));
        Location = axisProgram.attributeLocation("aColor");
        axisProgram.enableAttributeArray(Location);
        axisProgram.setAttributeBuffer(Location, GL_FLOAT, 3 * sizeof(float), 4, 7 * sizeof(float));
        binder.release();
        axisProgram.release();
    }
}

void GLwindow::AxisRendering(QMatrix4x4 model, QMatrix4x4 projection, QMatrix4x4 view)
// 坐标系渲染
{
    axisProgram.bind();
    axisProgram.setUniformValue("model", model);
    axisProgram.setUniformValue("view", view);
    axisProgram.setUniformValue("projection", projection);
    QOpenGLVertexArrayObject::Binder binder{&axisVAO};
    this->glDrawArrays(GL_LINES, 0, 6);
}
//**************************************************************************


// 鼠标键盘事件*************************************************************************
//
bool GLwindow::event(QEvent *e)
{
    camera.Hadle(e, deltaTime);
    return QWidget::event(e);
}
void GLwindow::mousePressEvent(QMouseEvent *event)
// 鼠标按下
{

}

void GLwindow::mouseMoveEvent(QMouseEvent *)
// 鼠标移动
{

}

void GLwindow::mouseReleaseEvent(QMouseEvent * /* event */)
// 鼠标释放
{

}

void GLwindow::wheelEvent(QWheelEvent *)
// 滚轮事件
{

}

void GLwindow::keyPressEvent(QKeyEvent *)
// 键盘按下
{

}

void GLwindow::keyReleaseEvent(QKeyEvent *)
{

}

void GLwindow::timerEvent(QTimerEvent* /*event*/)                //键盘操作
{

}

void GLwindow::enterEvent(QEvent *)
{

}

void GLwindow::leaveEvent(QEvent *)
{

}

float GLwindow::myGetTime()
{
    float h=QDateTime::currentDateTime().toString("hh").toFloat();
    float m=QDateTime::currentDateTime().toString("mm").toFloat();
    float s=QDateTime::currentDateTime().toString("ss").toFloat();
    float z=QDateTime::currentDateTime().toString("zzz").toFloat();
    return h*3600.0f + m*60.0f + s + z/1000.0f;
}
