#pragma once

// Qt
#include <QObject>
#include <QMouseEvent>
#include <QDateTime>
#include <QTimer>
// QOpenGL
#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLFramebufferObject>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>

#include "qCamera.h"
#include "physics/physicalworld.h"


class GLwindow : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
    Q_OBJECT
public:
    explicit GLwindow(QWidget *parent = nullptr);

    ~GLwindow();

    QCamera camera;
    void setClearColor(const QColor &color);
    float myGetTime();


protected:
    QSize minimumSizeHint() const Q_DECL_OVERRIDE;
    QSize sizeHint() const Q_DECL_OVERRIDE;
    void initializeGL() Q_DECL_OVERRIDE;
    void paintGL() Q_DECL_OVERRIDE;
    void resizeGL(int width, int height) Q_DECL_OVERRIDE;
    bool event(QEvent * e) Q_DECL_OVERRIDE;
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void wheelEvent(QWheelEvent *event) Q_DECL_OVERRIDE;
    void keyPressEvent(QKeyEvent *event) Q_DECL_OVERRIDE;
    void keyReleaseEvent(QKeyEvent *event) Q_DECL_OVERRIDE;
    void timerEvent(QTimerEvent *) Q_DECL_OVERRIDE;
    void enterEvent(QEvent *)Q_DECL_OVERRIDE;
    void leaveEvent(QEvent *)Q_DECL_OVERRIDE;

private:
    QColor clearColor = QColor(255, 255, 255, 1.0); // 屏幕清理
    QTimer timer; // 刷新 widget

    physE::physicalworld physical;
    QOpenGLShaderProgram physicProgram;

    // MSAA 抗锯齿*********************
    QOpenGLFramebufferObject *framebuffer;
    QOpenGLFramebufferObject *intermediateFBO;
    QOpenGLVertexArrayObject quadVAO;
    QOpenGLShaderProgram screenShaderProgram;
    void QuadInit();
    void QuadRendering(GLint textureid);
    void FrameBufferConfiguration();

    // 坐标轴渲染*******************
    QOpenGLVertexArrayObject axisVAO;
    QOpenGLShaderProgram axisProgram;
    void AxisInit();
    void AxisRendering(QMatrix4x4 model, QMatrix4x4 projection, QMatrix4x4 view);

    // 计算帧生成时间*************
    float deltaTime = 0.0f;
    float lastFrame = 0.0f;
    // 鼠标键盘事件*************
    QPoint lastPos; //鼠标按下前的最终位置
    bool press = false; // 鼠标是否按下
    int xRot;
    int yRot;
    int zRot;
    QSet<int> keys;     //记录当前被按下按键的集合
    int timeId = 0;         //定时器id：此定时器用于完成键盘移动事件
};
