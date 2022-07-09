#ifndef QCAMERA_H
#define QCAMERA_H
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <QMatrix4x4>
#include <QVector3D>
#include <QMouseEvent>
#include <QTime>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
//#include <QOpenGLBuffer>

#include <vector>

// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific input methods
enum Camera_Movement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

// Default camera values
const float YAW = -90.0f;
const float PITCH = 0.0f;
const float SPEED = 200.5f;
const float SENSITIVITY = 0.1f;
const float ZOOM = 45.0f;

// An abstract camera class that processes input and calculates the corresponding Euler Angles, Vectors and Matrices for use in OpenGL
class QCamera
{
public:
    // constructor with vectors
    QCamera(QWidget* widget) :
        widget(widget),
        Position(QVector3D(0.0f, 0.0f, 0.0f)),
        Front(QVector3D(0.0f, 0.0f, -1.0f)),
        WorldUp(QVector3D(0.0f, 1.0f, 0.0f)),
        Yaw(YAW),
        Pitch(PITCH),
        MovementSpeed(SPEED),
        MouseSensitivity(SENSITIVITY),
        Zoom(ZOOM)
    {
        widget->activateWindow();
        widget->setFocusPolicy(Qt::ClickFocus);
        updateCameraVectors();
    }

    ~QCamera(){};

    QVector3D GetPosition() const
    {
        return Position;
    }

    void SetPosition(QVector3D position)
    {
        Position = position;
    }

    QVector3D GetFront() const
    {
        return Front;
    }

    void SetFront(QVector3D front)
    {
        Front = front;
    }

    QVector3D GetWorldUp() const
    {
        return WorldUp;
    }

    void SetWorldUp(QVector3D worldup)
    {
        WorldUp = worldup;
    }

    float GetYaw() const
    {
        return Yaw;
    }

    void SetYaw(float yaw)
    {
        Yaw = yaw;
    }

    float GetPitch() const
    {
        return Pitch;
    }

    void SetPitch(float pitch)
    {
        Pitch = pitch;
    }

    float GetMovementSpeed() const
    {
        return MovementSpeed;
    }

    void SetMovementSpeed(float movementspeed)
    {
        MovementSpeed = movementspeed;
    }

    float GetMouseSensitivity() const
    {
        return MouseSensitivity;
    }

    void SetMouseSensitivity(float mousesensitivity)
    {
        MouseSensitivity = mousesensitivity;
    }

    float GetZoom() const
    {
        return Zoom;
    }

    void SetZoom(float zoom)
    {
        Zoom = zoom;
    }

    // returns the view matrix calculated using Euler Angles and the LookAt Matrix
    QMatrix4x4 GetViewMatrix() const
    {
        QMatrix4x4 m;
        m.lookAt(Position, Position + Front, Up);
        return m;
    }

    void Hadle(QEvent *e, float deltaTime)
    {
        if(e->type() == QEvent::MouseButtonPress)
        {
            QMouseEvent* event = static_cast<QMouseEvent*>(e);
            lastPos = event->pos();
            Press = true;
        }
        else if (e->type() == QEvent::MouseMove)
        {
            QMouseEvent* event = static_cast<QMouseEvent*>(e);
            if(Press)
            {
                int dx = lastPos.x() - event->pos().x();
                int dy = event->pos().y() - lastPos.y();
                ProcessMouseMovement(-dx, -dy);

                lastPos = event->pos();
            }
            else if (!Press)
            {

            }
        }
        else if (e->type() == QEvent::MouseButtonRelease)
        {
            Press = false;
        }
        else if (e->type() == QEvent::Wheel)
        {
            QWheelEvent *event = static_cast<QWheelEvent *>(e);
            ProcessMouseScroll(event->delta()/120.0f);
        }
        else if (e->type() == QEvent::Timer)
        {
            if (keys.contains(Qt::Key_W))                           //前
                ProcessKeyboard(FORWARD, deltaTime/6.0f);
            if (keys.contains(Qt::Key_S))                           //后
                ProcessKeyboard(BACKWARD, deltaTime/6.0f);
            if (keys.contains(Qt::Key_A))                           //左
                ProcessKeyboard(LEFT, deltaTime/6.0f);
            if (keys.contains(Qt::Key_D))                           //右
                ProcessKeyboard(RIGHT, deltaTime/6.0f);
            widget->update();
        }
        else if (e->type() == QEvent::KeyPress)
        {
            QKeyEvent *event = static_cast<QKeyEvent *>(e);
            //isAutoRepeat用于判断此按键的来源是否是长按
            keys.insert(event->key());                              //添加按键
            if(!event->isAutoRepeat()&&timeId==0)
            {
                //如果定时器未启动，则启动定时器
                timeId = widget->startTimer(1);
            }
        }
        else if (e->type() == QEvent::KeyRelease)
        {
            QKeyEvent *event = static_cast<QKeyEvent *>(e);
            keys.remove(event->key());
            if(!event->isAutoRepeat()&&timeId!=0&&keys.empty())
            {
                //当没有按键按下且定时器正在运行，才关闭定时器
                widget->killTimer(timeId);
                timeId=0;
            }
        }
        else if (e->type() == QEvent::UpdateRequest)
        {
            qDebug()<<"update"<<endl;
        }
    }

    void reset()
    {
        // Default camera values
        Yaw = -90.0f;
        Pitch = 0.0f;
        updateCameraVectors();
    }

private:
    QWidget *widget;
    // camera Attributes
    QVector3D Position;
    QVector3D Front;
    QVector3D Up;
    QVector3D Right;
    QVector3D WorldUp;
    // euler Angles
    float Yaw;
    float Pitch;
    // camera options
    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;
    bool Press = false;
    // 鼠标键盘事件*************
    QPoint lastPos; //鼠标按下前的最终位置
    QSet<int> keys;     //记录当前被按下按键的集合
    int timeId = 0;         //定时器id：此定时器用于完成键盘移动事件

    QVector3D myCross(QVector3D A, QVector3D B)
    {
        QVector3D C(A.y()*B.z()-A.z()*B.y(), A.z()*B.x()-A.x()*B.z(), A.x()*B.y()-A.y()*B.x());
        return C;
    }

    // calculates the front vector from the Camera's (updated) Euler Angles
    void updateCameraVectors()
    {
        // calculate the new Front vector
        QVector3D front;
        front.setX(cos(glm::radians(Yaw)) * cos(glm::radians(Pitch)));
        front.setY(sin(glm::radians(Pitch)));
        front.setZ(sin(glm::radians(Yaw)) * cos(glm::radians(Pitch)));
        Front = front.normalized();
        // also re-calculate the Right and Up vector
        Right = myCross(Front, WorldUp).normalized();  // normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
        Up = myCross(Right, Front).normalized();
    }

    // processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
    void ProcessKeyboard(Camera_Movement direction, float deltaTime)
    {
        float velocity = MovementSpeed * deltaTime;
        if (direction == FORWARD)
            Position += Front * velocity;
        if (direction == BACKWARD)
            Position -= Front * velocity;
        if (direction == LEFT)
            Position -= Right * velocity;
        if (direction == RIGHT)
            this->Position += Right * velocity;
        widget->update();
    }

    // processes input received from a mouse input system. Expects the offset value in both the x and y direction.
    void ProcessMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch = true)
    {
        xoffset *= MouseSensitivity;
        yoffset *= MouseSensitivity;

        Yaw += xoffset;
        Pitch += yoffset;

        // make sure that when pitch is out of bounds, screen doesn't get flipped
        if (constrainPitch)
        {
            if (Pitch > 89.0f)
                Pitch = 89.0f;
            if (Pitch < -89.0f)
                Pitch = -89.0f;
        }

        // update Front, Right and Up Vectors using the updated Euler angles
        updateCameraVectors();
        widget->update();
    }

    // processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
    void ProcessMouseScroll(float yoffset)
    {
        Zoom -= float(yoffset);
        if (Zoom < 0.1f)
            Zoom = 0.1f;
        if (Zoom > 89.0f)
            Zoom = 89.0f;
        MouseSensitivity =  SENSITIVITY * Zoom/90;
        MovementSpeed = SPEED * Zoom/45;
    }

};

#endif // QCAMERA_H
