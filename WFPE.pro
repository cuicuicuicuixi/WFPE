QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    physics/Collision/GJK.cpp \
    physics/algo/kdtree.cpp \
    physics/physicalworld.cpp \
    render/GLwindow.cpp

HEADERS += \
    mainwindow.h \
    physics/Collision/Collider.h \
    physics/Collision/Collision.h \
    physics/Collision/CollisionObject.h \
    physics/Collision/CollisionPoints.h \
    physics/Collision/DetectCollisoin.h \
    physics/Collision/GJK.h \
    physics/Collision/HullCollider.h \
    physics/Collision/PlaneCollider.h \
    physics/Collision/SphereCollider.h \
    physics/Dynamic/ImpluseSolveer.h \
    physics/Dynamic/Solver.h \
    physics/Dynamic/smoothPositionSolver.h \
    physics/algo/kdtree.h \
    physics/physicalworld.h \
    render/GLwindow.h \
    render/qCamera.h

INCLUDEPATH += \
              .\include \
              .\include/eigen \

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    shader.qrc
