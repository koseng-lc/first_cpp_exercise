TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp
INCLUDEPATH += /usr/local/include/
LIBS += -L/usr/local/lib/ -lopencv_highgui -lopencv_imgproc -lopencv_core
