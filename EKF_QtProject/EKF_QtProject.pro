#-------------------------------------------------
#
# Project created by QtCreator 2017-03-24T13:33:55
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = EKF_QtProject
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    FusionEKF.cpp \
    kalman_filter.cpp \
    tools.cpp

HEADERS += \
    FusionEKF.h \
    ground_truth_package.h \
    kalman_filter.h \
    measurement_package.h \
    tools.h
