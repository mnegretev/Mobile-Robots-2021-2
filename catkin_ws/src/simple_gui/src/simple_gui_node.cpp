#include <iostream>
#include <QApplication>
#include <ros/ros.h>
#include "MainWindow.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING SIMPLE GUI NODE..." << std::endl;
    ros::init(argc, argv, "simple_gui");
    ros::NodeHandle n;

    QtRosNode qtRosNode;
    qtRosNode.setNodeHandle(&n);
    qtRosNode.start();

    QApplication app(argc, argv);
    MainWindow mainWindow;
    mainWindow.setRosNode(&qtRosNode);

    mainWindow.show();
    return app.exec();
}
