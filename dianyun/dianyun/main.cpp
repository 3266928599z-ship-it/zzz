#include "dianyun.h"
#include <QtWidgets/QApplication>

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
    dianyun w;
    w.show();
    return a.exec();
}