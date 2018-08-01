#include <QApplication>
#include "ponnector.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    a.setApplicationName("Ponnector");
    a.setApplicationVersion(QObject::tr("0.0.1"));

    Ponnector ponnector;
    ponnector.ponnector_main();

    return a.exec();
}
