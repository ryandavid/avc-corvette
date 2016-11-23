#include <QApplication>
#include "dashboard/dashboard.h"

int main(int argc, char** argv) {
    QApplication app(argc, argv);

    Dashboard s(argc, argv);

	return app.exec();
}
