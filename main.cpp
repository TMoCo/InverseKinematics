// Where the main window is created 

#include "Window.h"

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    Window aWindow(NULL);

    aWindow.resize(1024,768);

    aWindow.show();

    return app.exec();
}