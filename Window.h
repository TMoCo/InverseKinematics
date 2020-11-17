#ifndef _WINDOW_H
#define _WINDOW_H

#include <string>

#include <QtWidgets>

#include "BVHWidget.h"

// convenience class for buttons
class CtrlButton : public QPushButton
{
    Q_OBJECT
    public:
    //  constructor, set the size 
    CtrlButton(QString label, QWidget *parent);
};

// main window class for BVH viewing and editing
class Window : public  QMainWindow
{
    Q_OBJECT
    public:

    // constructor
    Window(QWidget *parent);
    // destructor
    ~Window();

    // slots for file loading and saving
    public slots:
    void loadFileDialog();
    void updateFrameSlider(int nFrames);
    void updateTimerInterval(int interval);
    void incrementFrame();
    void goToFirstFrame();
    void goToLastFrame();
    //void multInterval();

    // signals for file loading and saving
    signals:
    void selectedBVH(QString fileName);
    
    private:
    // widget for rendering and interacting with BVH file
    BVHWidget *bvhWidget;

    // widgets for loading and saving BVH
    QMenu *fileMenu;
    QAction *open;
    QAction *save;

    // widgets for control dock
    QDockWidget *ctrlDock;
    QGroupBox *ctrlBox;
    QGridLayout *ctrlLayout;
    // widgets for controlling the animation
    CtrlButton *play;
    CtrlButton *stop;
    CtrlButton *start;
    CtrlButton *ffwrd;
    CtrlButton *rewnd;
    CtrlButton *end;
    QSlider *frames;

    // timer for animating the scene
    QTimer *timer;
};


#endif