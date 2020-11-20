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

    //  constructor, sets the geometry of button 
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
    void saveFileDialog();
    void updateFrameSlider(int nFrames);
    void updateTimerInterval(int interval);
    void incrementFrame();
    void goToFirstFrame();
    void goToLastFrame();
    

    // signals for file loading and saving
    signals:
    void selectedOpenBVH(QString fileName);
    void selectedSaveBVH(QString fileName);
    

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
    CtrlButton *frwrd;
    CtrlButton *rewnd;
    CtrlButton *end;
    QSlider *frames;
    

    // widgets for IK dock
    QDockWidget *ikDock;
    QGroupBox *ikBox;
    QGridLayout *ikLayout;
    // widgets for controlling th IK
    QSlider *dampeningLambda;
    QLabel *dampeningLabel;
    QCheckBox *dampening;
    CtrlButton *saveFrame;
    CtrlButton *resetArcBall;


    // timer for animating the scene
    QTimer *timer;
};


#endif