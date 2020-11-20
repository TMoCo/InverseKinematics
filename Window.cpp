#include <iostream>

#include <QFileDialog>

#include "Window.h"


// constructor for control button
CtrlButton::CtrlButton(QString label, QWidget *parent) 
    : QPushButton(label, parent)
{
    setFixedSize(70, 30);
}

Window::Window(QWidget *parent) 
    : QMainWindow(parent)
{
    // OpenGL widget for BVH viewing
    bvhWidget = new BVHWidget(this);

    // timer widget for animation
    timer = new QTimer(this);
    timer->setInterval(50); // in ms
    // connect the widget to the update slot of an openGL widget
    QObject::connect(timer, SIGNAL(timeout()), this, SLOT(incrementFrame()));
    // connect the signal from a loaded BVH file to update the timer
    QObject::connect(bvhWidget, SIGNAL(setTimerInterval(int)), this, SLOT(updateTimerInterval(int)));

    //
    // Window layout
    //

    // add file menu to the main window's menu bar
    fileMenu = menuBar()->addMenu(tr("&File"));
    // create actions for BVH files 
    open = new QAction(tr("&Open"), this);
    open->setStatusTip(tr("Open a BVH file"));
    save = new QAction(tr("&Save"), this);
    save->setStatusTip(tr("Save a BVH file"));
    // connect them to a slot
    // connect open 
    QObject::connect(open, SIGNAL(triggered()), this, SLOT(loadFileDialog()));
    QObject::connect(this, SIGNAL(selectedOpenBVH(QString)), bvhWidget, SLOT(loadBVHFile(QString)));
    // connect save
    QObject::connect(save, SIGNAL(triggered()), this, SLOT(saveFileDialog()));
    QObject::connect(this, SIGNAL(selectedSaveBVH(QString)), bvhWidget, SLOT(saveBVHFile(QString)));
    // add the actions to the file menu
    fileMenu->addAction(open);
    fileMenu->addAction(save);

    // create the box widget to contain control widgets (play, pause, frame select...)
    ctrlBox = new QGroupBox;
    ctrlLayout = new QGridLayout;
    // create the control widgets
    play = new CtrlButton("&Play", this);
    stop = new CtrlButton("&Stop", this);
    start = new CtrlButton("&Start", this);
    frwrd = new CtrlButton("&Frwrd", this);
    rewnd = new CtrlButton("&Rewnd", this);
    end = new CtrlButton("&End", this);
    frames = new QSlider(Qt::Horizontal, this);
    frames->setMaximum(0);
    // add them to the box widget 
    ctrlLayout->addWidget(start, 0, 0);
    ctrlLayout->addWidget(rewnd, 0, 1);
    ctrlLayout->addWidget(play, 0, 2);
    ctrlLayout->addWidget(stop, 0, 3);
    ctrlLayout->addWidget(frwrd, 0, 4);
    ctrlLayout->addWidget(end, 0, 5);
    ctrlLayout->addWidget(frames, 1, 0, 1, 6);
    ctrlBox->setLayout(ctrlLayout);
    // connect the control widgets
    // connect start to load first frame
    QObject::connect(start, SIGNAL(pressed()), this, SLOT(goToFirstFrame()));
    // connect play to start playing the animation (start the timer)
    QObject::connect(play, SIGNAL(pressed()), timer, SLOT(start()));
    // connect stop to pause playing the animation
    QObject::connect(stop, SIGNAL(pressed()), timer, SLOT(stop()));
    // connect frwrd to increase animation play speed
    QObject::connect(frwrd, SIGNAL(pressed()), bvhWidget, SLOT(changePlayDirPlus()));
    // connect rewnd to decrease animation play speed
    QObject::connect(rewnd, SIGNAL(pressed()), bvhWidget, SLOT(changePlayDirMinus()));
    // connect end to load last frame
    QObject::connect(end, SIGNAL(pressed()), this, SLOT(goToLastFrame()));
    // connect slider to load a new frame on value change (must change value range upon loading a BVH file)
    QObject::connect(bvhWidget, SIGNAL(setMaxFrame(int)), this, SLOT(updateFrameSlider(int)));
    // the slider controls the animation so connect it to the animate slot
    QObject::connect(frames, SIGNAL(valueChanged(int)), bvhWidget, SLOT(animateFigure(int)));
    // create the dock widget containing the animation controls
    ctrlDock = new QDockWidget(tr("Animation Controls"), this);
    ctrlDock->setAllowedAreas(Qt::BottomDockWidgetArea);
    ctrlDock->setWidget(ctrlBox);
    addDockWidget(Qt::BottomDockWidgetArea, ctrlDock);

    // create the widgets to contain the IK options
    ikBox = new QGroupBox;
    ikLayout = new QGridLayout;
    // create the IK widgets
    dampening = new QCheckBox(this);
    dampening->setCheckState(Qt::Unchecked);
    dampeningLabel = new QLabel(tr("Dampening"), this);
    dampeningLambda = new QSlider(Qt::Horizontal, this);
    dampeningLambda->setRange(1, 5);
    saveFrame = new CtrlButton("&Save", this);
    resetArcBall = new CtrlButton("&Reset", this);
    // add widgets to the dock and set layout
    ikLayout->setAlignment(Qt::AlignTop); 
    ikLayout->addWidget(dampeningLabel, 0, 0);
    ikLayout->addWidget(dampening, 0, 3);
    ikLayout->addWidget(dampeningLambda, 1, 0, 1, 4);
    ikLayout->addWidget(saveFrame, 3, 0, 1, 5);
    ikLayout->addWidget(resetArcBall, 4, 0, 1, 5);
    ikBox->setLayout(ikLayout);
    // connect widgets
    // dampening tells bvh widget to dampen IK
    QObject::connect(dampening, SIGNAL(stateChanged(int)), bvhWidget,SLOT(setDampening(int)));
    // the slider changes the scalar value of the dampening
    QObject::connect(dampeningLambda, SIGNAL(valueChanged(int)), bvhWidget, SLOT(updateLambda(int)));
    // the button that informs the BVHWidget to update the bvh file's motion with the IK computed values (in currentMotion)
    QObject::connect(saveFrame, SIGNAL(pressed()), bvhWidget, SLOT(saveFrame()));
    // reset the rotation of the arcball
    QObject::connect(resetArcBall, SIGNAL(pressed()), bvhWidget, SLOT(resetRotation()));
    // create the dock widget for IK control
    ikDock = new QDockWidget(tr("IK controls"), this);
    ikDock->setAllowedAreas(Qt::LeftDockWidgetArea);
    ikDock->setWidget(ikBox);
    addDockWidget(Qt::LeftDockWidgetArea, ikDock);

    // central widget where we render
    setCentralWidget(bvhWidget);

    // initialise the status bar message, also creates it
    statusBar()->showMessage(tr("Ready"));
}

// delete all widgets
Window::~Window()
{
    // there must be a better way of doing this...
}

// opens up a file browser dialog and emits a signal to the GL widget if a BVH file is chosen
void Window::loadFileDialog()
{
    // open dialog
    QString fileName = QFileDialog::getOpenFileName(this, tr("&Load BVH"), "./", tr("BVH (*.bvh)"));
    // check name chosen is not empty
    if (!fileName.isEmpty())
        emit selectedOpenBVH(fileName);
}

void Window::saveFileDialog()
{
    // open dialog
    QString fileName = QFileDialog::getSaveFileName(this, tr("Load BVH"), "./", tr("BVH (*.bvh)"));
    // check if file name is valid
    if (!fileName.isEmpty() )
    {
        // get the file's info
        QFileInfo fileInf(fileName);
        // set the suffix of the file to ".bvh"
        // user entered just a file name (no extensions)
        if (fileInf.completeSuffix().isEmpty())
            fileName.append(".bvh");
        // user entered an extension that is not .bvh
        else if (fileInf.completeSuffix() != QString("bvh"))
            fileName.replace(fileInf.completeSuffix(), QString("bvh"));
        // selected file already exists, overwrite?
        if (QFileInfo::exists(fileName))
        {
            // ask user
            QMessageBox errorMsg;
            errorMsg.setText("You are about to overwrite an existing bvh file, proceed?");
            errorMsg.setInformativeText("\"" +  fileName + "\"" + " already exists.");
            errorMsg.setStandardButtons(QMessageBox::Save | QMessageBox::Cancel);
            errorMsg.setDefaultButton(QMessageBox::Cancel);
            // get user input
            int selected = errorMsg.exec();
            // only two outcomes possible
            switch(selected)
            {
                // don't overwrite, reopen the dialog
                case QMessageBox::Cancel:
                    saveFileDialog();
                    break;
                // do overwrite, emit file name
                case QMessageBox::Save:
                    emit selectedSaveBVH(fileName);
                    break;
            }
        }
        // file doesnt exist, we are good to save
        else
            emit selectedSaveBVH(fileName);     
    }
}

// slot that set the slider's maximum to the number of frames in a loaded BVH file
void Window::updateFrameSlider(int nFrames)
{
    goToFirstFrame();
    frames->setMaximum(nFrames);
}

// slot that updates the timer's time interval 
void Window::updateTimerInterval(int interval)
{
    timer->setInterval(interval);
}

// connected to the timer timeout signal, increments the slider value by 1
void Window::incrementFrame()
{
    // get current value in slider
    int curr = frames->value();
    // set the slider to current + the play direction, automatically clamped
    frames->setValue(curr + bvhWidget->playDir);
}

// reset the slider to initial value
void Window::goToFirstFrame()
{
    frames->setValue(frames->minimum());
}

// set the slider to maximum value
void Window::goToLastFrame()
{
    frames->setValue(frames->maximum());
}

