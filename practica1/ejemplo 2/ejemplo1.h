#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "cmake-build-debug/ui_counterDlg.h"
#include <QTimer>
#include "timer.h"

class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
    public:
        ejemplo1();

    private:
        //QTimer *timer;
        int cont = 0;
        Timer mytimer, mytimerLong;

    public slots:
        void doButtonStop();
        void doButtonGetPeriodo();
        void doButtonSetPeriodo();
        void doButtonGetTimeElapsed();
        void MyTimerSlot();
};

#endif // ejemplo1_H
