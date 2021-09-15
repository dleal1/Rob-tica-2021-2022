#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "cmake-build-debug/ui_counterDlg.h"
#include <QTimer>

class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
    public:
        ejemplo1();

    private:
        QTimer *timer;

    public slots:
        void doButton();
        void MyTimerSlot();
};

#endif // ejemplo1_H
