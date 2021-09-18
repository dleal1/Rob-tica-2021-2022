#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "cmake-build-debug/ui_counterDlg.h"
#include <QTimer>
//#include "timer.h"

class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
    public:
        ejemplo1();

    private:
        bool activoContador = true;
        int cont = 0;

    public slots:
        void doButtonStop();
        void doButtonGetPeriodo();
        void doButtonSetPeriodo();
        void doButtonGetTimeElapsed();
        void doImprimirPantalla(int opcionTexto);
        void MyTimerSlot();
};

#endif // ejemplo1_H
