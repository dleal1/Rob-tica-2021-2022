#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButtonStop()));
    connect(getPeriodo, SIGNAL(clicked()), this, SLOT(doButtonGetPeriodo()));
    connect(setPeriodo, SIGNAL(clicked()), this, SLOT(doButtonSetPeriodo()));
    connect(getTimeElapsed, SIGNAL(clicked()), this, SLOT(doButtonGetTimeElapsed()));

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(MyTimerSlot()));

    timer->start(1000);
}

/*void ejemplo1::doButtonStop()
{
        timer->stop();
        pantalla->setText("Contador detenido");
        thread()->sleep(5);
        lcdNumber->display(0);
}*/

void ejemplo1::doButtonStop()
{
    static bool stopped = false;
    stopped = !stopped;
    if(stopped)
        mytimer.stop();
    else
        mytimer.start(500);
    pantalla->setText("Contador detenido");
}

void ejemplo1::MyTimerSlot()
{
    lcdNumber->display(++cont);
}

void ejemplo1::doButtonGetPeriodo() {
    pantalla->setText("El período es: ")
}

void ejemplo1::doButtonSetPeriodo() {
    qDebug() << "";
}

void ejemplo1::doButtonGetTimeElapsed() {
    qDebug() << "";
}



