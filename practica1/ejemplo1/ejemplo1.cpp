#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(MyTimerSlot()));

    timer->start(1000);
}

void ejemplo1::doButton()
{
    timer->stop();
	qDebug() << "Contador detenido";
    thread()->sleep(5);
    lcdNumber->display(0);

}

void ejemplo1::MyTimerSlot()
{
    int i;
    lcdNumber->display(i);
    i++;
}




