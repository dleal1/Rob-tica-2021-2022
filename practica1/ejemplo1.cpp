#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButtonStop()) );
    connect(getPeriodo, SIGNAL(clicked()), this, SLOT(doButtonGetPeriodo()) );
    connect(setPeriodo, SIGNAL(clicked()), this, SLOT(doButtonSetPeriodo()) );
    connect(getTimeElapsed, SIGNAL(clicked()), this, SLOT(doButtonGetTimeElapsed()) );
/*
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(MyTimerSlot()));
    timer->start(1000);
*/
}

void ejemplo1::doButtonStop()
{

    if (activoContador){
        //timer.stop();
    }
    else{
        //timer.start(500);
    }
    doImprimirPantalla(0);
    activoContador = !activoContador;
}

void ejemplo1::doButtonGetPeriodo()
{
    doImprimirPantalla(1);
}

void ejemplo1::doButtonSetPeriodo()
{
    int nuevoPeriodo = 0;

    //qDebug() << "Incremento del timer" + cont;

    //Esto es para pedir el nuevo periodo pero no se como leer y llevo un buen rato buscando

    pantalla->setText("Introduce el nuevo valor del periodo (Cuando se borre el texto)");
    pantalla->setText("");

    //Crear una variable para leer el valor y ponerla en el setPeriodo

    //timer.setPeriodo ();

    doImprimirPantalla(2);
}

void ejemplo1::doButtonGetTimeElapsed()
{
    doImprimirPantalla(3);
}

void ejemplo1::doImprimirPantalla(int opcionTexto)
{
    QString texto = "";

    switch (opcionTexto) {
        case 0:
            if (activoContador){
                texto = "SE HA PARADO EL CONTADOR";
            }
            else {
                texto = "SE HA ACTIVADO EL CONTADOR";
            }

            break;
        case 1:
            texto = "VALOR PERIODO -> " /*+ mytimer.getPeriodo()*/;
            break;
        case 2:
            texto = "NUEVO VALOR PERIODO -> " /*+ mytimer.getPeriodo()*/;
            break;
        case 3:
            texto = "TIEMPO DE EJECUCION -> " + cont;
            break;
    }

    pantalla->setText(texto);
}

void ejemplo1::MyTimerSlot()
{
    lcdNumber->display(++cont);
    //qDebug() << "Incremento del timer" + cont;
}




