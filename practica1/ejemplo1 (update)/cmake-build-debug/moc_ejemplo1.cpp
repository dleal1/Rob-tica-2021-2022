/****************************************************************************
** Meta object code from reading C++ file 'ejemplo1.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../ejemplo1.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ejemplo1.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ejemplo1_t {
    QByteArrayData data[9];
    char stringdata0[127];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ejemplo1_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ejemplo1_t qt_meta_stringdata_ejemplo1 = {
    {
QT_MOC_LITERAL(0, 0, 8), // "ejemplo1"
QT_MOC_LITERAL(1, 9, 12), // "doButtonStop"
QT_MOC_LITERAL(2, 22, 0), // ""
QT_MOC_LITERAL(3, 23, 18), // "doButtonGetPeriodo"
QT_MOC_LITERAL(4, 42, 18), // "doButtonSetPeriodo"
QT_MOC_LITERAL(5, 61, 22), // "doButtonGetTimeElapsed"
QT_MOC_LITERAL(6, 84, 18), // "doImprimirPantalla"
QT_MOC_LITERAL(7, 103, 11), // "opcionTexto"
QT_MOC_LITERAL(8, 115, 11) // "MyTimerSlot"

    },
    "ejemplo1\0doButtonStop\0\0doButtonGetPeriodo\0"
    "doButtonSetPeriodo\0doButtonGetTimeElapsed\0"
    "doImprimirPantalla\0opcionTexto\0"
    "MyTimerSlot"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ejemplo1[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   44,    2, 0x0a /* Public */,
       3,    0,   45,    2, 0x0a /* Public */,
       4,    0,   46,    2, 0x0a /* Public */,
       5,    0,   47,    2, 0x0a /* Public */,
       6,    1,   48,    2, 0x0a /* Public */,
       8,    0,   51,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    7,
    QMetaType::Void,

       0        // eod
};

void ejemplo1::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ejemplo1 *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->doButtonStop(); break;
        case 1: _t->doButtonGetPeriodo(); break;
        case 2: _t->doButtonSetPeriodo(); break;
        case 3: _t->doButtonGetTimeElapsed(); break;
        case 4: _t->doImprimirPantalla((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->MyTimerSlot(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject ejemplo1::staticMetaObject = { {
    &QWidget::staticMetaObject,
    qt_meta_stringdata_ejemplo1.data,
    qt_meta_data_ejemplo1,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ejemplo1::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ejemplo1::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ejemplo1.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "Ui_Counter"))
        return static_cast< Ui_Counter*>(this);
    return QWidget::qt_metacast(_clname);
}

int ejemplo1::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
