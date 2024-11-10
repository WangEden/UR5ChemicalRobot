/****************************************************************************
** Meta object code from reading C++ file 'widget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "widget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'widget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Widget_t {
    QByteArrayData data[28];
    char stringdata0[330];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Widget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Widget_t qt_meta_stringdata_Widget = {
    {
QT_MOC_LITERAL(0, 0, 6), // "Widget"
QT_MOC_LITERAL(1, 7, 22), // "switchSerialPortStatus"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 21), // "flashSerialPortDevice"
QT_MOC_LITERAL(4, 53, 12), // "clearMessage"
QT_MOC_LITERAL(5, 66, 11), // "sendMessage"
QT_MOC_LITERAL(6, 78, 11), // "getBaudrate"
QT_MOC_LITERAL(7, 90, 5), // "index"
QT_MOC_LITERAL(8, 96, 11), // "getDataBits"
QT_MOC_LITERAL(9, 108, 9), // "getParity"
QT_MOC_LITERAL(10, 118, 11), // "getStopbits"
QT_MOC_LITERAL(11, 130, 9), // "clearEdit"
QT_MOC_LITERAL(12, 140, 11), // "showMessage"
QT_MOC_LITERAL(13, 152, 10), // "openCamera"
QT_MOC_LITERAL(14, 163, 11), // "closeCamera"
QT_MOC_LITERAL(15, 175, 14), // "setPawPosition"
QT_MOC_LITERAL(16, 190, 20), // "clearPawPositionEdit"
QT_MOC_LITERAL(17, 211, 15), // "readPawPosition"
QT_MOC_LITERAL(18, 227, 16), // "readPawIntensity"
QT_MOC_LITERAL(19, 244, 10), // "catchShelf"
QT_MOC_LITERAL(20, 255, 10), // "connectUR5"
QT_MOC_LITERAL(21, 266, 15), // "initUR5Position"
QT_MOC_LITERAL(22, 282, 5), // "UR5Up"
QT_MOC_LITERAL(23, 288, 7), // "UR5Down"
QT_MOC_LITERAL(24, 296, 8), // "UR5Front"
QT_MOC_LITERAL(25, 305, 7), // "UR5Back"
QT_MOC_LITERAL(26, 313, 7), // "UR5Left"
QT_MOC_LITERAL(27, 321, 8) // "UR5Right"

    },
    "Widget\0switchSerialPortStatus\0\0"
    "flashSerialPortDevice\0clearMessage\0"
    "sendMessage\0getBaudrate\0index\0getDataBits\0"
    "getParity\0getStopbits\0clearEdit\0"
    "showMessage\0openCamera\0closeCamera\0"
    "setPawPosition\0clearPawPositionEdit\0"
    "readPawPosition\0readPawIntensity\0"
    "catchShelf\0connectUR5\0initUR5Position\0"
    "UR5Up\0UR5Down\0UR5Front\0UR5Back\0UR5Left\0"
    "UR5Right"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Widget[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      25,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,  139,    2, 0x08 /* Private */,
       3,    0,  140,    2, 0x08 /* Private */,
       4,    0,  141,    2, 0x08 /* Private */,
       5,    0,  142,    2, 0x08 /* Private */,
       6,    1,  143,    2, 0x08 /* Private */,
       8,    1,  146,    2, 0x08 /* Private */,
       9,    1,  149,    2, 0x08 /* Private */,
      10,    1,  152,    2, 0x08 /* Private */,
      11,    0,  155,    2, 0x08 /* Private */,
      12,    0,  156,    2, 0x08 /* Private */,
      13,    0,  157,    2, 0x08 /* Private */,
      14,    0,  158,    2, 0x08 /* Private */,
      15,    0,  159,    2, 0x08 /* Private */,
      16,    0,  160,    2, 0x08 /* Private */,
      17,    0,  161,    2, 0x08 /* Private */,
      18,    0,  162,    2, 0x08 /* Private */,
      19,    0,  163,    2, 0x08 /* Private */,
      20,    0,  164,    2, 0x08 /* Private */,
      21,    0,  165,    2, 0x08 /* Private */,
      22,    0,  166,    2, 0x08 /* Private */,
      23,    0,  167,    2, 0x08 /* Private */,
      24,    0,  168,    2, 0x08 /* Private */,
      25,    0,  169,    2, 0x08 /* Private */,
      26,    0,  170,    2, 0x08 /* Private */,
      27,    0,  171,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    7,
    QMetaType::Void, QMetaType::Int,    7,
    QMetaType::Void, QMetaType::Int,    7,
    QMetaType::Void, QMetaType::Int,    7,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void Widget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Widget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->switchSerialPortStatus(); break;
        case 1: _t->flashSerialPortDevice(); break;
        case 2: _t->clearMessage(); break;
        case 3: _t->sendMessage(); break;
        case 4: _t->getBaudrate((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->getDataBits((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->getParity((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: _t->getStopbits((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->clearEdit(); break;
        case 9: _t->showMessage(); break;
        case 10: _t->openCamera(); break;
        case 11: _t->closeCamera(); break;
        case 12: _t->setPawPosition(); break;
        case 13: _t->clearPawPositionEdit(); break;
        case 14: _t->readPawPosition(); break;
        case 15: _t->readPawIntensity(); break;
        case 16: _t->catchShelf(); break;
        case 17: _t->connectUR5(); break;
        case 18: _t->initUR5Position(); break;
        case 19: _t->UR5Up(); break;
        case 20: _t->UR5Down(); break;
        case 21: _t->UR5Front(); break;
        case 22: _t->UR5Back(); break;
        case 23: _t->UR5Left(); break;
        case 24: _t->UR5Right(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject Widget::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_Widget.data,
    qt_meta_data_Widget,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *Widget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Widget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Widget.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int Widget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 25)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 25;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 25)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 25;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
