/****************************************************************************
** Meta object code from reading C++ file 'Function.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "Function.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Function.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Function_t {
    QByteArrayData data[16];
    char stringdata0[169];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Function_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Function_t qt_meta_stringdata_Function = {
    {
QT_MOC_LITERAL(0, 0, 8), // "Function"
QT_MOC_LITERAL(1, 9, 20), // "shelf_grasp_finished"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 19), // "init_grasp_finished"
QT_MOC_LITERAL(4, 51, 16), // "init_pose_change"
QT_MOC_LITERAL(5, 68, 5), // "index"
QT_MOC_LITERAL(6, 74, 11), // "shelf_grasp"
QT_MOC_LITERAL(7, 86, 7), // "Camera*"
QT_MOC_LITERAL(8, 94, 6), // "camera"
QT_MOC_LITERAL(9, 101, 4), // "UR5*"
QT_MOC_LITERAL(10, 106, 3), // "ur5"
QT_MOC_LITERAL(11, 110, 7), // "cv::Mat"
QT_MOC_LITERAL(12, 118, 6), // "matrix"
QT_MOC_LITERAL(13, 125, 8), // "recv_mat"
QT_MOC_LITERAL(14, 134, 17), // "init_UR5_position"
QT_MOC_LITERAL(15, 152, 16) // "change_init_pose"

    },
    "Function\0shelf_grasp_finished\0\0"
    "init_grasp_finished\0init_pose_change\0"
    "index\0shelf_grasp\0Camera*\0camera\0UR5*\0"
    "ur5\0cv::Mat\0matrix\0recv_mat\0"
    "init_UR5_position\0change_init_pose"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Function[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   49,    2, 0x06 /* Public */,
       3,    0,   50,    2, 0x06 /* Public */,
       4,    1,   51,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    3,   54,    2, 0x0a /* Public */,
      13,    1,   61,    2, 0x0a /* Public */,
      14,    1,   64,    2, 0x0a /* Public */,
      15,    1,   67,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    5,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 7, 0x80000000 | 9, 0x80000000 | 11,    8,   10,   12,
    QMetaType::Void, 0x80000000 | 11,    2,
    QMetaType::Void, 0x80000000 | 9,   10,
    QMetaType::Void, QMetaType::Int,    5,

       0        // eod
};

void Function::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Function *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->shelf_grasp_finished(); break;
        case 1: _t->init_grasp_finished(); break;
        case 2: _t->init_pose_change((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->shelf_grasp((*reinterpret_cast< Camera*(*)>(_a[1])),(*reinterpret_cast< UR5*(*)>(_a[2])),(*reinterpret_cast< cv::Mat(*)>(_a[3]))); break;
        case 4: _t->recv_mat((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 5: _t->init_UR5_position((*reinterpret_cast< UR5*(*)>(_a[1]))); break;
        case 6: _t->change_init_pose((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (Function::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Function::shelf_grasp_finished)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (Function::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Function::init_grasp_finished)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (Function::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Function::init_pose_change)) {
                *result = 2;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject Function::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_Function.data,
    qt_meta_data_Function,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *Function::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Function::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Function.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int Function::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void Function::shelf_grasp_finished()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void Function::init_grasp_finished()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void Function::init_pose_change(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
